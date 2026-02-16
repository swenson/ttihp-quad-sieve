/*
 * Copyright (c) 2026 Christopher Swenson
 * SPDX-License-Identifier: Apache-2.0
 *
 * Block Processor (BP) for Quadratic Sieve Factoring - Tiny Tapeout Edition
 * Based on "A Pipeline Architecture for Factoring Large Integers
 * with the Quadratic Sieve Algorithm" by Pomerance, Smith, and Tuler (1988)
 *
 * This implementation uses external SPI RAM for the 64KB sieve memory
 * to fit within Tiny Tapeout's constraints.
 *
 * Global vs Local Addressing:
 *   - Global address space is GLOBAL_ADDR_BITS wide (32 bits in paper)
 *   - Each BP handles LOCAL_ADDR_BITS of address space (16 bits = 64KB)
 *   - If incoming global address >= LOCAL_SIZE, decrement and forward
 */

`default_nettype none

// Include the RAM-agnostic block processor core
// `include "block_processor_core.v"
// `include "spi.v"

module tt_um_swenson_cqs #(
    parameter GLOBAL_ADDR_BITS = 20,  // Full sieve address (20-bit = 1MB total sieve)
    parameter STRIDE_BITS = 20,       // Stride/prime size (20-bit, supports primes up to ~1M)
    parameter LOCAL_ADDR_BITS = 16    // Local RAM address (16-bit = 64KB per BP)
    // Note: No BP_INDEX - all chips have identical RTL for silicon reuse
) (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

    // Pin assignments:
    // ui_in[1:0] - Mode select: 00=idle, 01=init, 10=sieve, 11=report
    // ui_in[2]   - Data valid input
    // ui_in[7:3] - Command/data input (5 bits)
    //              In MODE_REPORT: BP report address when data_valid=1
    //
    // uo_out[0]  - Busy signal
    // uo_out[1]  - Valid output / BP param output valid
    // uo_out[2]  - Report valid (BP report forwarding)
    // uo_out[7:3] - Status/data output (5 bits)
    //               Multiplexed with BP param output data[4:0]
    //
    // uio (bidirectional) - SPI RAM + Inter-BP communication:
    // uio[0] - SPI CS (chip select) - output
    // uio[1] - SPI SCK (clock) - output
    // uio[2] - SPI MOSI (master out) - output
    // uio[3] - SPI MISO (master in) - input
    // uio[4] - From left BP: param_valid (input) - connect to left chip's uo_out[1]
    // uio[5] - From left BP: report_valid (input) - connect to left chip's uo_out[2]
    // uio[6] - From left BP: data[0] (input) - connect to left chip's uo_out[3]
    // uio[7] - From left BP: data[1] (input) - connect to left chip's uo_out[4]

    // =========================================================================
    // Generic RAM interface (from block_processor_core)
    // =========================================================================
    wire [LOCAL_ADDR_BITS-1:0] ram_addr;
    wire [7:0] ram_wdata;
    wire [7:0] ram_rdata;
    wire ram_we;
    wire ram_start;
    wire ram_done;

    // =========================================================================
    // SPI RAM Controller Interface
    // =========================================================================
    wire spi_cs, spi_sck, spi_mosi, spi_miso;
    wire spi_busy;
    wire [7:0] spi_data_out;

    // Convert generic RAM interface to SPI controller interface
    wire spi_start_read = ram_start && !ram_we;
    wire spi_start_write = ram_start && ram_we;
    assign ram_done = !spi_busy;
    assign ram_rdata = spi_data_out;

    // =========================================================================
    // Inter-BP Communication Wires (using global address widths)
    // =========================================================================
    wire bp_param_valid_out, bp_param_valid_in;
    wire [GLOBAL_ADDR_BITS-1:0] bp_addr_out, bp_addr_in;
    wire [STRIDE_BITS-1:0] bp_stride_out, bp_stride_in;
    wire [7:0] bp_lambda_out, bp_lambda_in;
    wire bp_report_valid_out, bp_report_valid_in;
    wire [GLOBAL_ADDR_BITS-1:0] bp_report_addr_out, bp_report_addr_in;

    // =========================================================================
    // Inter-BP Communication Module (receives from left neighbor via uio[7:4])
    // =========================================================================
    // Pin mapping from left neighbor:
    //   Left uo_out[1] (param_valid)  -> This uio[4]
    //   Left uo_out[2] (report_valid) -> This uio[5]
    //   Left uo_out[3] (data[0])      -> This uio[6]
    //   Left uo_out[4] (data[1])      -> This uio[7]

    wire inter_bp_param_valid;
    wire [GLOBAL_ADDR_BITS-1:0] inter_bp_addr;
    wire [STRIDE_BITS-1:0] inter_bp_stride;
    wire [7:0] inter_bp_lambda;
    wire inter_bp_report_valid;
    wire [GLOBAL_ADDR_BITS-1:0] inter_bp_report_addr;
    wire bp_param_ack;  // Acknowledge from bp_core to inter_bp when params consumed

    // Optional: serialized output to right neighbor via uio (not used by default)
    wire to_right_param_valid_uio;
    wire to_right_report_valid_uio;
    wire [1:0] to_right_data_uio;
    wire inter_bp_serializing;

    tt_um_swenson_inter_bp #(
        .GLOBAL_ADDR_BITS(GLOBAL_ADDR_BITS),
        .STRIDE_BITS(STRIDE_BITS),
        .LAMBDA_BITS(8),
        .DATA_BITS_PER_CYCLE(2)
    ) inter_bp (
        .clk(clk),
        .rst_n(rst_n),

        // UIO pins [7:4] - inputs from left neighbor
        .from_left_param_valid(uio_in[4]),
        .from_left_report_valid(uio_in[5]),
        .from_left_data(uio_in[7:6]),

        // Deserialized outputs to BP core
        .bp_param_valid_out(inter_bp_param_valid),
        .bp_param_ack_in(bp_param_ack),
        .bp_addr_out(inter_bp_addr),
        .bp_stride_out(inter_bp_stride),
        .bp_lambda_out(inter_bp_lambda),
        .bp_report_valid_out(inter_bp_report_valid),
        .bp_report_addr_out(inter_bp_report_addr),

        // Serialized output to right (optional - could use uio or uo_out)
        .serialize_param_valid_in(bp_param_valid_out),
        .serialize_addr_in(bp_addr_out),
        .serialize_stride_in(bp_stride_out),
        .serialize_lambda_in(bp_lambda_out),
        .serialize_report_valid_in(bp_report_valid_out),
        .serialize_report_addr_in(bp_report_addr_out),

        .to_right_param_valid(to_right_param_valid_uio),
        .to_right_report_valid(to_right_report_valid_uio),
        .to_right_data(to_right_data_uio),
        .serializing(inter_bp_serializing)
    );

    // Input multiplexing:
    // BP parameters come from either:
    //   1. External serial load via ui_in (host commands)
    //   2. Left neighbor via uio[7:4] (inter-BP communication)
    assign bp_param_valid_in = inter_bp_param_valid;
    assign bp_addr_in = inter_bp_addr;
    assign bp_stride_in = inter_bp_stride;
    assign bp_lambda_in = inter_bp_lambda;

    // BP report injection - from left neighbor OR via MODE_REPORT (mode=11)
    wire mode_report_valid = (ui_in[1:0] == 2'b11) && ui_in[2];
    assign bp_report_valid_in = inter_bp_report_valid | mode_report_valid;
    assign bp_report_addr_in = inter_bp_report_valid ? inter_bp_report_addr :
                               mode_report_valid ? {{(GLOBAL_ADDR_BITS-5){1'b0}}, ui_in[7:3]} :
                               {GLOBAL_ADDR_BITS{1'b0}};

    // BP parameter output (serialized)
    wire bp_param_output_valid;
    wire [7:0] bp_param_output_data;

    // Internal signals from core
    wire core_busy, core_valid, core_report_valid;
    wire [4:0] core_status_out;

    // Output multiplexing
    // uo_out[1] must include bp_param_output_valid so downstream BPs can receive
    // serialized parameters via the uio inter-BP communication path
    assign uo_out[0] = core_busy;
    assign uo_out[1] = bp_param_output_valid | core_valid;  // Param output OR done signal
    assign uo_out[2] = core_report_valid;
    assign uo_out[7:3] = bp_param_output_valid ? bp_param_output_data[4:0] : core_status_out;

    // =========================================================================
    // Block Processor Core (RAM-agnostic)
    // =========================================================================
    block_processor_core #(
        .GLOBAL_ADDR_BITS(GLOBAL_ADDR_BITS),
        .STRIDE_BITS(STRIDE_BITS),
        .LOCAL_ADDR_BITS(LOCAL_ADDR_BITS)
    ) bp_core (
        .clk(clk),
        .rst_n(rst_n),

        // Control interface
        .mode(ui_in[1:0]),
        .data_valid_in(ui_in[2]),
        .cmd_data_in(ui_in[7:3]),
        .busy_out(core_busy),
        .valid_out(core_valid),
        .report_valid(core_report_valid),
        .status_out(core_status_out),
        .bp_param_output_valid(bp_param_output_valid),
        .bp_param_output_data(bp_param_output_data),

        // Generic RAM interface
        .ram_addr(ram_addr),
        .ram_wdata(ram_wdata),
        .ram_rdata(ram_rdata),
        .ram_we(ram_we),
        .ram_start(ram_start),
        .ram_done(ram_done),

        // Inter-BP communication
        .bp_param_valid_in(bp_param_valid_in),
        .bp_addr_in(bp_addr_in),
        .bp_stride_in(bp_stride_in),
        .bp_lambda_in(bp_lambda_in),
        .bp_param_ack_out(bp_param_ack),
        .bp_param_valid_out(bp_param_valid_out),
        .bp_addr_out(bp_addr_out),
        .bp_stride_out(bp_stride_out),
        .bp_lambda_out(bp_lambda_out),
        .bp_report_valid_in(bp_report_valid_in),
        .bp_report_addr_in(bp_report_addr_in),
        .bp_report_valid_out(bp_report_valid_out),
        .bp_report_addr_out(bp_report_addr_out)
    );

    // =========================================================================
    // SPI RAM Controller (for 23LC512 or similar 64KB SPI SRAM)
    // =========================================================================
    spi_ram_controller #(
        .DATA_WIDTH_BYTES(1),
        .ADDR_BITS(LOCAL_ADDR_BITS)
    ) spi_ctrl (
        .clk(clk),
        .rstn(rst_n),

        // Internal interface
        .addr_in(ram_addr),
        .data_in(ram_wdata),
        .start_read(spi_start_read),
        .start_write(spi_start_write),
        .data_out(spi_data_out),
        .busy(spi_busy),

        // External SPI interface
        .spi_miso(spi_miso),
        .spi_select(spi_cs),
        .spi_clk_out(spi_sck),
        .spi_mosi(spi_mosi)
    );

    // =========================================================================
    // Bidirectional Pin Assignments (SPI + Inter-BP Interface)
    // =========================================================================
    // uio[3:0] - SPI RAM interface
    assign uio_out[0] = spi_cs;
    assign uio_out[1] = spi_sck;
    assign uio_out[2] = spi_mosi;
    assign uio_out[3] = 1'b0;  // MISO is input only
    assign spi_miso = uio_in[3];

    // uio[7:4] - Inter-BP serialized output (2-bit protocol)
    // These signals go to the next BP in the chain:
    //   uio_out[4] = param_valid (to right neighbor)
    //   uio_out[5] = report_valid (to right neighbor)
    //   uio_out[6] = data[0] (to right neighbor)
    //   uio_out[7] = data[1] (to right neighbor)
    assign uio_out[4] = to_right_param_valid_uio;
    assign uio_out[5] = to_right_report_valid_uio;
    assign uio_out[6] = to_right_data_uio[0];
    assign uio_out[7] = to_right_data_uio[1];

    // Pin direction: CS, SCK, MOSI, and inter-BP outputs are outputs; MISO and inter-BP inputs are inputs
    assign uio_oe[2:0] = 3'b111;  // SPI outputs
    assign uio_oe[3] = 1'b0;      // SPI MISO input
    assign uio_oe[7:4] = 4'b1111; // Inter-BP outputs

    // Unused signals
    // Note: uio_in[7:4] are used by inter_bp module for left neighbor communication
    // Note: uio_out[7:4] carry inter_bp serialized output to right neighbor
    wire _unused = &{ena, uio_in[2:0], inter_bp_serializing, 1'b0};

endmodule

`default_nettype wire
