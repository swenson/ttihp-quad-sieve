/*
 * Copyright (c) 2026 Christopher Swenson
 * SPDX-License-Identifier: Apache-2.0
 *
 * Block Processor Core for Quadratic Sieve Factoring
 *
 * RAM-Agnostic Implementation
 *
 * Based on "A Pipeline Architecture for Factoring Large Integers
 * with the Quadratic Sieve Algorithm" by Pomerance, Smith, and Tuler (1988)
 *
 * This module is designed to be RAM-agnostic. It uses a simple generic RAM
 * interface that can be adapted to various RAM implementations:
 *   - SPI RAM (e.g., 23LC512)
 *   - QSPI RAM
 *   - FPGA BRAM
 *   - External SRAM
 *
 * Global vs Local Addressing:
 *   - Global address space is GLOBAL_ADDR_BITS wide (32 bits in paper)
 *   - Each BP handles LOCAL_ADDR_BITS of address space (16 bits = 64KB)
 *   - If incoming global address >= LOCAL_SIZE, decrement by LOCAL_SIZE and forward
 *   - When sieving completes local range, forward remaining work to next BP
 *
 * RAM Interface Requirements:
 *   - ram_addr[LOCAL_ADDR_BITS-1:0] : Local address (16-bit = 64KB)
 *   - ram_wdata[7:0]  : 8-bit write data
 *   - ram_rdata[7:0]  : 8-bit read data
 *   - ram_we          : Write enable (1=write, 0=read)
 *   - ram_start       : Pulse high for 1 cycle to start operation
 *   - ram_done        : High when operation complete, data valid
 */

`default_nettype none

module block_processor_core #(
    parameter GLOBAL_ADDR_BITS = 20,  // Full sieve address (20-bit = 1MB total sieve)
    parameter STRIDE_BITS = 20,       // Stride/prime size (20-bit, supports primes up to ~1M)
    parameter LOCAL_ADDR_BITS = 16    // Local RAM address (16-bit = 64KB per BP)
    // Note: No BP_INDEX - all chips have identical RTL
    // Addresses are relative: addr < LOCAL_SIZE means process locally,
    // addr >= LOCAL_SIZE means forward to next BP with addr -= LOCAL_SIZE
) (
    input wire clk,
    input wire rst_n,

    // Control interface
    input wire [1:0] mode,
    input wire data_valid_in,
    input wire [4:0] cmd_data_in,
    output reg busy_out,
    output reg valid_out,
    output reg report_valid,
    output reg [4:0] status_out,

    // BP parameter output (serialized)
    output wire bp_param_output_valid,
    output wire [7:0] bp_param_output_data,

    // Generic RAM interface (active high, active on clock edge)
    output reg [LOCAL_ADDR_BITS-1:0] ram_addr,
    output reg [7:0] ram_wdata,
    input wire [7:0] ram_rdata,
    output reg ram_we,          // 1 = write, 0 = read
    output reg ram_start,       // Pulse to start operation
    input wire ram_done,        // Operation complete, data valid

    // Inter-BP communication - Parameter passing (uses global addresses)
    input wire bp_param_valid_in,
    input wire [GLOBAL_ADDR_BITS-1:0] bp_addr_in,
    input wire [STRIDE_BITS-1:0] bp_stride_in,
    input wire [7:0] bp_lambda_in,
    output wire bp_param_ack_out,       // Acknowledge when params are consumed from neighbor
    output wire bp_param_valid_out,
    output wire [GLOBAL_ADDR_BITS-1:0] bp_addr_out,
    output wire [STRIDE_BITS-1:0] bp_stride_out,
    output wire [7:0] bp_lambda_out,

    // Inter-BP communication - Result chaining (uses global addresses)
    input wire bp_report_valid_in,
    input wire [GLOBAL_ADDR_BITS-1:0] bp_report_addr_in,
    output reg bp_report_valid_out,
    output reg [GLOBAL_ADDR_BITS-1:0] bp_report_addr_out
);

    // Local address space size
    localparam [GLOBAL_ADDR_BITS-1:0] LOCAL_SIZE = (1 << LOCAL_ADDR_BITS);
    // Note: No RANGE_START/RANGE_END - all BPs are identical
    // Addresses are relative to this BP: 0 to LOCAL_SIZE-1 are local

    // Mode definitions
    localparam MODE_IDLE = 2'b00;
    localparam MODE_INIT = 2'b01;
    localparam MODE_SIEVE = 2'b10;
    localparam MODE_REPORT = 2'b11;

    // State machine
    localparam ST_IDLE = 4'd0;
    localparam ST_INIT_LOAD_THRESHOLD = 4'd1;
    localparam ST_INIT_START = 4'd2;
    localparam ST_INIT_WRITE = 4'd3;
    localparam ST_SIEVE_WAIT_PARAMS = 4'd4;
    localparam ST_SIEVE_CHECK_RANGE = 4'd5;  // New: check if address is in local range
    localparam ST_SIEVE_READ = 4'd6;
    localparam ST_SIEVE_READ_WAIT = 4'd7;
    localparam ST_SIEVE_WAIT_READ = 4'd8;
    localparam ST_SIEVE_CHECK = 4'd9;
    localparam ST_SIEVE_FORWARD = 4'd10;
    localparam ST_INIT_WAIT = 4'd11;  // Wait for SPI to start

    reg [3:0] state;
    reg [GLOBAL_ADDR_BITS-1:0] addr_reg;  // Current relative address (relative to this BP)
    reg [7:0] data_reg;      // Current data value
    reg [7:0] threshold_reg; // Threshold for reporting
    reg [LOCAL_ADDR_BITS-1:0] init_counter; // Counter for initialization
    reg threshold_load_counter; // Counter for loading threshold (0-1)
    reg initialized;

    // Signals for params module
    reg params_start_load;
    reg params_start_from_neighbor;
    reg params_forward;

    // Acknowledge when params consumed from neighbor (for inter_bp module)
    assign bp_param_ack_out = params_start_from_neighbor;
    wire params_loading;
    wire params_ready;
    wire params_done;
    wire [GLOBAL_ADDR_BITS-1:0] params_addr;
    wire [STRIDE_BITS-1:0] params_stride;
    wire [7:0] params_lambda;

    // Local address for RAM access (valid when addr_in_range is true)
    wire [LOCAL_ADDR_BITS-1:0] local_addr = addr_reg[LOCAL_ADDR_BITS-1:0];

    // Check if address is within this BP's local range [0, LOCAL_SIZE)
    // All addresses are relative - no BP_INDEX needed
    wire addr_in_range = (addr_reg < LOCAL_SIZE);

    // Compute next address after sieving
    wire [GLOBAL_ADDR_BITS-1:0] next_addr = addr_reg + {{(GLOBAL_ADDR_BITS-STRIDE_BITS){1'b0}}, params_stride};

    // For forwarding: subtract LOCAL_SIZE to convert to next BP's relative address
    // When skipping: forward (addr - LOCAL_SIZE) to next BP
    // When done sieving: forward (next_addr - LOCAL_SIZE) to next BP
    reg skip_mode;  // Set when we're forwarding due to out-of-range address
    reg [GLOBAL_ADDR_BITS-1:0] forward_addr_reg;  // Latched address to forward (already decremented)
    wire [GLOBAL_ADDR_BITS-1:0] forward_addr = forward_addr_reg;

    // Instantiate the parameter loading/forwarding module
    tt_um_swenson_params #(
        .GLOBAL_ADDR_BITS(GLOBAL_ADDR_BITS),
        .STRIDE_BITS(STRIDE_BITS),
        .LOCAL_ADDR_BITS(LOCAL_ADDR_BITS)
    ) params_module (
        .clk(clk),
        .rst_n(rst_n),
        .data_valid_in(data_valid_in),
        .cmd_data_in(cmd_data_in),

        // Loading control
        .start_load(params_start_load),
        .start_from_neighbor(params_start_from_neighbor),
        .loading(params_loading),
        .params_ready(params_ready),
        .params_done(params_done),

        // Loaded parameters output
        .addr_out(params_addr),
        .stride_out(params_stride),
        .lambda_out(params_lambda),

        // Inter-BP parameter input
        .bp_param_valid_in(bp_param_valid_in),
        .bp_addr_in(bp_addr_in),
        .bp_stride_in(bp_stride_in),
        .bp_lambda_in(bp_lambda_in),

        // Inter-BP parameter output
        .forward_params(params_forward),
        .forward_addr(forward_addr),
        .forward_stride(params_stride),
        .forward_lambda(params_lambda),
        .bp_param_valid_out(bp_param_valid_out),
        .bp_addr_out(bp_addr_out),
        .bp_stride_out(bp_stride_out),
        .bp_lambda_out(bp_lambda_out),

        // Serialized output
        .bp_param_output_valid(bp_param_output_valid),
        .bp_param_output_data(bp_param_output_data)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            busy_out <= 0;
            valid_out <= 0;
            report_valid <= 0;
            status_out <= 0;
            ram_start <= 0;
            ram_we <= 0;
            ram_addr <= 0;
            ram_wdata <= 0;
            addr_reg <= 0;
            data_reg <= 0;
            threshold_reg <= 8'd200;  // Default threshold (will be overwritten in MODE_INIT)
            init_counter <= 0;
            threshold_load_counter <= 0;
            bp_report_valid_out <= 0;
            bp_report_addr_out <= 0;
            initialized <= 0;
            params_start_load <= 0;
            params_start_from_neighbor <= 0;
            params_forward <= 0;
            skip_mode <= 0;
            forward_addr_reg <= 0;
        end else begin
            // Clear single-cycle signals every cycle (will be set if needed)
            report_valid <= 0;
            bp_report_valid_out <= 0;
            valid_out <= 0;
            ram_start <= 0;
            params_start_load <= 0;
            params_start_from_neighbor <= 0;
            params_forward <= 0;

            // Result forwarding - happens continuously regardless of mode/state
            // Add LOCAL_SIZE to convert local addresses to global as reports propagate
            // toward the host. Each BP adds one LOCAL_SIZE, so by the time a report
            // from BP_N reaches BP0, the address has been incremented N times.
            // For BP0's own reports: they are already global (BP0 starts at global 0)
            // and should be collected directly from BP0's output, not forwarded.
            if (bp_report_valid_in) begin
                report_valid <= 1;
                status_out <= bp_report_addr_in[4:0];
                bp_report_valid_out <= 1;
                bp_report_addr_out <= bp_report_addr_in + LOCAL_SIZE;
            end

            case (mode)
                MODE_IDLE: begin
                    state <= ST_IDLE;
                    busy_out <= 0;
                    valid_out <= 0;
                end

                MODE_INIT: begin
                    // First load threshold, then initialize RAM to 0
                    case (state)
                        ST_IDLE: begin
                            if (!initialized) begin
                                busy_out <= 1;
                                threshold_load_counter <= 0;
                                init_counter <= 0;
                                state <= ST_INIT_LOAD_THRESHOLD;
                            end
                        end

                        ST_INIT_LOAD_THRESHOLD: begin
                            // Load 8-bit threshold over 2 cycles (5 bits + 3 bits)
                            if (data_valid_in) begin
                                if (threshold_load_counter == 0) begin
                                    threshold_reg[4:0] <= cmd_data_in[4:0];
                                    threshold_load_counter <= 1;
                                end else begin
                                    threshold_reg[7:5] <= cmd_data_in[2:0];
                                    threshold_load_counter <= 0;
                                    state <= ST_INIT_START;
                                end
                            end
                        end

                        ST_INIT_START: begin
                            ram_addr <= init_counter;
                            ram_wdata <= 0;
                            ram_we <= 1;
                            ram_start <= 1;
                            state <= ST_INIT_WAIT;
                        end

                        ST_INIT_WAIT: begin
                            // Wait for SPI to start (ram_done goes low)
                            if (!ram_done) begin
                                state <= ST_INIT_WRITE;
                            end
                        end

                        ST_INIT_WRITE: begin
                            if (ram_done) begin
                                init_counter <= init_counter + 1;
                                if (init_counter == {LOCAL_ADDR_BITS{1'b1}}) begin
                                    state <= ST_IDLE;
                                    busy_out <= 0;
                                    initialized <= 1;
                                end else begin
                                    state <= ST_INIT_START;
                                end
                            end
                        end

                        default: state <= ST_IDLE;
                    endcase
                end

                MODE_SIEVE: begin
                    // Main sieving operation
                    case (state)
                        ST_IDLE: begin
                            busy_out <= 0;
                            valid_out <= 0;
                            skip_mode <= 0;

                            if (data_valid_in) begin
                                // Start loading parameters from serial input
                                busy_out <= 1;
                                params_start_load <= 1;
                                state <= ST_SIEVE_WAIT_PARAMS;
                            end else if (bp_param_valid_in) begin
                                // Receive parameters from neighbor BP
                                busy_out <= 1;
                                params_start_from_neighbor <= 1;
                                state <= ST_SIEVE_WAIT_PARAMS;
                            end
                        end

                        ST_SIEVE_WAIT_PARAMS: begin
                            // Wait for params module to finish loading
                            if (params_ready) begin
                                // Copy starting global address from module
                                addr_reg <= params_addr;
                                state <= ST_SIEVE_CHECK_RANGE;
                            end
                        end

                        ST_SIEVE_CHECK_RANGE: begin
                            // Check if address is within this BP's local range [0, LOCAL_SIZE)
                            if (!addr_in_range) begin
                                // Address is out of range - forward to next BP with addr decremented
                                skip_mode <= 1;
                                forward_addr_reg <= addr_reg - LOCAL_SIZE;  // Decrement for next BP
                                params_forward <= 1;
                                state <= ST_SIEVE_FORWARD;
                            end else begin
                                // Address is in range - proceed with sieving
                                skip_mode <= 0;
                                state <= ST_SIEVE_READ;
                            end
                        end

                        ST_SIEVE_READ: begin
                            if (ram_done) begin
                                ram_addr <= local_addr;
                                ram_we <= 0;
                                ram_start <= 1;
                                state <= ST_SIEVE_READ_WAIT;
                            end
                        end

                        ST_SIEVE_READ_WAIT: begin
                            if (!ram_done) begin
                                state <= ST_SIEVE_WAIT_READ;
                            end
                        end

                        ST_SIEVE_WAIT_READ: begin
                            // Wait for SPI completion
                            if (ram_done) begin
                                ram_addr <= local_addr;
                                ram_wdata <= ram_rdata + params_lambda;
                                data_reg <= ram_rdata + params_lambda;
                                ram_we <= 1;
                                ram_start <= 1;
                                state <= ST_SIEVE_CHECK;
                            end
                        end

                        ST_SIEVE_CHECK: begin
                            // Check if above threshold - report using local address
                            if (data_reg > threshold_reg) begin
                                report_valid <= 1;
                                status_out <= addr_reg[4:0];
                                bp_report_valid_out <= 1;
                                bp_report_addr_out <= addr_reg;  // Local address for this BP's report
                            end

                            // Update address: A <- A + stride
                            addr_reg <= next_addr;

                            // Check if next address falls outside this BP's local range
                            if (next_addr >= LOCAL_SIZE) begin
                                // Need to forward params to next BP with decremented address
                                skip_mode <= 0;  // Normal forward (not skip)
                                forward_addr_reg <= next_addr - LOCAL_SIZE;  // Decrement for next BP
                                params_forward <= 1;
                                state <= ST_SIEVE_FORWARD;
                            end else begin
                                state <= ST_SIEVE_READ;
                            end
                        end

                        ST_SIEVE_FORWARD: begin
                            // Wait for params module to finish forwarding
                            busy_out <= 1;
                            if (params_done) begin
                                valid_out <= 1;
                                state <= ST_IDLE;
                            end
                        end

                        default: state <= ST_IDLE;
                    endcase
                end

                MODE_REPORT: begin
                    state <= ST_IDLE;
                    busy_out <= 0;
                end

                default: begin
                    state <= ST_IDLE;
                    busy_out <= 0;
                end
            endcase
        end
    end

endmodule

`default_nettype wire
