/*
 * Copyright (c) 2026 Christopher Swenson
 * SPDX-License-Identifier: Apache-2.0
 *
 * Inter-BP Communication Module for Tiny Tapeout
 *
 * This module facilitates communication between adjacent BP ASICs using
 * the available uio[7:4] pins. It handles:
 *   - Receiving serialized parameters from left neighbor (uio input)
 *   - Receiving serialized reports from left neighbor (uio input)
 *   - Deserializing into parallel interface for block_processor_core
 *
 * Pin Assignment (uio[7:4] as inputs from left neighbor):
 *   uio[4] = from_left_param_valid  - High during param transmission
 *   uio[5] = from_left_report_valid - High during report transmission
 *   uio[6] = from_left_data[0]      - Serial data bit 0
 *   uio[7] = from_left_data[1]      - Serial data bit 1
 *
 * External Wiring (from left chip to this chip):
 *   Left uo_out[1] (param_valid)  -> This uio[4]
 *   Left uo_out[2] (report_valid) -> This uio[5]
 *   Left uo_out[3] (data[0])      -> This uio[6]
 *   Left uo_out[4] (data[1])      -> This uio[7]
 *
 * Note: This receives 2 bits per cycle from the left neighbor's 5-bit output.
 * The left neighbor must be configured to use the 2-bit serial protocol
 * (via its own inter-BP module) for compatible communication, OR the external
 * wiring accepts that only 2 of the 5 data bits are captured (slower transfer).
 *
 * Serial Protocol (2-bit data, LSB first):
 *   Parameters: addr[19:0], stride[19:0], lambda[7:0] = 48 bits = 24 cycles
 *   Reports:    addr[19:0] = 20 bits = 10 cycles
 */

`default_nettype none

module tt_um_swenson_inter_bp #(
    parameter GLOBAL_ADDR_BITS = 20,
    parameter STRIDE_BITS = 20,
    parameter LAMBDA_BITS = 8,
    parameter DATA_BITS_PER_CYCLE = 2  // Bits received per cycle from left
) (
    input wire clk,
    input wire rst_n,

    // UIO pins [7:4] - inputs from left neighbor
    input wire from_left_param_valid,   // uio_in[4]
    input wire from_left_report_valid,  // uio_in[5]
    input wire [DATA_BITS_PER_CYCLE-1:0] from_left_data, // uio_in[7:6]

    // Parallel interface to block_processor_core (from left neighbor, deserialized)
    output reg bp_param_valid_out,      // High when params available (stays high until ack)
    input wire bp_param_ack_in,         // Acknowledge from core when params consumed
    output reg [GLOBAL_ADDR_BITS-1:0] bp_addr_out,
    output reg [STRIDE_BITS-1:0] bp_stride_out,
    output reg [LAMBDA_BITS-1:0] bp_lambda_out,
    output reg bp_report_valid_out,     // Pulse when report complete
    output reg [GLOBAL_ADDR_BITS-1:0] bp_report_addr_out,

    // Optional: Re-serialize parallel output for symmetric 2-bit protocol
    // (for connecting to right neighbor via uio instead of uo_out)
    input wire serialize_param_valid_in,  // Trigger to start serializing
    input wire [GLOBAL_ADDR_BITS-1:0] serialize_addr_in,
    input wire [STRIDE_BITS-1:0] serialize_stride_in,
    input wire [LAMBDA_BITS-1:0] serialize_lambda_in,
    input wire serialize_report_valid_in, // Trigger to start serializing report
    input wire [GLOBAL_ADDR_BITS-1:0] serialize_report_addr_in,

    output wire to_right_param_valid,   // For uio_out (optional)
    output wire to_right_report_valid,  // For uio_out (optional)
    output wire [DATA_BITS_PER_CYCLE-1:0] to_right_data,  // For uio_out (optional)
    output wire serializing             // High while serializing output
);

    // Total bits for params and reports
    localparam PARAM_TOTAL_BITS = GLOBAL_ADDR_BITS + STRIDE_BITS + LAMBDA_BITS;  // 48
    localparam REPORT_TOTAL_BITS = GLOBAL_ADDR_BITS;  // 20

    // Cycles needed for each transmission
    localparam PARAM_CYCLES = (PARAM_TOTAL_BITS + DATA_BITS_PER_CYCLE - 1) / DATA_BITS_PER_CYCLE;  // 24
    localparam REPORT_CYCLES = (REPORT_TOTAL_BITS + DATA_BITS_PER_CYCLE - 1) / DATA_BITS_PER_CYCLE;  // 10

    // Counter width
    localparam COUNTER_BITS = 5;  // Enough for 24 cycles

    // =========================================================================
    // Receive from left neighbor (deserialization)
    // =========================================================================

    // Shift registers for receiving
    reg [PARAM_TOTAL_BITS-1:0] param_shift_reg;
    reg [REPORT_TOTAL_BITS-1:0] report_shift_reg;

    // Counters for tracking received bits
    reg [COUNTER_BITS-1:0] param_rx_counter;
    reg [COUNTER_BITS-1:0] report_rx_counter;

    // Previous valid signals for edge detection
    reg from_left_param_valid_prev;
    reg from_left_report_valid_prev;

    // Receiving state
    reg param_receiving;
    reg report_receiving;

    // State for parameter receiving
    reg param_rx_done;  // One cycle delay for clean extraction

    // 1-deep buffer for params (holds received params if output latch is busy)
    reg param_pending;  // True if we have params waiting in shift_reg
    reg [PARAM_TOTAL_BITS-1:0] param_pending_reg;  // Buffered params

    // Receive parameters from left neighbor
    // Strategy: Always capture incoming params. If output latch is busy, buffer them.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            param_shift_reg <= 0;
            param_rx_counter <= 0;
            param_receiving <= 0;
            param_rx_done <= 0;
            from_left_param_valid_prev <= 0;
            bp_param_valid_out <= 0;
            bp_addr_out <= 0;
            bp_stride_out <= 0;
            bp_lambda_out <= 0;
            param_pending <= 0;
            param_pending_reg <= 0;
        end else begin
            from_left_param_valid_prev <= from_left_param_valid;
            param_rx_done <= 0;

            // Handle acknowledge from core
            if (bp_param_ack_in) begin
                if (param_pending) begin
                    // Transfer pending params to output latch
                    bp_param_valid_out <= 1;
                    bp_addr_out <= param_pending_reg[GLOBAL_ADDR_BITS-1:0];
                    bp_stride_out <= param_pending_reg[GLOBAL_ADDR_BITS+STRIDE_BITS-1:GLOBAL_ADDR_BITS];
                    bp_lambda_out <= param_pending_reg[PARAM_TOTAL_BITS-1:GLOBAL_ADDR_BITS+STRIDE_BITS];
                    param_pending <= 0;
                end else begin
                    // Just clear the valid flag
                    bp_param_valid_out <= 0;
                end
            end

            if (param_rx_done) begin
                // Params fully received - try to transfer to output or buffer
                if (!bp_param_valid_out) begin
                    // Output latch is free - transfer immediately
                    bp_param_valid_out <= 1;
                    bp_addr_out <= param_shift_reg[GLOBAL_ADDR_BITS-1:0];
                    bp_stride_out <= param_shift_reg[GLOBAL_ADDR_BITS+STRIDE_BITS-1:GLOBAL_ADDR_BITS];
                    bp_lambda_out <= param_shift_reg[PARAM_TOTAL_BITS-1:GLOBAL_ADDR_BITS+STRIDE_BITS];
                end else begin
                    // Output latch is busy - buffer the params
                    param_pending <= 1;
                    param_pending_reg <= param_shift_reg;
                end
            end else if (from_left_param_valid && !from_left_param_valid_prev && !param_pending) begin
                // Rising edge - start receiving AND capture first data
                // Only start if we don't already have pending params (1-deep buffer limit)
                // Put first data in HIGH bits so subsequent shifts preserve it
                param_receiving <= 1;
                param_rx_counter <= 1;  // Already counting first cycle
                param_shift_reg <= {from_left_data, {(PARAM_TOTAL_BITS-DATA_BITS_PER_CYCLE){1'b0}}};
            end else if (param_receiving && from_left_param_valid) begin
                // Shift in data (LSB first) - new data goes to MSB, shifts down
                param_shift_reg <= {from_left_data, param_shift_reg[PARAM_TOTAL_BITS-1:DATA_BITS_PER_CYCLE]};
                param_rx_counter <= param_rx_counter + 1;

                if (param_rx_counter == PARAM_CYCLES - 1) begin
                    // All bits received - signal done for next cycle
                    param_receiving <= 0;
                    param_rx_done <= 1;
                end
            end else if (!from_left_param_valid && param_receiving) begin
                // Valid went low early - stop receiving (incomplete transmission)
                param_receiving <= 0;
            end
        end
    end

    // State for report receiving
    reg report_rx_done;  // One cycle delay for clean extraction

    // Receive reports from left neighbor
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            report_shift_reg <= 0;
            report_rx_counter <= 0;
            report_receiving <= 0;
            report_rx_done <= 0;
            from_left_report_valid_prev <= 0;
            bp_report_valid_out <= 0;
            bp_report_addr_out <= 0;
        end else begin
            from_left_report_valid_prev <= from_left_report_valid;
            bp_report_valid_out <= 0;  // Default: pulse is single cycle
            report_rx_done <= 0;

            if (report_rx_done) begin
                // Extract report address from completed shift register
                bp_report_valid_out <= 1;
                bp_report_addr_out <= report_shift_reg[GLOBAL_ADDR_BITS-1:0];
            end else if (from_left_report_valid && !from_left_report_valid_prev) begin
                // Rising edge - start receiving AND capture first data
                // Put first data in HIGH bits so subsequent shifts preserve it
                report_receiving <= 1;
                report_rx_counter <= 1;  // Already counting first cycle
                report_shift_reg <= {from_left_data, {(REPORT_TOTAL_BITS-DATA_BITS_PER_CYCLE){1'b0}}};
            end else if (report_receiving && from_left_report_valid) begin
                // Shift in data (LSB first)
                report_shift_reg <= {from_left_data, report_shift_reg[REPORT_TOTAL_BITS-1:DATA_BITS_PER_CYCLE]};
                report_rx_counter <= report_rx_counter + 1;

                if (report_rx_counter == REPORT_CYCLES - 1) begin
                    // All bits received - signal done for next cycle
                    report_receiving <= 0;
                    report_rx_done <= 1;
                end
            end else if (!from_left_report_valid && report_receiving) begin
                // Valid went low early - stop receiving
                report_receiving <= 0;
            end
        end
    end

    // =========================================================================
    // Serialize to right neighbor (optional - for symmetric 2-bit protocol)
    // =========================================================================

    // Shift registers for transmitting
    reg [PARAM_TOTAL_BITS-1:0] param_tx_shift_reg;
    reg [REPORT_TOTAL_BITS-1:0] report_tx_shift_reg;

    // Counters for tracking transmitted bits
    reg [COUNTER_BITS-1:0] param_tx_counter;
    reg [COUNTER_BITS-1:0] report_tx_counter;

    // Transmitting state
    reg param_transmitting;
    reg report_transmitting;

    // Edge detection for serialize triggers
    reg serialize_param_valid_prev;
    reg serialize_report_valid_prev;

    // Transmit parameters to right neighbor
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            param_tx_shift_reg <= 0;
            param_tx_counter <= 0;
            param_transmitting <= 0;
            serialize_param_valid_prev <= 0;
        end else begin
            serialize_param_valid_prev <= serialize_param_valid_in;

            if (serialize_param_valid_in && !serialize_param_valid_prev) begin
                // Rising edge - start transmitting
                param_transmitting <= 1;
                param_tx_counter <= 0;
                // Pack params into shift register: addr, stride, lambda (LSB first)
                param_tx_shift_reg <= {serialize_lambda_in, serialize_stride_in, serialize_addr_in};
            end else if (param_transmitting) begin
                // Shift out data (LSB first)
                param_tx_shift_reg <= {{DATA_BITS_PER_CYCLE{1'b0}}, param_tx_shift_reg[PARAM_TOTAL_BITS-1:DATA_BITS_PER_CYCLE]};
                param_tx_counter <= param_tx_counter + 1;

                if (param_tx_counter == PARAM_CYCLES - 1) begin
                    // All bits transmitted
                    param_transmitting <= 0;
                end
            end
        end
    end

    // Transmit reports to right neighbor
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            report_tx_shift_reg <= 0;
            report_tx_counter <= 0;
            report_transmitting <= 0;
            serialize_report_valid_prev <= 0;
        end else begin
            serialize_report_valid_prev <= serialize_report_valid_in;

            if (serialize_report_valid_in && !serialize_report_valid_prev) begin
                // Rising edge - start transmitting
                report_transmitting <= 1;
                report_tx_counter <= 0;
                report_tx_shift_reg <= serialize_report_addr_in;
            end else if (report_transmitting) begin
                // Shift out data (LSB first)
                report_tx_shift_reg <= {{DATA_BITS_PER_CYCLE{1'b0}}, report_tx_shift_reg[REPORT_TOTAL_BITS-1:DATA_BITS_PER_CYCLE]};
                report_tx_counter <= report_tx_counter + 1;

                if (report_tx_counter == REPORT_CYCLES - 1) begin
                    // All bits transmitted
                    report_transmitting <= 0;
                end
            end
        end
    end

    // Output assignments for serialized data
    assign to_right_param_valid = param_transmitting;
    assign to_right_report_valid = report_transmitting;
    assign to_right_data = param_transmitting ? param_tx_shift_reg[DATA_BITS_PER_CYCLE-1:0] :
                           report_transmitting ? report_tx_shift_reg[DATA_BITS_PER_CYCLE-1:0] :
                           {DATA_BITS_PER_CYCLE{1'b0}};
    assign serializing = param_transmitting | report_transmitting;

endmodule

`default_nettype wire
