/*
 * Copyright (c) 2026 Christopher Swenson
 * SPDX-License-Identifier: Apache-2.0
 *
 * Parameter Loading and Forwarding Module for Quadratic Sieve
 *
 * This module handles:
 *   - Loading sieve parameters (addr, stride, lambda) from 5-bit serial input
 *   - Receiving parameters from the previous BP in a pipeline
 *   - Forwarding parameters to the next BP (parallel and serialized interfaces)
 *
 * Based on the paper "A Pipeline Architecture for Factoring Large Integers
 * with the Quadratic Sieve Algorithm" by Pomerance, Smith, and Tuler (1988)
 *
 * Default parameter sizes:
 *   - Global address: 20 bits (1MB total sieve space across all BPs)
 *   - Stride: 20 bits (prime value, supports primes up to ~1M)
 *   - Lambda: 8 bits (log approximation)
 *
 * Parameters are loaded over multiple cycles at 5 bits per cycle.
 * For 20+20+8=48 bits: 10 cycles (clean alignment, no cross-field packing)
 */

`default_nettype none

module tt_um_swenson_params #(
    parameter GLOBAL_ADDR_BITS = 20,  // Full sieve address (20-bit = 1MB total sieve)
    parameter STRIDE_BITS = 20,       // Stride/prime size (20-bit, supports primes up to ~1M)
    parameter LOCAL_ADDR_BITS = 16    // Local RAM address (16-bit = 64KB per BP)
) (
    input wire clk,
    input wire rst_n,

    // Control interface
    input wire data_valid_in,
    input wire [4:0] cmd_data_in,

    // Loading control
    input wire start_load,           // Start loading from cmd_data_in
    input wire start_from_neighbor,  // Start from neighbor BP parameters
    output reg loading,              // Currently loading parameters
    output reg params_ready,         // Parameters loaded and ready to use
    output reg params_done,          // Finished forwarding, return to idle

    // Loaded parameters output (directly usable by sieve logic)
    output reg [GLOBAL_ADDR_BITS-1:0] addr_out,
    output reg [STRIDE_BITS-1:0] stride_out,
    output reg [7:0] lambda_out,

    // Inter-BP communication - Parameter input (from previous BP)
    input wire bp_param_valid_in,
    input wire [GLOBAL_ADDR_BITS-1:0] bp_addr_in,
    input wire [STRIDE_BITS-1:0] bp_stride_in,
    input wire [7:0] bp_lambda_in,

    // Inter-BP communication - Parameter output (to next BP)
    input wire forward_params,        // Trigger to forward params to next BP
    input wire [GLOBAL_ADDR_BITS-1:0] forward_addr,   // Current address to forward
    input wire [STRIDE_BITS-1:0] forward_stride,      // Current stride to forward
    input wire [7:0] forward_lambda,                  // Current lambda to forward
    output reg bp_param_valid_out,
    output wire [GLOBAL_ADDR_BITS-1:0] bp_addr_out,
    output wire [STRIDE_BITS-1:0] bp_stride_out,
    output wire [7:0] bp_lambda_out,

    // Serialized parameter output (alternative output format)
    output reg bp_param_output_valid,
    output reg [7:0] bp_param_output_data
);

    // Calculate number of load cycles needed
    // Total bits = GLOBAL_ADDR_BITS + STRIDE_BITS + 8 (lambda)
    localparam TOTAL_BITS = GLOBAL_ADDR_BITS + STRIDE_BITS + 8;
    localparam LOAD_CYCLES = (TOTAL_BITS + 4) / 5;  // Ceiling division

    // State machine
    localparam ST_IDLE = 2'd0;
    localparam ST_LOADING = 2'd1;
    localparam ST_READY = 2'd2;
    localparam ST_FORWARD = 2'd3;

    reg [1:0] state;
    reg [3:0] counter;    // Counter for loading/outputting parameters

    // bp_*_out are combinational pass-through from forward_* inputs
    // They're valid when bp_param_valid_out is high
    assign bp_addr_out = forward_addr;
    assign bp_stride_out = forward_stride;
    assign bp_lambda_out = forward_lambda;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            loading <= 0;
            params_ready <= 0;
            params_done <= 0;
            counter <= 0;
            addr_out <= 0;
            stride_out <= 0;
            lambda_out <= 0;
            bp_param_valid_out <= 0;
            bp_param_output_valid <= 0;
            bp_param_output_data <= 0;
        end else begin
            // Clear single-cycle signals
            params_ready <= 0;
            params_done <= 0;
            bp_param_output_valid <= 0;

            case (state)
                ST_IDLE: begin
                    loading <= 0;
                    bp_param_valid_out <= 0;
                    params_done <= 1;

                    if (start_load) begin
                        // Start loading from serial input
                        loading <= 1;
                        counter <= 0;
                        state <= ST_LOADING;
                    end else if (start_from_neighbor) begin
                        // Receive parameters from neighbor BP
                        // Note: bp_*_in values are held stable by the inter_bp module's
                        // output registers even after bp_param_valid_in goes low
                        addr_out <= bp_addr_in;
                        stride_out <= bp_stride_in;
                        lambda_out <= bp_lambda_in;
                        params_ready <= 1;
                        state <= ST_READY;
                    end
                end

                ST_LOADING: begin
                    // Load parameters over multiple cycles (5 bits/cycle)
                    // Bit layout for 20-bit addr, 20-bit stride, 8-bit lambda (48 bits total):
                    //   Cycle 0:  addr[4:0]
                    //   Cycle 1:  addr[9:5]
                    //   Cycle 2:  addr[14:10]
                    //   Cycle 3:  addr[19:15]
                    //   Cycle 4:  stride[4:0]
                    //   Cycle 5:  stride[9:5]
                    //   Cycle 6:  stride[14:10]
                    //   Cycle 7:  stride[19:15]
                    //   Cycle 8:  lambda[4:0]
                    //   Cycle 9:  lambda[7:5] (3 bits, upper 2 unused)
                    if (data_valid_in) begin
                        case (counter)
                            4'd0: begin
                                addr_out[4:0] <= cmd_data_in[4:0];
                                counter <= counter + 1;
                            end
                            4'd1: begin
                                addr_out[9:5] <= cmd_data_in[4:0];
                                counter <= counter + 1;
                            end
                            4'd2: begin
                                addr_out[14:10] <= cmd_data_in[4:0];
                                counter <= counter + 1;
                            end
                            4'd3: begin
                                addr_out[19:15] <= cmd_data_in[4:0];
                                counter <= counter + 1;
                            end
                            4'd4: begin
                                stride_out[4:0] <= cmd_data_in[4:0];
                                counter <= counter + 1;
                            end
                            4'd5: begin
                                stride_out[9:5] <= cmd_data_in[4:0];
                                counter <= counter + 1;
                            end
                            4'd6: begin
                                stride_out[14:10] <= cmd_data_in[4:0];
                                counter <= counter + 1;
                            end
                            4'd7: begin
                                stride_out[19:15] <= cmd_data_in[4:0];
                                counter <= counter + 1;
                            end
                            4'd8: begin
                                lambda_out[4:0] <= cmd_data_in[4:0];
                                counter <= counter + 1;
                            end
                            4'd9: begin
                                lambda_out[7:5] <= cmd_data_in[2:0];
                                counter <= 0;
                                loading <= 0;
                                params_ready <= 1;
                                state <= ST_READY;
                            end
                            default: begin
                                counter <= 0;
                                loading <= 0;
                                state <= ST_IDLE;
                            end
                        endcase
                    end
                end

                ST_READY: begin
                    // Parameters are ready, wait for forward request
                    if (forward_params) begin
                        // Forward parameters to next BP
                        // bp_*_out are combinational from forward_* inputs
                        bp_param_valid_out <= 1;
                        counter <= 0;
                        state <= ST_FORWARD;
                    end
                end

                ST_FORWARD: begin
                    // Output parameters serially from forward_* inputs
                    // Same bit layout as loading
                    bp_param_output_valid <= 1;

                    case (counter)
                        4'd0: begin
                            bp_param_output_data <= {3'b0, forward_addr[4:0]};
                            counter <= counter + 1;
                        end
                        4'd1: begin
                            bp_param_output_data <= {3'b0, forward_addr[9:5]};
                            counter <= counter + 1;
                        end
                        4'd2: begin
                            bp_param_output_data <= {3'b0, forward_addr[14:10]};
                            counter <= counter + 1;
                        end
                        4'd3: begin
                            bp_param_output_data <= {3'b0, forward_addr[19:15]};
                            counter <= counter + 1;
                        end
                        4'd4: begin
                            bp_param_output_data <= {3'b0, forward_stride[4:0]};
                            counter <= counter + 1;
                        end
                        4'd5: begin
                            bp_param_output_data <= {3'b0, forward_stride[9:5]};
                            counter <= counter + 1;
                        end
                        4'd6: begin
                            bp_param_output_data <= {3'b0, forward_stride[14:10]};
                            counter <= counter + 1;
                        end
                        4'd7: begin
                            bp_param_output_data <= {3'b0, forward_stride[19:15]};
                            counter <= counter + 1;
                        end
                        4'd8: begin
                            bp_param_output_data <= {3'b0, forward_lambda[4:0]};
                            counter <= counter + 1;
                        end
                        4'd9: begin
                            bp_param_output_data <= {5'b0, forward_lambda[7:5]};
                            counter <= 0;
                            state <= ST_IDLE;
                        end
                        default: begin
                            counter <= 0;
                            state <= ST_IDLE;
                        end
                    endcase
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
