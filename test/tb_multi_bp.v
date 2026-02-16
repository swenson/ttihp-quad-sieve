`default_nettype none
`timescale 1ns / 1ps

/* Multi-BP testbench - Two block processors wired together in a pipeline.
   Tests parameter passing and result chaining across BP boundary.
*/
module tb_multi_bp #(
    parameter GLOBAL_ADDR_BITS = 20,  // Full sieve address (20-bit = 1MB total sieve)
    parameter STRIDE_BITS = 20,       // Stride/prime size (20-bit, supports primes up to ~1M)
    parameter LOCAL_ADDR_BITS = 16    // 64KB per BP
) ();

  // Dump the signals to a FST file
  initial begin
    $dumpfile("tb_multi_bp.fst");
    $dumpvars(0, tb_multi_bp);
    #1;
  end

  // Common signals
  reg clk;
  reg rst_n;
  reg [7:0] ui_in;
  reg [7:0] uio_in;

  // BP0 outputs
  wire [7:0] bp0_uo_out;
  wire [7:0] bp0_uio_out;
  wire [7:0] bp0_uio_oe;

  // BP1 outputs
  wire [7:0] bp1_uo_out;
  wire [7:0] bp1_uio_out;
  wire [7:0] bp1_uio_oe;

  // Inter-BP wires (BP0 → BP1) using global address widths
  wire bp0_param_valid_out;
  wire [GLOBAL_ADDR_BITS-1:0] bp0_addr_out;
  wire [STRIDE_BITS-1:0] bp0_stride_out;
  wire [7:0] bp0_lambda_out;
  wire bp0_param_output_valid;
  wire [7:0] bp0_param_output_data;

  wire bp0_report_valid_out;
  wire [GLOBAL_ADDR_BITS-1:0] bp0_report_addr_out;

  // ============================================================================
  // BP0 RAM model
  // ============================================================================
  wire [LOCAL_ADDR_BITS-1:0] bp0_ram_addr;
  wire [7:0] bp0_ram_wdata;
  wire [7:0] bp0_ram_rdata;
  wire bp0_ram_we;
  wire bp0_ram_start;
  reg bp0_ram_done;
  reg [7:0] bp0_ram_data [0:(1<<LOCAL_ADDR_BITS)-1];  // 64KB RAM

  // BP0 RAM behavior - single-cycle delay
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      bp0_ram_done <= 1;
    end else begin
      if (bp0_ram_start && bp0_ram_done) begin
        bp0_ram_done <= 0;
        if (bp0_ram_we) begin
          bp0_ram_data[bp0_ram_addr] <= bp0_ram_wdata;
        end
      end else if (!bp0_ram_done) begin
        bp0_ram_done <= 1;
      end
    end
  end
  assign bp0_ram_rdata = bp0_ram_data[bp0_ram_addr];

  // Instantiate BP0 (first in chain)
  block_processor_core #(
      .GLOBAL_ADDR_BITS(GLOBAL_ADDR_BITS),
      .STRIDE_BITS(STRIDE_BITS),
      .LOCAL_ADDR_BITS(LOCAL_ADDR_BITS)
  ) bp0 (
      .clk(clk),
      .rst_n(rst_n),
      .mode(ui_in[1:0]),
      .data_valid_in(ui_in[2]),
      .cmd_data_in(ui_in[7:3]),
      .busy_out(bp0_uo_out[0]),
      .valid_out(bp0_uo_out[1]),
      .report_valid(bp0_uo_out[2]),
      .status_out(bp0_uo_out[7:3]),

      // BP parameter output (serialized)
      .bp_param_output_valid(bp0_param_output_valid),
      .bp_param_output_data(bp0_param_output_data),

      // Generic RAM interface with simple sync RAM model
      .ram_addr(bp0_ram_addr),
      .ram_wdata(bp0_ram_wdata),
      .ram_rdata(bp0_ram_rdata),
      .ram_we(bp0_ram_we),
      .ram_start(bp0_ram_start),
      .ram_done(bp0_ram_done),

      // Inter-BP communication - BP0 has no left neighbor
      .bp_param_valid_in(1'b0),
      .bp_addr_in({GLOBAL_ADDR_BITS{1'b0}}),
      .bp_stride_in({STRIDE_BITS{1'b0}}),
      .bp_lambda_in(8'h00),
      .bp_param_valid_out(bp0_param_valid_out),
      .bp_addr_out(bp0_addr_out),
      .bp_stride_out(bp0_stride_out),
      .bp_lambda_out(bp0_lambda_out),

      .bp_report_valid_in(1'b0),
      .bp_report_addr_in({GLOBAL_ADDR_BITS{1'b0}}),
      .bp_report_valid_out(bp0_report_valid_out),
      .bp_report_addr_out(bp0_report_addr_out)
  );

  // ============================================================================
  // BP1 RAM model
  // ============================================================================
  wire [LOCAL_ADDR_BITS-1:0] bp1_ram_addr;
  wire [7:0] bp1_ram_wdata;
  wire [7:0] bp1_ram_rdata;
  wire bp1_ram_we;
  wire bp1_ram_start;
  reg bp1_ram_done;
  reg [7:0] bp1_ram_data [0:(1<<LOCAL_ADDR_BITS)-1];  // 64KB RAM

  // BP1 RAM behavior - single-cycle delay
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      bp1_ram_done <= 1;
    end else begin
      if (bp1_ram_start && bp1_ram_done) begin
        bp1_ram_done <= 0;
        if (bp1_ram_we) begin
          bp1_ram_data[bp1_ram_addr] <= bp1_ram_wdata;
        end
      end else if (!bp1_ram_done) begin
        bp1_ram_done <= 1;
      end
    end
  end
  assign bp1_ram_rdata = bp1_ram_data[bp1_ram_addr];

  // BP1 only receives serial input during INIT mode (for threshold loading)
  wire bp1_mode_init = (ui_in[1:0] == 2'b01);  // MODE_INIT = 01
  wire bp1_data_valid = bp1_mode_init ? ui_in[2] : 1'b0;
  wire [4:0] bp1_cmd_data = bp1_mode_init ? ui_in[7:3] : 5'b0;

  // Instantiate BP1 (second in chain, receives from BP0)
  block_processor_core #(
      .GLOBAL_ADDR_BITS(GLOBAL_ADDR_BITS),
      .STRIDE_BITS(STRIDE_BITS),
      .LOCAL_ADDR_BITS(LOCAL_ADDR_BITS)
  ) bp1 (
      .clk(clk),
      .rst_n(rst_n),
      .mode(ui_in[1:0]),  // Share mode signal
      .data_valid_in(bp1_data_valid),  // BP1 gets params from BP0 in SIEVE, but threshold in INIT
      .cmd_data_in(bp1_cmd_data),
      .busy_out(bp1_uo_out[0]),
      .valid_out(bp1_uo_out[1]),
      .report_valid(bp1_uo_out[2]),
      .status_out(bp1_uo_out[7:3]),

      // BP parameter output (serialized) - not connected for last BP
      .bp_param_output_valid(),
      .bp_param_output_data(),

      // Generic RAM interface with simple sync RAM model
      .ram_addr(bp1_ram_addr),
      .ram_wdata(bp1_ram_wdata),
      .ram_rdata(bp1_ram_rdata),
      .ram_we(bp1_ram_we),
      .ram_start(bp1_ram_start),
      .ram_done(bp1_ram_done),

      // Inter-BP communication - BP1 receives from BP0
      .bp_param_valid_in(bp0_param_valid_out),
      .bp_addr_in(bp0_addr_out),
      .bp_stride_in(bp0_stride_out),
      .bp_lambda_in(bp0_lambda_out),
      .bp_param_valid_out(),  // BP1 output not connected (last BP)
      .bp_addr_out(),
      .bp_stride_out(),
      .bp_lambda_out(),

      .bp_report_valid_in(bp0_report_valid_out),
      .bp_report_addr_in(bp0_report_addr_out),
      .bp_report_valid_out(),  // BP1 output not connected (last BP)
      .bp_report_addr_out()
  );

endmodule

`default_nettype wire
