`default_nettype none
`timescale 1ns / 1ps

`include "sim_spi_ram.v"

/* Pipeline Testbench for Quadratic Sieve Factorization
   Two block processors in a pipeline, each with its own SPI RAM.
   BP0 handles local addresses 0x000-0xFFF (4KB) - using LOCAL_ADDR_BITS=12
   BP1 handles local addresses 0x000-0xFFF (4KB) - continuation after BP0 wraps
   Total sieve array: 8KB

   When BP0's global address exceeds 4KB (LOCAL_SIZE), it passes parameters to BP1
   which continues sieving in its own 4KB region. Using smaller regions for faster
   simulation while still verifying the pipeline architecture.
*/
module tb_qs_pipeline #(
    parameter GLOBAL_ADDR_BITS = 20,  // Full sieve address (20-bit = 1MB total sieve)
    parameter STRIDE_BITS = 20,       // Stride/prime size (20-bit, supports primes up to ~1M)
    parameter LOCAL_ADDR_BITS = 12    // 4KB per BP for faster simulation
) ();

  // Dump the signals to a FST file
  initial begin
    $dumpfile("tb_qs_pipeline.fst");
    $dumpvars(0, tb_qs_pipeline);
    #1;
  end

  // Common signals
  reg clk;
  reg rst_n;
  reg [7:0] ui_in;

  // BP0 outputs
  wire [7:0] bp0_uo_out;
  wire bp0_busy = bp0_uo_out[0];
  wire bp0_valid = bp0_uo_out[1];
  wire bp0_report_valid = bp0_uo_out[2];
  wire [4:0] bp0_status = bp0_uo_out[7:3];

  // BP1 outputs
  wire [7:0] bp1_uo_out;
  wire bp1_busy = bp1_uo_out[0];
  wire bp1_valid = bp1_uo_out[1];
  wire bp1_report_valid = bp1_uo_out[2];
  wire [4:0] bp1_status = bp1_uo_out[7:3];

  // Inter-BP parameter wires (BP0 -> BP1) using global address widths
  wire bp0_param_valid_out;
  wire [GLOBAL_ADDR_BITS-1:0] bp0_addr_out;
  wire [STRIDE_BITS-1:0] bp0_stride_out;
  wire [7:0] bp0_lambda_out;
  wire bp0_param_output_valid;
  wire [7:0] bp0_param_output_data;

  // Inter-BP report wires (BP0 -> BP1) using global addresses
  wire bp0_report_valid_out;
  wire [GLOBAL_ADDR_BITS-1:0] bp0_report_addr_out;

  // Final report output from BP1
  wire bp1_report_valid_out;
  wire [GLOBAL_ADDR_BITS-1:0] bp1_report_addr_out;

  // Combined report signals (from BP1, which includes BP0's reports)
  wire final_report_valid = bp1_report_valid_out;
  wire [GLOBAL_ADDR_BITS-1:0] final_report_addr = bp1_report_addr_out;

  // Also expose BP0's direct reports for tracking which BP generated what
  wire bp0_direct_report_valid = bp0_report_valid_out;
  wire [GLOBAL_ADDR_BITS-1:0] bp0_direct_report_addr = bp0_report_addr_out;

  // ============================================================================
  // BP0 SPI RAM
  // ============================================================================
  wire bp0_spi_cs, bp0_spi_sck, bp0_spi_mosi, bp0_spi_miso;
  wire bp0_spi_busy;
  wire [7:0] bp0_spi_data_out;

  wire [LOCAL_ADDR_BITS-1:0] bp0_ram_addr;
  wire [7:0] bp0_ram_wdata;
  wire [7:0] bp0_ram_rdata;
  wire bp0_ram_we;
  wire bp0_ram_start;
  wire bp0_ram_done;

  // SPI RAM controller for BP0
  spi_ram_controller #(
      .DATA_WIDTH_BYTES(1),
      .ADDR_BITS(LOCAL_ADDR_BITS)
  ) bp0_spi_ctrl (
      .clk(clk),
      .rstn(rst_n),
      .addr_in(bp0_ram_addr),
      .data_in(bp0_ram_wdata),
      .start_read(bp0_ram_start && !bp0_ram_we),
      .start_write(bp0_ram_start && bp0_ram_we),
      .data_out(bp0_spi_data_out),
      .busy(bp0_spi_busy),
      .spi_miso(bp0_spi_miso),
      .spi_select(bp0_spi_cs),
      .spi_clk_out(bp0_spi_sck),
      .spi_mosi(bp0_spi_mosi)
  );

  assign bp0_ram_done = !bp0_spi_busy;
  assign bp0_ram_rdata = bp0_spi_data_out;

  // SPI RAM model for BP0
  sim_spi_ram bp0_ram (
      .spi_clk(bp0_spi_sck),
      .spi_mosi(bp0_spi_mosi),
      .spi_select(bp0_spi_cs),
      .spi_miso(bp0_spi_miso),
      .debug_clk(clk),
      .debug_addr(24'b0),
      .debug_data()
  );

  // ============================================================================
  // BP0 - First block processor (4KB region for faster simulation)
  // ============================================================================
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

      // RAM interface
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
  // BP1 SPI RAM
  // ============================================================================
  wire bp1_spi_cs, bp1_spi_sck, bp1_spi_mosi, bp1_spi_miso;
  wire bp1_spi_busy;
  wire [7:0] bp1_spi_data_out;

  wire [LOCAL_ADDR_BITS-1:0] bp1_ram_addr;
  wire [7:0] bp1_ram_wdata;
  wire [7:0] bp1_ram_rdata;
  wire bp1_ram_we;
  wire bp1_ram_start;
  wire bp1_ram_done;

  // SPI RAM controller for BP1
  spi_ram_controller #(
      .DATA_WIDTH_BYTES(1),
      .ADDR_BITS(LOCAL_ADDR_BITS)
  ) bp1_spi_ctrl (
      .clk(clk),
      .rstn(rst_n),
      .addr_in(bp1_ram_addr),
      .data_in(bp1_ram_wdata),
      .start_read(bp1_ram_start && !bp1_ram_we),
      .start_write(bp1_ram_start && bp1_ram_we),
      .data_out(bp1_spi_data_out),
      .busy(bp1_spi_busy),
      .spi_miso(bp1_spi_miso),
      .spi_select(bp1_spi_cs),
      .spi_clk_out(bp1_spi_sck),
      .spi_mosi(bp1_spi_mosi)
  );

  assign bp1_ram_done = !bp1_spi_busy;
  assign bp1_ram_rdata = bp1_spi_data_out;

  // SPI RAM model for BP1
  sim_spi_ram bp1_ram (
      .spi_clk(bp1_spi_sck),
      .spi_mosi(bp1_spi_mosi),
      .spi_select(bp1_spi_cs),
      .spi_miso(bp1_spi_miso),
      .debug_clk(clk),
      .debug_addr(24'b0),
      .debug_data()
  );

  // ============================================================================
  // BP1 - Second block processor (4KB region)
  // ============================================================================
  // BP1 only receives serial input during INIT mode (for threshold loading)
  // During SIEVE mode, BP1 gets parameters from BP0 via inter-BP connection
  wire bp1_mode_init = (ui_in[1:0] == 2'b01);  // MODE_INIT = 01
  wire bp1_data_valid = bp1_mode_init ? ui_in[2] : 1'b0;
  wire [4:0] bp1_cmd_data = bp1_mode_init ? ui_in[7:3] : 5'b0;

  block_processor_core #(
      .GLOBAL_ADDR_BITS(GLOBAL_ADDR_BITS),
      .STRIDE_BITS(STRIDE_BITS),
      .LOCAL_ADDR_BITS(LOCAL_ADDR_BITS)
  ) bp1 (
      .clk(clk),
      .rst_n(rst_n),
      .mode(ui_in[1:0]),
      .data_valid_in(bp1_data_valid),
      .cmd_data_in(bp1_cmd_data),
      .busy_out(bp1_uo_out[0]),
      .valid_out(bp1_uo_out[1]),
      .report_valid(bp1_uo_out[2]),
      .status_out(bp1_uo_out[7:3]),

      // BP parameter output (serialized) - not used in this test
      .bp_param_output_valid(),
      .bp_param_output_data(),

      // RAM interface
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
      .bp_param_valid_out(),  // BP1 has no right neighbor in this test
      .bp_addr_out(),
      .bp_stride_out(),
      .bp_lambda_out(),

      // Report chain: BP1 receives BP0's reports and outputs combined reports
      .bp_report_valid_in(bp0_report_valid_out),
      .bp_report_addr_in(bp0_report_addr_out),
      .bp_report_valid_out(bp1_report_valid_out),
      .bp_report_addr_out(bp1_report_addr_out)
  );

  // ============================================================================
  // Expose internal signals for Python test
  // ============================================================================

  // BP0 internal state
  wire [GLOBAL_ADDR_BITS-1:0] bp0_internal_addr = bp0.addr_reg;
  wire [STRIDE_BITS-1:0] bp0_internal_stride = bp0.params_module.stride_out;
  wire [7:0] bp0_internal_lambda = bp0.params_module.lambda_out;
  wire [3:0] bp0_internal_state = bp0.state;
  wire [7:0] bp0_internal_threshold = bp0.threshold_reg;

  // BP1 internal state
  wire [GLOBAL_ADDR_BITS-1:0] bp1_internal_addr = bp1.addr_reg;
  wire [STRIDE_BITS-1:0] bp1_internal_stride = bp1.params_module.stride_out;
  wire [7:0] bp1_internal_lambda = bp1.params_module.lambda_out;
  wire [3:0] bp1_internal_state = bp1.state;
  wire [7:0] bp1_internal_threshold = bp1.threshold_reg;

  // Combined busy - either BP is working
  wire pipeline_busy = bp0_busy | bp1_busy;

endmodule

`default_nettype wire
