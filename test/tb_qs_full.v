`default_nettype none
`timescale 1ns / 1ps

`include "sim_spi_ram.v"

/* Testbench for complete quadratic sieve factorization test.
   Includes SPI RAM model for actual sieve array storage and
   exposes internal signals for monitoring.
*/
module tb_qs_full #(
  parameter GLOBAL_ADDR_BITS = 20,  // Full sieve address (20-bit = 1MB total sieve)
  parameter STRIDE_BITS = 20,       // Stride/prime size (20-bit, supports primes up to ~1M)
  parameter LOCAL_ADDR_BITS = 12    // Default 4KB for faster tests
) ();

  // Dump the signals to a FST file
  initial begin
    $dumpfile("tb_qs_full.fst");
    $dumpvars(0, tb_qs_full);
    #1;
  end

  // Standard signals
  reg clk;
  reg rst_n;
  reg ena;
  reg [7:0] ui_in;
  reg [7:0] uio_in;
  wire [7:0] uo_out;
  wire [7:0] uio_out;
  wire [7:0] uio_oe;

  // Extract SPI signals
  wire spi_cs = uio_out[0];
  wire spi_sck = uio_out[1];
  wire spi_mosi = uio_out[2];
  wire spi_miso;

  // Connect MISO to uio_in[3]
  assign uio_in[3] = spi_miso;
  assign uio_in[7:4] = 4'b0;
  assign uio_in[2:0] = 3'b0;

  // Expose internal signals for monitoring and debugging
  // These allow the Python test to track the BP's internal state
  wire [GLOBAL_ADDR_BITS-1:0] internal_addr = user_project.bp_core.addr_reg;
  wire [STRIDE_BITS-1:0] internal_stride = user_project.bp_core.params_module.stride_out;
  wire [7:0] internal_lambda = user_project.bp_core.params_module.lambda_out;
  wire [7:0] internal_data = user_project.bp_core.data_reg;
  wire [3:0] internal_state = user_project.bp_core.state;
  wire [7:0] internal_threshold = user_project.bp_core.threshold_reg;

  // Expose report signals (global addresses)
  wire internal_report_valid = user_project.bp_core.bp_report_valid_out;
  wire [GLOBAL_ADDR_BITS-1:0] internal_report_addr = user_project.bp_core.bp_report_addr_out;

  // Expose SPI/RAM signals for debugging
  wire internal_spi_busy = user_project.spi_busy;
  wire internal_ram_done = user_project.ram_done;
  wire internal_ram_start = user_project.ram_start;

  // Instantiate DUT (Device Under Test)
  tt_um_swenson_cqs #(
      .GLOBAL_ADDR_BITS(GLOBAL_ADDR_BITS),
      .STRIDE_BITS(STRIDE_BITS),
      .LOCAL_ADDR_BITS(LOCAL_ADDR_BITS)
  ) user_project (
      .ui_in  (ui_in),    // Dedicated inputs
      .uo_out (uo_out),   // Dedicated outputs
      .uio_in (uio_in),   // IOs: Input path
      .uio_out(uio_out),  // IOs: Output path
      .uio_oe (uio_oe),   // IOs: Enable path
      .ena    (ena),      // enable
      .clk    (clk),      // clock
      .rst_n  (rst_n)     // reset_n
  );

  // SPI RAM model (configurable size)
  sim_spi_ram ram (
    .spi_clk(spi_sck),
    .spi_mosi(spi_mosi),
    .spi_select(spi_cs),
    .spi_miso(spi_miso),
    .debug_clk(),
    .debug_addr(),
    .debug_data()
  );

endmodule
