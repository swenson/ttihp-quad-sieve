`default_nettype none
`timescale 1ns / 1ps

/* Multiplexed Pipeline Testbench
   Tests two BPs connected ONLY via external input/output wires,
   using the multiplexing protocol to pass parameters and reports.

   This verifies the complete multiplexing implementation:
   - BP0 receives parameters via ui_in/uio_in
   - BP0 outputs parameters via uo_out/uio_out (serialized over 5 cycles)
   - BP1 receives those parameters via its ui_in/uio_in
   - BP1 outputs results via uo_out
*/
module tb_mux_pipeline #(
    parameter GLOBAL_ADDR_BITS = 20,  // Full sieve address (20-bit = 1MB total sieve)
    parameter STRIDE_BITS = 20,       // Stride/prime size (20-bit, supports primes up to ~1M)
    parameter LOCAL_ADDR_BITS = 16    // 64KB per BP
) ();

  // Dump the signals to a FST file
  initial begin
    $dumpfile("tb_mux_pipeline.fst");
    $dumpvars(0, tb_mux_pipeline);
    #1;
  end

  // Common signals
  reg clk;
  reg rst_n;

  // BP0 inputs (from external controller)
  reg [7:0] bp0_ui_in = 0;
  reg [7:0] bp0_uio_in = 0;

  // BP0 outputs
  wire [7:0] bp0_uo_out;
  wire [7:0] bp0_uio_out;
  wire [7:0] bp0_uio_oe;

  // BP1 inputs (wired from BP0 outputs + control logic)
  wire [7:0] bp1_ui_in;
  wire [7:0] bp1_uio_in;

  // BP1 outputs
  wire [7:0] bp1_uo_out;
  wire [7:0] bp1_uio_out;
  wire [7:0] bp1_uio_oe;

  // Intermediate wires for BP parameter output
  wire bp0_param_output_valid;
  wire [7:0] bp0_param_output_data;
  wire [4:0] bp0_status_out;
  wire bp1_param_output_valid;
  wire [7:0] bp1_param_output_data;
  wire [4:0] bp1_status_out;

  // Debug: Expose internal state for debugging
  wire [3:0] bp0_state = bp0_core.state;
  wire [GLOBAL_ADDR_BITS-1:0] bp0_addr_reg = bp0_core.addr_reg;
  wire [STRIDE_BITS-1:0] bp0_stride_reg = bp0_core.params_module.stride_out;
  wire [7:0] bp0_lambda_reg = bp0_core.params_module.lambda_out;
  wire [3:0] bp0_load_counter = bp0_core.params_module.counter;

  // ============================================================================
  // Wiring BP0 → BP1 via multiplexed protocol
  // ============================================================================

  // Detect when BP0 is outputting parameters
  // BP0 sets bp_param_output_valid when sending parameter data
  wire bp0_sending_params = bp0_param_output_valid;

  // Detect when BP0 is outputting a report
  wire bp0_sending_report = bp0_uo_out[2];  // report_valid signal

  // BP1 mode selection based on BP0's outputs
  wire [1:0] bp1_mode = bp0_sending_params ? 2'b10 :  // SIEVE mode for params
                        bp0_sending_report ? 2'b11 :   // REPORT mode for reports
                        2'b00;                          // IDLE otherwise

  // BP1 data_valid is high when receiving params or reports
  wire bp1_data_valid = bp0_sending_params | bp0_sending_report;

  // Connect data paths
  assign bp1_ui_in[1:0] = bp1_mode;
  assign bp1_ui_in[2] = bp1_data_valid;
  assign bp1_ui_in[7:3] = bp0_uo_out[7:3];  // Data comes from BP0's output

  // uio_in for BP1: SPI MISO only (no BP parameter data)
  assign bp1_uio_in = 8'b0;  // All unused in this test (no SPI MISO connected)

  // ============================================================================
  // Simple synchronous RAM models for BP0 and BP1
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

  // ============================================================================
  // Instantiate BP0 (first in pipeline) - using core directly with simple RAM
  // ============================================================================
  block_processor_core #(
      .GLOBAL_ADDR_BITS(GLOBAL_ADDR_BITS),
      .STRIDE_BITS(STRIDE_BITS),
      .LOCAL_ADDR_BITS(LOCAL_ADDR_BITS)
  ) bp0_core (
      .clk(clk),
      .rst_n(rst_n),
      .mode(bp0_ui_in[1:0]),
      .data_valid_in(bp0_ui_in[2]),
      .cmd_data_in(bp0_ui_in[7:3]),
      .busy_out(bp0_uo_out[0]),
      .valid_out(bp0_uo_out[1]),
      .report_valid(bp0_uo_out[2]),
      .status_out(bp0_status_out),

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

      // Inter-BP communication - not used in mux test
      .bp_param_valid_in(1'b0),
      .bp_addr_in({GLOBAL_ADDR_BITS{1'b0}}),
      .bp_stride_in({STRIDE_BITS{1'b0}}),
      .bp_lambda_in(8'h00),
      .bp_param_valid_out(),
      .bp_addr_out(),
      .bp_stride_out(),
      .bp_lambda_out(),
      .bp_report_valid_in(1'b0),
      .bp_report_addr_in({GLOBAL_ADDR_BITS{1'b0}}),
      .bp_report_valid_out(),
      .bp_report_addr_out()
  );

  // BP0 output wiring - multiplex parameter data onto uo_out only (5 bits)
  assign bp0_uo_out[7:3] = bp0_param_output_valid ? bp0_param_output_data[4:0] : bp0_status_out;
  assign bp0_uio_out = 8'b0;  // uio is SPI-only, not used in this test
  assign bp0_uio_oe = 8'b0;   // All inputs

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

  // ============================================================================
  // Instantiate BP1 (second in pipeline) - using core directly with simple RAM
  // ============================================================================
  block_processor_core #(
      .GLOBAL_ADDR_BITS(GLOBAL_ADDR_BITS),
      .STRIDE_BITS(STRIDE_BITS),
      .LOCAL_ADDR_BITS(LOCAL_ADDR_BITS)
  ) bp1_core (
      .clk(clk),
      .rst_n(rst_n),
      .mode(bp1_ui_in[1:0]),
      .data_valid_in(bp1_ui_in[2]),
      .cmd_data_in(bp1_ui_in[7:3]),
      .busy_out(bp1_uo_out[0]),
      .valid_out(bp1_uo_out[1]),
      .report_valid(bp1_uo_out[2]),
      .status_out(bp1_status_out),

      // BP parameter output (serialized) - not monitored for BP1
      .bp_param_output_valid(bp1_param_output_valid),
      .bp_param_output_data(bp1_param_output_data),

      // Generic RAM interface with simple sync RAM model
      .ram_addr(bp1_ram_addr),
      .ram_wdata(bp1_ram_wdata),
      .ram_rdata(bp1_ram_rdata),
      .ram_we(bp1_ram_we),
      .ram_start(bp1_ram_start),
      .ram_done(bp1_ram_done),

      // Inter-BP communication - not used in mux test
      .bp_param_valid_in(1'b0),
      .bp_addr_in({GLOBAL_ADDR_BITS{1'b0}}),
      .bp_stride_in({STRIDE_BITS{1'b0}}),
      .bp_lambda_in(8'h00),
      .bp_param_valid_out(),
      .bp_addr_out(),
      .bp_stride_out(),
      .bp_lambda_out(),
      .bp_report_valid_in(1'b0),
      .bp_report_addr_in({GLOBAL_ADDR_BITS{1'b0}}),
      .bp_report_valid_out(),
      .bp_report_addr_out()
  );

  // BP1 output wiring - multiplex parameter data onto uo_out only (5 bits)
  assign bp1_uo_out[7:3] = bp1_param_output_valid ? bp1_param_output_data[4:0] : bp1_status_out;
  assign bp1_uio_out = 8'b0;  // uio is SPI-only, not used in this test
  assign bp1_uio_oe = 8'b0;   // All inputs

endmodule

`default_nettype wire
