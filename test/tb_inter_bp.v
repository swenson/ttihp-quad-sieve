`default_nettype none
`timescale 1ns / 1ps

`include "sim_spi_ram.v"

/* Testbench for Inter-BP Communication via UIO Pins

   Four complete tt_um_swenson_cqs instances wired together:
   - BP0's uio_out[4:7] connects to BP1's uio_in[4:7]
   - BP1's uio_out[4:7] connects to BP2's uio_in[4:7]
   - BP2's uio_out[4:7] connects to BP3's uio_in[4:7]

   Each BP has 1KB of SPI RAM (LOCAL_ADDR_BITS=10), so the total sieve
   array is 4KB across the four BPs, matching the original single-BP test size.

   This tests the actual ASIC-to-ASIC communication mechanism using
   the 2-bit serial protocol over the uio pins.
*/
module tb_inter_bp #(
    parameter GLOBAL_ADDR_BITS = 20,
    parameter STRIDE_BITS = 20,
    parameter LOCAL_ADDR_BITS = 10  // 1KB per BP (4 BPs = 4KB total)
) ();

    // Dump the signals to a FST file
    initial begin
        $dumpfile("tb_inter_bp.fst");
        $dumpvars(0, tb_inter_bp);
        #1;
    end

    // Common signals
    reg clk;
    reg rst_n;
    reg ena;

    // Host interface (directly to BP0)
    reg [7:0] ui_in;

    // =========================================================================
    // BP0 - First block processor (receives host commands)
    // =========================================================================
    wire [7:0] bp0_uo_out;
    wire [7:0] bp0_uio_out;
    wire [7:0] bp0_uio_oe;
    reg [7:0] bp0_uio_in;

    // BP0 SPI RAM signals
    wire bp0_spi_cs = bp0_uio_out[0];
    wire bp0_spi_sck = bp0_uio_out[1];
    wire bp0_spi_mosi = bp0_uio_out[2];
    wire bp0_spi_miso;

    // Connect MISO and tie unused uio inputs low (BP0 has no left neighbor)
    always @(*) begin
        bp0_uio_in = 8'b0;
        bp0_uio_in[3] = bp0_spi_miso;
        // uio[7:4] = 0 (no left neighbor)
    end

    tt_um_swenson_cqs #(
        .GLOBAL_ADDR_BITS(GLOBAL_ADDR_BITS),
        .STRIDE_BITS(STRIDE_BITS),
        .LOCAL_ADDR_BITS(LOCAL_ADDR_BITS)
    ) bp0 (
        .ui_in(ui_in),
        .uo_out(bp0_uo_out),
        .uio_in(bp0_uio_in),
        .uio_out(bp0_uio_out),
        .uio_oe(bp0_uio_oe),
        .ena(ena),
        .clk(clk),
        .rst_n(rst_n)
    );

    sim_spi_ram bp0_ram (
        .spi_clk(bp0_spi_sck),
        .spi_mosi(bp0_spi_mosi),
        .spi_select(bp0_spi_cs),
        .spi_miso(bp0_spi_miso),
        .debug_clk(clk),
        .debug_addr(24'b0),
        .debug_data()
    );

    // =========================================================================
    // BP1 - Second block processor (receives from BP0 via uio)
    // =========================================================================
    wire [7:0] bp1_uo_out;
    wire [7:0] bp1_uio_out;
    wire [7:0] bp1_uio_oe;
    reg [7:0] bp1_uio_in;
    reg [7:0] bp1_ui_in;

    // BP1 SPI RAM signals
    wire bp1_spi_cs = bp1_uio_out[0];
    wire bp1_spi_sck = bp1_uio_out[1];
    wire bp1_spi_mosi = bp1_uio_out[2];
    wire bp1_spi_miso;

    // Connect BP0's uio output to BP1's uio input
    // Wiring: BP0 uio_out[4:7] -> BP1 uio_in[4:7] (2-bit serial protocol)
    always @(*) begin
        bp1_uio_in = 8'b0;
        bp1_uio_in[3] = bp1_spi_miso;
        bp1_uio_in[4] = bp0_uio_out[4];  // param_valid
        bp1_uio_in[5] = bp0_uio_out[5];  // report_valid
        bp1_uio_in[6] = bp0_uio_out[6];  // data[0]
        bp1_uio_in[7] = bp0_uio_out[7];  // data[1]
    end

    // BP1 receives full ui_in from host during INIT mode (for threshold loading)
    // During SIEVE mode, BP1 receives only mode bits (params come via inter-BP)
    always @(*) begin
        if (ui_in[1:0] == 2'b01) begin  // MODE_INIT
            bp1_ui_in = ui_in;  // Full ui_in for threshold loading
        end else begin
            bp1_ui_in = {5'b0, 1'b0, ui_in[1:0]};  // Mode only, data_valid=0
        end
    end

    tt_um_swenson_cqs #(
        .GLOBAL_ADDR_BITS(GLOBAL_ADDR_BITS),
        .STRIDE_BITS(STRIDE_BITS),
        .LOCAL_ADDR_BITS(LOCAL_ADDR_BITS)
    ) bp1 (
        .ui_in(bp1_ui_in),
        .uo_out(bp1_uo_out),
        .uio_in(bp1_uio_in),
        .uio_out(bp1_uio_out),
        .uio_oe(bp1_uio_oe),
        .ena(ena),
        .clk(clk),
        .rst_n(rst_n)
    );

    sim_spi_ram bp1_ram (
        .spi_clk(bp1_spi_sck),
        .spi_mosi(bp1_spi_mosi),
        .spi_select(bp1_spi_cs),
        .spi_miso(bp1_spi_miso),
        .debug_clk(clk),
        .debug_addr(24'b0),
        .debug_data()
    );

    // =========================================================================
    // BP2 - Third block processor (receives from BP1 via uio)
    // =========================================================================
    wire [7:0] bp2_uo_out;
    wire [7:0] bp2_uio_out;
    wire [7:0] bp2_uio_oe;
    reg [7:0] bp2_uio_in;
    reg [7:0] bp2_ui_in;

    // BP2 SPI RAM signals
    wire bp2_spi_cs = bp2_uio_out[0];
    wire bp2_spi_sck = bp2_uio_out[1];
    wire bp2_spi_mosi = bp2_uio_out[2];
    wire bp2_spi_miso;

    // Connect BP1's uio output to BP2's uio input
    always @(*) begin
        bp2_uio_in = 8'b0;
        bp2_uio_in[3] = bp2_spi_miso;
        bp2_uio_in[4] = bp1_uio_out[4];  // param_valid
        bp2_uio_in[5] = bp1_uio_out[5];  // report_valid
        bp2_uio_in[6] = bp1_uio_out[6];  // data[0]
        bp2_uio_in[7] = bp1_uio_out[7];  // data[1]
    end

    // BP2 receives full ui_in from host during INIT mode (for threshold loading)
    always @(*) begin
        if (ui_in[1:0] == 2'b01) begin  // MODE_INIT
            bp2_ui_in = ui_in;
        end else begin
            bp2_ui_in = {5'b0, 1'b0, ui_in[1:0]};
        end
    end

    tt_um_swenson_cqs #(
        .GLOBAL_ADDR_BITS(GLOBAL_ADDR_BITS),
        .STRIDE_BITS(STRIDE_BITS),
        .LOCAL_ADDR_BITS(LOCAL_ADDR_BITS)
    ) bp2 (
        .ui_in(bp2_ui_in),
        .uo_out(bp2_uo_out),
        .uio_in(bp2_uio_in),
        .uio_out(bp2_uio_out),
        .uio_oe(bp2_uio_oe),
        .ena(ena),
        .clk(clk),
        .rst_n(rst_n)
    );

    sim_spi_ram bp2_ram (
        .spi_clk(bp2_spi_sck),
        .spi_mosi(bp2_spi_mosi),
        .spi_select(bp2_spi_cs),
        .spi_miso(bp2_spi_miso),
        .debug_clk(clk),
        .debug_addr(24'b0),
        .debug_data()
    );

    // =========================================================================
    // BP3 - Fourth block processor (receives from BP2 via uio)
    // =========================================================================
    wire [7:0] bp3_uo_out;
    wire [7:0] bp3_uio_out;
    wire [7:0] bp3_uio_oe;
    reg [7:0] bp3_uio_in;
    reg [7:0] bp3_ui_in;

    // BP3 SPI RAM signals
    wire bp3_spi_cs = bp3_uio_out[0];
    wire bp3_spi_sck = bp3_uio_out[1];
    wire bp3_spi_mosi = bp3_uio_out[2];
    wire bp3_spi_miso;

    // Connect BP2's uio output to BP3's uio input
    always @(*) begin
        bp3_uio_in = 8'b0;
        bp3_uio_in[3] = bp3_spi_miso;
        bp3_uio_in[4] = bp2_uio_out[4];  // param_valid
        bp3_uio_in[5] = bp2_uio_out[5];  // report_valid
        bp3_uio_in[6] = bp2_uio_out[6];  // data[0]
        bp3_uio_in[7] = bp2_uio_out[7];  // data[1]
    end

    // BP3 receives full ui_in from host during INIT mode (for threshold loading)
    always @(*) begin
        if (ui_in[1:0] == 2'b01) begin  // MODE_INIT
            bp3_ui_in = ui_in;
        end else begin
            bp3_ui_in = {5'b0, 1'b0, ui_in[1:0]};
        end
    end

    tt_um_swenson_cqs #(
        .GLOBAL_ADDR_BITS(GLOBAL_ADDR_BITS),
        .STRIDE_BITS(STRIDE_BITS),
        .LOCAL_ADDR_BITS(LOCAL_ADDR_BITS)
    ) bp3 (
        .ui_in(bp3_ui_in),
        .uo_out(bp3_uo_out),
        .uio_in(bp3_uio_in),
        .uio_out(bp3_uio_out),
        .uio_oe(bp3_uio_oe),
        .ena(ena),
        .clk(clk),
        .rst_n(rst_n)
    );

    sim_spi_ram bp3_ram (
        .spi_clk(bp3_spi_sck),
        .spi_mosi(bp3_spi_mosi),
        .spi_select(bp3_spi_cs),
        .spi_miso(bp3_spi_miso),
        .debug_clk(clk),
        .debug_addr(24'b0),
        .debug_data()
    );

    // =========================================================================
    // Expose internal signals for Python test monitoring
    // =========================================================================

    // BP0 status
    wire bp0_busy = bp0_uo_out[0];
    wire bp0_valid = bp0_uo_out[1];
    wire bp0_report_valid = bp0_uo_out[2];
    wire [4:0] bp0_status = bp0_uo_out[7:3];

    // BP1 status
    wire bp1_busy = bp1_uo_out[0];
    wire bp1_valid = bp1_uo_out[1];
    wire bp1_report_valid = bp1_uo_out[2];
    wire [4:0] bp1_status = bp1_uo_out[7:3];

    // BP2 status
    wire bp2_busy = bp2_uo_out[0];
    wire bp2_valid = bp2_uo_out[1];
    wire bp2_report_valid = bp2_uo_out[2];
    wire [4:0] bp2_status = bp2_uo_out[7:3];

    // BP3 status (final output)
    wire bp3_busy = bp3_uo_out[0];
    wire bp3_valid = bp3_uo_out[1];
    wire bp3_report_valid = bp3_uo_out[2];
    wire [4:0] bp3_status = bp3_uo_out[7:3];

    // Pipeline busy (any BP is working OR inter-BP serialization in progress)
    // Must include transmitting/receiving states to avoid exiting early during
    // the 24-cycle inter-BP serialization window
    wire pipeline_busy = bp0_busy | bp1_busy | bp2_busy | bp3_busy |
                         bp0.inter_bp.param_transmitting | bp1.inter_bp.param_transmitting |
                         bp2.inter_bp.param_transmitting | bp3.inter_bp.param_transmitting |
                         bp1.inter_bp.param_receiving | bp2.inter_bp.param_receiving |
                         bp3.inter_bp.param_receiving;

    // Internal state exposure for debugging
    wire [3:0] bp0_state = bp0.bp_core.state;
    wire [3:0] bp1_state = bp1.bp_core.state;
    wire [3:0] bp2_state = bp2.bp_core.state;
    wire [3:0] bp3_state = bp3.bp_core.state;

    wire [GLOBAL_ADDR_BITS-1:0] bp0_addr = bp0.bp_core.addr_reg;
    wire [GLOBAL_ADDR_BITS-1:0] bp1_addr = bp1.bp_core.addr_reg;
    wire [GLOBAL_ADDR_BITS-1:0] bp2_addr = bp2.bp_core.addr_reg;
    wire [GLOBAL_ADDR_BITS-1:0] bp3_addr = bp3.bp_core.addr_reg;

    wire [7:0] bp0_threshold = bp0.bp_core.threshold_reg;
    wire [7:0] bp1_threshold = bp1.bp_core.threshold_reg;
    wire [7:0] bp2_threshold = bp2.bp_core.threshold_reg;
    wire [7:0] bp3_threshold = bp3.bp_core.threshold_reg;

    wire bp0_initialized = bp0.bp_core.initialized;
    wire bp1_initialized = bp1.bp_core.initialized;
    wire bp2_initialized = bp2.bp_core.initialized;
    wire bp3_initialized = bp3.bp_core.initialized;

    // Inter-BP communication monitoring
    wire bp0_inter_bp_param_valid = bp0.inter_bp_param_valid;
    wire bp1_inter_bp_param_valid = bp1.inter_bp_param_valid;
    wire bp2_inter_bp_param_valid = bp2.inter_bp_param_valid;
    wire bp3_inter_bp_param_valid = bp3.inter_bp_param_valid;

    // Debug: Monitor BP0's serializer output
    wire bp0_to_right_param_valid = bp0_uio_out[4];
    wire bp0_to_right_report_valid = bp0_uio_out[5];
    wire [1:0] bp0_to_right_data = bp0_uio_out[7:6];

    // Debug: Monitor BP1's received input
    wire bp1_from_left_param_valid = bp1_uio_in[4];
    wire bp1_from_left_report_valid = bp1_uio_in[5];
    wire [1:0] bp1_from_left_data = bp1_uio_in[7:6];

    // Debug: Monitor serializer trigger (bp_param_valid_out from bp_core)
    wire bp0_serialize_trigger = bp0.bp_core.bp_param_valid_out;
    wire bp1_serialize_trigger = bp1.bp_core.bp_param_valid_out;
    wire bp2_serialize_trigger = bp2.bp_core.bp_param_valid_out;

    // Debug: Monitor inter_bp transmitting state
    wire bp0_inter_bp_transmitting = bp0.inter_bp.param_transmitting;
    wire bp1_inter_bp_transmitting = bp1.inter_bp.param_transmitting;
    wire bp2_inter_bp_transmitting = bp2.inter_bp.param_transmitting;

    // Debug: Monitor inter_bp receiving state
    wire bp1_inter_bp_receiving = bp1.inter_bp.param_receiving;
    wire bp2_inter_bp_receiving = bp2.inter_bp.param_receiving;
    wire bp3_inter_bp_receiving = bp3.inter_bp.param_receiving;

    // Debug: Monitor forward_addr_reg in block_processor_core
    wire [GLOBAL_ADDR_BITS-1:0] bp0_forward_addr_reg = bp0.bp_core.forward_addr_reg;
    wire [GLOBAL_ADDR_BITS-1:0] bp1_forward_addr_reg = bp1.bp_core.forward_addr_reg;

    // Debug: Monitor what inter_bp captures into param_tx_shift_reg
    wire [47:0] bp0_tx_shift_reg = bp0.inter_bp.param_tx_shift_reg;
    wire [47:0] bp1_tx_shift_reg = bp1.inter_bp.param_tx_shift_reg;

    // Debug: Monitor params_module outputs
    wire [GLOBAL_ADDR_BITS-1:0] bp0_bp_addr_out = bp0.bp_addr_out;
    wire bp0_bp_param_valid_out = bp0.bp_param_valid_out;

    // Debug: Monitor BP0's stride and lambda for serialization
    wire [STRIDE_BITS-1:0] bp0_bp_stride_out = bp0.bp_stride_out;
    wire [7:0] bp0_bp_lambda_out = bp0.bp_lambda_out;
    wire [STRIDE_BITS-1:0] bp0_params_stride = bp0.bp_core.params_stride;
    wire [7:0] bp0_params_lambda = bp0.bp_core.params_lambda;

    // Debug: Monitor BP1's received params from inter_bp
    wire [GLOBAL_ADDR_BITS-1:0] bp1_inter_bp_addr = bp1.inter_bp.bp_addr_out;
    wire [STRIDE_BITS-1:0] bp1_inter_bp_stride = bp1.inter_bp.bp_stride_out;
    wire [7:0] bp1_inter_bp_lambda = bp1.inter_bp.bp_lambda_out;

    // Debug: Monitor BP1's rx shift register
    wire [47:0] bp1_rx_shift_reg = bp1.inter_bp.param_shift_reg;

    // Debug: Monitor BP1's params_module outputs (what was actually latched)
    wire [GLOBAL_ADDR_BITS-1:0] bp1_params_addr = bp1.bp_core.params_module.addr_out;
    wire [STRIDE_BITS-1:0] bp1_params_stride = bp1.bp_core.params_module.stride_out;
    wire [7:0] bp1_params_lambda = bp1.bp_core.params_module.lambda_out;
    wire [1:0] bp1_params_state = bp1.bp_core.params_module.state;
    wire bp1_params_ready = bp1.bp_core.params_module.params_ready;

    // Debug: Monitor BP1's bp_core signals related to parameter receiving
    wire bp1_start_from_neighbor = bp1.bp_core.params_start_from_neighbor;
    wire bp1_bp_param_valid_in = bp1.bp_core.bp_param_valid_in;

    // Debug: Monitor what inter_bp is providing (should match params_module input)
    wire [STRIDE_BITS-1:0] bp1_inter_bp_stride_debug = bp1.inter_bp.bp_stride_out;

endmodule

`default_nettype wire
