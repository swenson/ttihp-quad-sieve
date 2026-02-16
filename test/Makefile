# Makefile
# See https://docs.cocotb.org/en/stable/quickstart.html for more info

# defaults
SIM ?= icarus
FST ?= -fst # Use more efficient FST format
TOPLEVEL_LANG ?= verilog
SRC_DIR = $(PWD)/../src
PROJECT_SOURCES = project.v block_processor_core.v tt_um_swenson_params.v tt_um_swenson_inter_bp.v spi.v

ifneq ($(GATES),yes)

# RTL simulation:
SIM_BUILD				= sim_build/rtl
VERILOG_SOURCES += $(addprefix $(SRC_DIR)/,$(PROJECT_SOURCES))

else

# Gate level simulation:
SIM_BUILD				= sim_build/gl
COMPILE_ARGS    += -DGL_TEST
COMPILE_ARGS    += -DFUNCTIONAL
COMPILE_ARGS    += -DSIM
VERILOG_SOURCES += $(PDK_ROOT)/ihp-sg13g2/libs.ref/sg13g2_io/verilog/sg13g2_io.v
VERILOG_SOURCES += $(PDK_ROOT)/ihp-sg13g2/libs.ref/sg13g2_stdcell/verilog/sg13g2_stdcell.v

# this gets copied in by the GDS action workflow
VERILOG_SOURCES += $(PWD)/gate_level_netlist.v

endif

# Allow sharing configuration between design and testbench via `include`:
COMPILE_ARGS 		+= -I$(SRC_DIR)

# Include the testbench sources:
VERILOG_SOURCES += $(PWD)/tb.v
TOPLEVEL = tb

# List test modules to run, separated by commas and without the .py suffix:
COCOTB_TEST_MODULES = test

# include cocotb's make rules to take care of the simulator setup
COCOTB_MAKEFILES_DIR := $(shell cocotb-config --makefiles 2>/dev/null)
ifneq ($(COCOTB_MAKEFILES_DIR),)
include $(COCOTB_MAKEFILES_DIR)/Makefile.sim
endif

# Phony targets for running all test suites
.PHONY: test-all test-all-clean test-unit lint

# Run Verilator lint on all Verilog sources
# Suppress warnings for:
#   UNUSEDSIGNAL/UNUSEDPARAM - Some signals kept for debugging or future use
#   SYNCASYNCNET - We intentionally use rst_n as async reset in some modules
#   DECLFILENAME - TT convention: project.v contains tt_um_* module
#   WIDTHTRUNC - Benign truncations where values fit (e.g., 15 into 4 bits)
lint:
	@echo "================================================================================"
	@echo "Running Verilator lint..."
	@echo "================================================================================"
	verilator --lint-only -Wall \
		-Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-SYNCASYNCNET \
		-Wno-DECLFILENAME -Wno-WIDTHTRUNC \
		-I$(SRC_DIR) $(addprefix $(SRC_DIR)/,$(PROJECT_SOURCES))
	@echo "Lint passed!"

# Run unit tests (pure Python, no simulation)
test-unit:
	@echo "================================================================================"
	@echo "Running unit tests..."
	@echo "================================================================================"
	python3 test_qs_algorithm_unit.py

# Run all test suites sequentially
test-all: test-unit
	@echo ""
	@echo "================================================================================"
	@echo "Running all simulation test suites..."
	@echo "================================================================================"
	@echo ""
	@echo "1/7: QS Small (N=10403 = 101 x 103)"
	@echo "--------------------------------------------------------------------------------"
	@$(MAKE) --no-print-directory -f Makefile.qs_small
	@echo ""
	@echo "2/7: QS Medium (N=604229 = 107 x 5647)"
	@echo "--------------------------------------------------------------------------------"
	@$(MAKE) --no-print-directory -f Makefile.qs_medium
	@echo ""
	@echo "3/7: QS Full (N=604227100699 = 677779 x 891481)"
	@echo "--------------------------------------------------------------------------------"
	@$(MAKE) --no-print-directory -f Makefile.qs_full
	@echo ""
	@echo "4/7: Multi-BP tests (two block processors in pipeline)"
	@echo "--------------------------------------------------------------------------------"
	@$(MAKE) --no-print-directory -f Makefile.multi_bp
	@echo ""
	@echo "5/7: Multiplexed pipeline tests (external wire protocol)"
	@echo "--------------------------------------------------------------------------------"
	@$(MAKE) --no-print-directory -f Makefile.mux_pipeline
	@echo ""
	@echo "6/7: QS Pipeline (two BPs with 8KB sieve)"
	@echo "--------------------------------------------------------------------------------"
	@$(MAKE) --no-print-directory -f Makefile.qs_pipeline
	@echo ""
	@echo "7/7: Inter-BP Communication (three BPs via uio)"
	@echo "--------------------------------------------------------------------------------"
	@$(MAKE) --no-print-directory -f Makefile.inter_bp
	@echo ""
	@echo "================================================================================"
	@echo "All test suites completed successfully!"
	@echo "================================================================================"

# Clean all test builds and run all tests
test-all-clean:
	@echo "Cleaning all test builds..."
	@$(MAKE) --no-print-directory -f Makefile.qs_small clean
	@$(MAKE) --no-print-directory -f Makefile.qs_medium clean
	@$(MAKE) --no-print-directory -f Makefile.qs_full clean
	@$(MAKE) --no-print-directory -f Makefile.multi_bp clean
	@$(MAKE) --no-print-directory -f Makefile.mux_pipeline clean
	@$(MAKE) --no-print-directory -f Makefile.qs_pipeline clean
	@$(MAKE) --no-print-directory -f Makefile.inter_bp clean
	@rm -rf sim_build
	@echo "Running all tests..."
	@$(MAKE) --no-print-directory test-all
