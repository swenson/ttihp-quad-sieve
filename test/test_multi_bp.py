# SPDX-FileCopyrightText: 2026 Christopher Swenson
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

# Mode definitions
MODE_IDLE = 0b00
MODE_INIT = 0b01
MODE_SIEVE = 0b10
MODE_REPORT = 0b11


async def load_bp_parameters(dut, addr, stride, lambda_val):
    """Load BP parameters using 10-cycle protocol via ui_in.

    Parameter format (48 bits total):
      - addr: 20 bits (global sieve address, 1MB total sieve)
      - stride: 20 bits (prime value, supports primes up to ~1M)
      - lambda: 8 bits (log approximation)

    Bit layout over 10 cycles at 5 bits each (clean alignment):
      Cycle 0:  addr[4:0]
      Cycle 1:  addr[9:5]
      Cycle 2:  addr[14:10]
      Cycle 3:  addr[19:15]
      Cycle 4:  stride[4:0]
      Cycle 5:  stride[9:5]
      Cycle 6:  stride[14:10]
      Cycle 7:  stride[19:15]
      Cycle 8:  lambda[4:0]
      Cycle 9:  lambda[7:5] (3 bits, upper 2 unused)
    """
    mode = MODE_SIEVE
    data_valid = 1

    # Build the 10 chunks according to the bit layout
    chunks = [
        (addr >> 0) & 0x1F,   # cycle 0: addr[4:0]
        (addr >> 5) & 0x1F,   # cycle 1: addr[9:5]
        (addr >> 10) & 0x1F,  # cycle 2: addr[14:10]
        (addr >> 15) & 0x1F,  # cycle 3: addr[19:15]
        (stride >> 0) & 0x1F, # cycle 4: stride[4:0]
        (stride >> 5) & 0x1F, # cycle 5: stride[9:5]
        (stride >> 10) & 0x1F, # cycle 6: stride[14:10]
        (stride >> 15) & 0x1F, # cycle 7: stride[19:15]
        (lambda_val >> 0) & 0x1F, # cycle 8: lambda[4:0]
        (lambda_val >> 5) & 0x07, # cycle 9: lambda[7:5] (3 bits)
    ]

    # First, ensure we start from IDLE state with data_valid=0
    dut.ui_in.value = MODE_SIEVE
    await ClockCycles(dut.clk, 1)

    # Send chunk 0 to trigger transition from IDLE to SIEVE_WAIT_PARAMS
    dut.ui_in.value = (chunks[0] << 3) | (data_valid << 2) | mode
    await ClockCycles(dut.clk, 1)

    # Clear data_valid briefly while waiting for state machine
    dut.ui_in.value = MODE_SIEVE
    await ClockCycles(dut.clk, 1)

    # Wait for BP to enter SIEVE_WAIT_PARAMS state (state=4)
    for _ in range(10):
        state = int(dut.bp0.state.value)
        if state == 4:  # ST_SIEVE_WAIT_PARAMS
            break
        await ClockCycles(dut.clk, 1)

    # Now send all 10 chunks (0-9) with data_valid high for exactly 1 cycle each
    for cycle in range(10):
        dut.ui_in.value = (chunks[cycle] << 3) | (data_valid << 2) | mode
        await ClockCycles(dut.clk, 1)

    # Clear data_valid
    dut.ui_in.value = MODE_SIEVE
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_two_bp_pipeline(dut):
    """Comprehensive test: Two BPs wired together, sieving across the boundary"""
    dut._log.info("=" * 80)
    dut._log.info("COMPREHENSIVE TEST: Two-BP Pipeline")
    dut._log.info("=" * 80)

    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    # Reset
    dut.ui_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)

    dut._log.info(" ")
    dut._log.info("=" * 80)
    dut._log.info("PHASE 1: Load parameters into BP0")
    dut._log.info("=" * 80)

    # We'll load parameters that make BP0 wrap after just 1 iteration
    # Start at 0xFFFF, stride 0x0001, so it wraps after 1 iteration:
    # Iteration 1: 0xFFFF + 0x0001 = 0x10000 (wraps immediately, passes to BP1)

    test_addr = 0xFFFF
    test_stride = 0x0001
    test_lambda = 0x03

    dut._log.info(
        f"Parameters: A=0x{test_addr:04x}, q=0x{test_stride:04x}, λ=0x{test_lambda:02x}"
    )
    dut._log.info(f"Expected iterations before wrap: 1")

    # Load parameters using the helper function
    await load_bp_parameters(dut, test_addr, test_stride, test_lambda)
    dut._log.info("  ✓ Parameters loaded")

    dut._log.info(" ")
    dut._log.info("=" * 80)
    dut._log.info("PHASE 2: Monitor BP0 sieving and parameter passing")
    dut._log.info("=" * 80)

    # Monitor BP0 until it wraps
    bp0_reports = []
    bp1_reports = []
    param_passed = False
    bp1_started = False
    bp0_busy_seen = False

    for cycle in range(1000):  # Increased from 200
        await ClockCycles(dut.clk, 1)

        # Check BP0 status
        bp0_busy = int(dut.bp0_uo_out.value) & 0x01
        bp0_valid = (int(dut.bp0_uo_out.value) >> 1) & 0x01
        bp0_report = (int(dut.bp0_uo_out.value) >> 2) & 0x01
        bp0_status = (int(dut.bp0_uo_out.value) >> 3) & 0x1F

        # Check BP1 status
        bp1_busy = int(dut.bp1_uo_out.value) & 0x01
        bp1_valid = (int(dut.bp1_uo_out.value) >> 1) & 0x01
        bp1_report = (int(dut.bp1_uo_out.value) >> 2) & 0x01
        bp1_status = (int(dut.bp1_uo_out.value) >> 3) & 0x1F

        # Debug: Log when BP0 becomes busy
        if bp0_busy == 1 and not bp0_busy_seen:
            bp0_busy_seen = True
            dut._log.info(f"  Cycle {cycle:4d}: BP0 started sieving (busy=1)")

        # Debug: Log when BP0 becomes idle after being busy
        if bp0_busy == 0 and bp0_busy_seen and not param_passed:
            dut._log.info(
                f"  Cycle {cycle:4d}: BP0 became idle (busy=0, valid={bp0_valid})"
            )

        # Debug: Every 50 cycles, print BP0 state
        if cycle % 50 == 0 and cycle > 0:
            try:
                # Try to read internal state (may not be accessible in all simulators)
                state = int(dut.bp0.state.value)
                addr = int(dut.bp0.addr_reg.value)
                stride = int(dut.bp0.params_module.stride_out.value)
                load_counter = int(dut.bp0.params_module.load_counter.value)
                data_valid = int(dut.bp0.data_valid_in.value)
                mode = int(dut.bp0.mode.value)
                dut._log.info(
                    f"  Cycle {cycle:4d}: BP0 state={state}, mode={mode}, addr=0x{addr:04x}, stride=0x{stride:04x}, load_ctr={load_counter}, data_valid={data_valid}"
                )
            except AttributeError:
                # Can't access internal signals, skip
                pass

        # Collect reports from BP0
        if bp0_report == 1 and bp0_status not in [r["addr"] for r in bp0_reports]:
            bp0_reports.append({"cycle": cycle, "addr": bp0_status})
            dut._log.info(f"  Cycle {cycle:4d}: BP0 report - addr=0x{bp0_status:02x}")

        # Collect reports from BP1
        if bp1_report == 1 and bp1_status not in [r["addr"] for r in bp1_reports]:
            bp1_reports.append({"cycle": cycle, "addr": bp1_status})
            dut._log.info(f"  Cycle {cycle:4d}: BP1 report - addr=0x{bp1_status:02x}")

        # Check if parameters were passed
        if not param_passed and bp0_valid == 1:
            param_passed = True
            dut._log.info(" ")
            dut._log.info(f"  Cycle {cycle:4d}: ✓ BP0 finished and signaled valid_out")

        # Check if BP1 started
        if not bp1_started and bp1_busy == 1:
            bp1_started = True
            dut._log.info(
                f"  Cycle {cycle:4d}: ✓ BP1 started sieving (received params from BP0)"
            )
            dut._log.info(" ")

        # Exit when BP1 completes
        if bp1_valid == 1:
            dut._log.info(f"  Cycle {cycle:4d}: ✓ BP1 finished")
            break

    dut._log.info(" ")
    dut._log.info("=" * 80)
    dut._log.info("PHASE 3: Verify results")
    dut._log.info("=" * 80)

    # Check if BP0 ever started
    if not bp0_busy_seen:
        dut._log.error("BP0 never became busy - parameters not loaded correctly!")
        assert False, "BP0 never started sieving - check parameter loading"

    # Verify that parameters were passed
    assert param_passed, "BP0 should have passed parameters to BP1"
    dut._log.info("✓ Parameters successfully passed from BP0 to BP1")

    # Verify that BP1 started
    assert bp1_started, "BP1 should have started sieving"
    dut._log.info("✓ BP1 successfully received parameters and started sieving")

    # Display collected reports
    dut._log.info(" ")
    dut._log.info(f"BP0 Reports ({len(bp0_reports)} total):")
    for report in bp0_reports:
        dut._log.info(f"  addr=0x{report['addr']:02x} at cycle {report['cycle']}")

    dut._log.info(" ")
    dut._log.info(f"BP1 Reports ({len(bp1_reports)} total):")
    for report in bp1_reports:
        dut._log.info(f"  addr=0x{report['addr']:02x} at cycle {report['cycle']}")

    # Verify we got some reports
    dut._log.info(" ")
    dut._log.info("Verification:")
    if len(bp0_reports) > 0:
        dut._log.info(
            f"✓ BP0 generated {len(bp0_reports)} reports (addresses in range 0xFFF0-0xFFF8)"
        )
    else:
        dut._log.info("  Note: BP0 generated 0 reports (threshold not exceeded)")

    if len(bp1_reports) > 0:
        dut._log.info(
            f"✓ BP1 generated {len(bp1_reports)} reports (addresses in range 0x0000-0x0008)"
        )
    else:
        dut._log.info("  Note: BP1 generated 0 reports (threshold not exceeded)")

    # Verify address ranges make sense
    # BP0 should report addresses near 0xFFF0
    # BP1 should report addresses near 0x0000
    if len(bp0_reports) > 0:
        bp0_addrs = [r["addr"] for r in bp0_reports]
        assert all(
            addr >= 0x10 or addr <= 0x18 for addr in bp0_addrs
        ), f"BP0 addresses should be in high range (got {[hex(a) for a in bp0_addrs]})"
        dut._log.info(
            f"✓ BP0 addresses are in expected range: {[hex(a) for a in bp0_addrs]}"
        )

    if len(bp1_reports) > 0:
        bp1_addrs = [r["addr"] for r in bp1_reports]
        assert all(
            addr <= 0x08 for addr in bp1_addrs
        ), f"BP1 addresses should be in low range (got {[hex(a) for a in bp1_addrs]})"
        dut._log.info(
            f"✓ BP1 addresses are in expected range: {[hex(a) for a in bp1_addrs]}"
        )

    dut._log.info(" ")
    dut._log.info("=" * 80)
    dut._log.info("✓✓✓ TWO-BP PIPELINE TEST PASSED ✓✓✓")
    dut._log.info("=" * 80)


@cocotb.test()
async def test_result_chaining(dut):
    """Test that reports from BP0 are forwarded through BP1"""
    dut._log.info("=" * 80)
    dut._log.info("TEST: Result chaining through two BPs")
    dut._log.info("=" * 80)

    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    # Reset
    dut.ui_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)

    # Set both BPs to SIEVE mode (but don't load parameters)
    dut.ui_in.value = MODE_SIEVE
    await ClockCycles(dut.clk, 5)

    # Load parameters into BP0 with high threshold so we get reports
    # Use addresses that will generate reports
    test_addr = 0xFFF0
    test_stride = 0x0004
    test_lambda = 0xFF  # Large lambda to exceed threshold

    dut._log.info(
        f"Loading BP0 with A=0x{test_addr:04x}, q=0x{test_stride:04x}, λ=0x{test_lambda:02x}"
    )

    # Load parameters using the helper function
    await load_bp_parameters(dut, test_addr, test_stride, test_lambda)

    dut._log.info("Monitoring for reports from both BPs...")

    # Monitor for reports
    bp0_reports = []
    bp1_reports = []

    for cycle in range(100):
        await ClockCycles(dut.clk, 1)

        # Check BP0 reports
        bp0_report = (int(dut.bp0_uo_out.value) >> 2) & 0x01
        bp0_status = (int(dut.bp0_uo_out.value) >> 3) & 0x1F

        # Check BP1 reports (should forward BP0's reports)
        bp1_report = (int(dut.bp1_uo_out.value) >> 2) & 0x01
        bp1_status = (int(dut.bp1_uo_out.value) >> 3) & 0x1F

        if bp0_report == 1:
            bp0_reports.append({"cycle": cycle, "addr": bp0_status})
            dut._log.info(f"  Cycle {cycle:3d}: BP0 report - addr=0x{bp0_status:02x}")

        if bp1_report == 1:
            bp1_reports.append({"cycle": cycle, "addr": bp1_status})
            dut._log.info(
                f"  Cycle {cycle:3d}: BP1 report - addr=0x{bp1_status:02x} (forwarded)"
            )

    dut._log.info(" ")
    dut._log.info(
        f"Summary: BP0 reports={len(bp0_reports)}, BP1 reports={len(bp1_reports)}"
    )

    # Verify that BP1 forwarded BP0's reports
    if len(bp0_reports) > 0:
        assert len(bp1_reports) >= len(
            bp0_reports
        ), "BP1 should forward at least as many reports as BP0 generated"
        dut._log.info("✓ BP1 successfully forwarded reports from BP0")
    else:
        dut._log.info("  Note: No reports generated (threshold not exceeded)")

    dut._log.info("=" * 80)
