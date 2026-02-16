# SPDX-FileCopyrightText: 2026 Christopher Swenson
# SPDX-License-Identifier: Apache-2.0

"""
Test inter-BP communication via UIO pins.

This test exercises the 2-bit serial protocol used for ASIC-to-ASIC
communication through the uio[7:4] pins. Three complete tt_um_swenson_cqs
instances are wired together:
  - BP0 receives host commands via ui_in
  - BP0's uio_out[4:7] connects to BP1's uio_in[4:7]
  - BP1's uio_out[4:7] connects to BP2's uio_in[4:7]
  - BP2's uo_out is the final pipeline output

Each BP has 4KB of SPI RAM (LOCAL_ADDR_BITS=12), so the total sieve
array is 12KB across the three BPs.

Includes a full factorization test that factors N=10403=101×103 using
all three BPs in the pipeline.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, Timer
import sys
import os

# Import QS algorithm for factorization test
sys.path.insert(0, os.path.dirname(__file__))
from qs_algorithm import QuadraticSieve, Relation
from test_qs_algorithm_unit import find_null_space_gf2

# Mode definitions
MODE_IDLE = 0b00
MODE_INIT = 0b01
MODE_SIEVE = 0b10
MODE_REPORT = 0b11

# Local size per BP (1KB with LOCAL_ADDR_BITS=10)
# 4 BPs × 1KB = 4KB total sieve (matches original single-BP test)
LOCAL_SIZE = 1024
NUM_BPS = 4
TOTAL_SIEVE_SIZE = LOCAL_SIZE * NUM_BPS


def compute_python_sieve(params_list, sieve_size, threshold):
    """Compute expected sieve values and candidates using Python reference.

    Args:
        params_list: List of (addr, stride, lambda_val) tuples
        sieve_size: Total sieve array size
        threshold: Threshold for candidate detection

    Returns:
        Tuple of (sieve_array, candidates_set)
    """
    sieve = [0] * sieve_size
    for addr, stride, lambda_val in params_list:
        x = addr
        while x < sieve_size:
            sieve[x] += lambda_val
            x += stride
    candidates = set(i for i, val in enumerate(sieve) if val > threshold)
    return sieve, candidates


async def load_threshold(dut, threshold):
    """Load threshold value using 2-cycle protocol via ui_in."""
    mode = MODE_INIT
    data_valid = 1

    # Cycle 0: threshold[4:0]
    chunk_val = threshold & 0x1F
    dut.ui_in.value = (chunk_val << 3) | (data_valid << 2) | mode
    await ClockCycles(dut.clk, 1)

    # Cycle 1: threshold[7:5]
    chunk_val = (threshold >> 5) & 0x07
    dut.ui_in.value = (chunk_val << 3) | (data_valid << 2) | mode
    await ClockCycles(dut.clk, 1)

    # Clear data_valid
    dut.ui_in.value = mode
    await ClockCycles(dut.clk, 1)


async def load_bp_parameters(dut, addr, stride, lambda_val):
    """Load BP parameters using 10-cycle protocol via ui_in.

    Parameter format (48 bits total):
      - addr: 20 bits (global sieve address)
      - stride: 20 bits (prime value)
      - lambda: 8 bits (log approximation)

    Bit layout over 10 cycles at 5 bits each:
      Cycle 0:  addr[4:0]
      Cycle 1:  addr[9:5]
      Cycle 2:  addr[14:10]
      Cycle 3:  addr[19:15]
      Cycle 4:  stride[4:0]
      Cycle 5:  stride[9:5]
      Cycle 6:  stride[14:10]
      Cycle 7:  stride[19:15]
      Cycle 8:  lambda[4:0]
      Cycle 9:  lambda[7:5] (3 bits)
    """
    mode = MODE_SIEVE
    data_valid = 1

    chunks = [
        (addr >> 0) & 0x1F,
        (addr >> 5) & 0x1F,
        (addr >> 10) & 0x1F,
        (addr >> 15) & 0x1F,
        (stride >> 0) & 0x1F,
        (stride >> 5) & 0x1F,
        (stride >> 10) & 0x1F,
        (stride >> 15) & 0x1F,
        (lambda_val >> 0) & 0x1F,
        (lambda_val >> 5) & 0x07,
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
        state = int(dut.bp0_state.value)
        if state == 4:  # ST_SIEVE_WAIT_PARAMS
            break
        await ClockCycles(dut.clk, 1)

    # Now send all 10 chunks with data_valid high
    for cycle in range(10):
        dut.ui_in.value = (chunks[cycle] << 3) | (data_valid << 2) | mode
        await ClockCycles(dut.clk, 1)

    # Clear data_valid
    dut.ui_in.value = MODE_SIEVE
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_init_all_bps(dut):
    """Test that MODE_INIT initializes all three BPs."""
    dut._log.info("=" * 80)
    dut._log.info("TEST: Initialize all three BPs")
    dut._log.info("=" * 80)

    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    # Reset
    dut.ui_in.value = 0
    dut.ena.value = 1
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)

    # Set MODE_INIT
    dut.ui_in.value = MODE_INIT
    await ClockCycles(dut.clk, 2)

    # Load threshold (same for all BPs since they share ui_in mode)
    threshold = 20
    dut._log.info(f"Loading threshold = {threshold}")
    await load_threshold(dut, threshold)

    # Wait for BP0 to complete initialization
    dut._log.info("Waiting for BP0 initialization...")
    for i in range(200000):
        await ClockCycles(dut.clk, 100)
        if int(dut.bp0_busy.value) == 0:
            dut._log.info(f"  BP0 initialization complete after {(i+1)*100} cycles")
            break
    else:
        assert False, "BP0 initialization timed out"

    # Verify threshold was loaded
    bp0_thresh = int(dut.bp0_threshold.value)
    dut._log.info(f"  BP0 threshold = {bp0_thresh}")
    assert bp0_thresh == threshold, f"BP0 threshold mismatch: {bp0_thresh} != {threshold}"

    dut._log.info("=" * 80)
    dut._log.info("INIT TEST PASSED")
    dut._log.info("=" * 80)


@cocotb.test()
async def test_sieve_single_bp(dut):
    """Test sieving that stays within BP0's range."""
    dut._log.info("=" * 80)
    dut._log.info("TEST: Sieve within single BP (BP0 only)")
    dut._log.info("=" * 80)

    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    # Reset
    dut.ui_in.value = 0
    dut.ena.value = 1
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)

    # Test parameters
    threshold = 5  # Low threshold to get candidates
    addr = 100
    stride = 50
    lambda_val = 10

    # Compute expected candidates using Python reference
    _, expected_candidates = compute_python_sieve(
        [(addr, stride, lambda_val)], TOTAL_SIEVE_SIZE, threshold
    )
    dut._log.info(f"Python expects {len(expected_candidates)} candidates above threshold {threshold}")

    # Initialize - wait for ALL BPs to complete (clears RAM to zeros)
    dut.ui_in.value = MODE_INIT
    await ClockCycles(dut.clk, 2)
    await load_threshold(dut, threshold)

    # Wait for ALL BPs to complete initialization
    for _ in range(200000):
        await ClockCycles(dut.clk, 100)
        if (int(dut.bp0_busy.value) == 0 and int(dut.bp1_busy.value) == 0 and
            int(dut.bp2_busy.value) == 0 and int(dut.bp3_busy.value) == 0):
            break

    # Switch to SIEVE mode
    dut.ui_in.value = MODE_SIEVE
    await ClockCycles(dut.clk, 5)

    dut._log.info(f"Loading params: addr={addr}, stride={stride}, lambda={lambda_val}")
    await load_bp_parameters(dut, addr, stride, lambda_val)

    # Collect reports using the standard pipeline function
    hw_candidates = await run_sieving_pipeline(dut, max_cycles=500000)

    dut._log.info(f"Hardware found {len(hw_candidates)} candidates")
    dut._log.info(f"Python expected {len(expected_candidates)} candidates")

    # Verify exact match
    assert hw_candidates == expected_candidates, (
        f"Candidate mismatch!\n"
        f"  Hardware: {sorted(hw_candidates)}\n"
        f"  Python:   {sorted(expected_candidates)}\n"
        f"  Missing:  {sorted(expected_candidates - hw_candidates)}\n"
        f"  Extra:    {sorted(hw_candidates - expected_candidates)}"
    )

    dut._log.info("=" * 80)
    dut._log.info("SINGLE BP SIEVE TEST PASSED")
    dut._log.info("=" * 80)


@cocotb.test()
async def test_sieve_crosses_bp_boundary(dut):
    """Test sieving that crosses from BP0 to BP1 via uio communication."""
    dut._log.info("=" * 80)
    dut._log.info("TEST: Sieve crossing BP boundary (BP0 -> BP1 via uio)")
    dut._log.info("=" * 80)

    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    # Reset
    dut.ui_in.value = 0
    dut.ena.value = 1
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)

    # Test parameters
    threshold = 10
    addr = 900
    stride = 100
    lambda_val = 15

    # Compute expected candidates using Python reference
    _, expected_candidates = compute_python_sieve(
        [(addr, stride, lambda_val)], TOTAL_SIEVE_SIZE, threshold
    )
    dut._log.info(f"Python expects {len(expected_candidates)} candidates above threshold {threshold}")

    # Initialize all BPs
    dut._log.info("1. Initializing all BPs...")
    dut.ui_in.value = MODE_INIT
    await ClockCycles(dut.clk, 2)
    await load_threshold(dut, threshold)

    # Wait for ALL BPs to complete initialization (clears RAM to zeros)
    for _ in range(200000):
        await ClockCycles(dut.clk, 100)
        if (int(dut.bp0_busy.value) == 0 and int(dut.bp1_busy.value) == 0 and
            int(dut.bp2_busy.value) == 0 and int(dut.bp3_busy.value) == 0):
            break

    dut._log.info("   All BPs initialized")

    # Switch to SIEVE mode
    dut.ui_in.value = MODE_SIEVE
    await ClockCycles(dut.clk, 5)

    dut._log.info(f"2. Loading params: addr={addr}, stride={stride}, lambda={lambda_val}")
    dut._log.info(f"   BP0 range: 0-{LOCAL_SIZE-1}, BP1 range: {LOCAL_SIZE}-{2*LOCAL_SIZE-1}")
    await load_bp_parameters(dut, addr, stride, lambda_val)

    # Collect reports using the standard pipeline function
    hw_candidates = await run_sieving_pipeline(dut, max_cycles=100000)

    dut._log.info(f"Hardware found {len(hw_candidates)} candidates")
    dut._log.info(f"Python expected {len(expected_candidates)} candidates")

    # Verify exact match
    assert hw_candidates == expected_candidates, (
        f"Candidate mismatch!\n"
        f"  Hardware: {sorted(hw_candidates)}\n"
        f"  Python:   {sorted(expected_candidates)}\n"
        f"  Missing:  {sorted(expected_candidates - hw_candidates)}\n"
        f"  Extra:    {sorted(hw_candidates - expected_candidates)}"
    )

    dut._log.info("=" * 80)
    dut._log.info("BP BOUNDARY CROSSING TEST PASSED")
    dut._log.info("=" * 80)


@cocotb.test()
async def test_sieve_all_four_bps(dut):
    """Test sieving that crosses all four BPs."""
    dut._log.info("=" * 80)
    dut._log.info("TEST: Sieve across all four BPs (BP0 -> BP1 -> BP2 -> BP3)")
    dut._log.info("=" * 80)

    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    # Reset
    dut.ui_in.value = 0
    dut.ena.value = 1
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)

    # Test parameters
    threshold = 15
    addr = 100
    stride = 1000
    lambda_val = 20

    # Compute expected candidates using Python reference
    _, expected_candidates = compute_python_sieve(
        [(addr, stride, lambda_val)], TOTAL_SIEVE_SIZE, threshold
    )
    dut._log.info(f"Python expects {len(expected_candidates)} candidates above threshold {threshold}")
    dut._log.info(f"Expected addresses: {sorted(expected_candidates)}")

    # Initialize
    dut._log.info("1. Initializing all BPs...")
    dut.ui_in.value = MODE_INIT
    await ClockCycles(dut.clk, 2)
    await load_threshold(dut, threshold)

    # Wait for ALL BPs to complete initialization (clears RAM to zeros)
    for _ in range(200000):
        await ClockCycles(dut.clk, 100)
        if (int(dut.bp0_busy.value) == 0 and int(dut.bp1_busy.value) == 0 and
            int(dut.bp2_busy.value) == 0 and int(dut.bp3_busy.value) == 0):
            break

    # Switch to SIEVE mode
    dut.ui_in.value = MODE_SIEVE
    await ClockCycles(dut.clk, 5)

    dut._log.info(f"2. Loading params: addr={addr}, stride={stride}, lambda={lambda_val}")
    dut._log.info(f"   Expected hits: {addr} (BP0), {addr+stride} (BP1), {addr+2*stride} (BP2), {addr+3*stride} (BP3)")
    await load_bp_parameters(dut, addr, stride, lambda_val)

    # Collect reports using the standard pipeline function
    hw_candidates = await run_sieving_pipeline(dut, max_cycles=200000)

    dut._log.info(f"Hardware found {len(hw_candidates)} candidates: {sorted(hw_candidates)}")
    dut._log.info(f"Python expected {len(expected_candidates)} candidates")

    # Verify exact match
    assert hw_candidates == expected_candidates, (
        f"Candidate mismatch!\n"
        f"  Hardware: {sorted(hw_candidates)}\n"
        f"  Python:   {sorted(expected_candidates)}\n"
        f"  Missing:  {sorted(expected_candidates - hw_candidates)}\n"
        f"  Extra:    {sorted(hw_candidates - expected_candidates)}"
    )

    dut._log.info("=" * 80)
    dut._log.info("FOUR BP PIPELINE TEST PASSED")
    dut._log.info("=" * 80)


@cocotb.test()
async def test_report_forwarding(dut):
    """Test that reports are forwarded through the BP chain."""
    dut._log.info("=" * 80)
    dut._log.info("TEST: Report forwarding through BP chain")
    dut._log.info("=" * 80)

    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    # Reset
    dut.ui_in.value = 0
    dut.ena.value = 1
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)

    # Test parameters
    threshold = 5  # Low threshold = more reports
    addr = 100
    stride = 200
    lambda_val = 10

    # Compute expected candidates using Python reference
    _, expected_candidates = compute_python_sieve(
        [(addr, stride, lambda_val)], TOTAL_SIEVE_SIZE, threshold
    )
    dut._log.info(f"Python expects {len(expected_candidates)} candidates above threshold {threshold}")

    # Initialize with low threshold to generate reports
    dut._log.info("1. Initializing with low threshold...")
    dut.ui_in.value = MODE_INIT
    await ClockCycles(dut.clk, 2)
    await load_threshold(dut, threshold)

    # Wait for ALL BPs to complete initialization (clears RAM to zeros)
    for _ in range(200000):
        await ClockCycles(dut.clk, 100)
        if (int(dut.bp0_busy.value) == 0 and int(dut.bp1_busy.value) == 0 and
            int(dut.bp2_busy.value) == 0 and int(dut.bp3_busy.value) == 0):
            break

    # Switch to SIEVE mode
    dut.ui_in.value = MODE_SIEVE
    await ClockCycles(dut.clk, 5)

    dut._log.info(f"2. Loading params: addr={addr}, stride={stride}, lambda={lambda_val}")
    await load_bp_parameters(dut, addr, stride, lambda_val)

    # Collect reports using the standard pipeline function
    hw_candidates = await run_sieving_pipeline(dut, max_cycles=100000)

    dut._log.info(f"Hardware found {len(hw_candidates)} candidates")
    dut._log.info(f"Python expected {len(expected_candidates)} candidates")

    # Verify exact match
    assert hw_candidates == expected_candidates, (
        f"Candidate mismatch!\n"
        f"  Hardware: {sorted(hw_candidates)}\n"
        f"  Python:   {sorted(expected_candidates)}\n"
        f"  Missing:  {sorted(expected_candidates - hw_candidates)}\n"
        f"  Extra:    {sorted(hw_candidates - expected_candidates)}"
    )

    dut._log.info("=" * 80)
    dut._log.info("REPORT FORWARDING TEST PASSED")
    dut._log.info("=" * 80)


@cocotb.test()
async def test_skip_to_bp3(dut):
    """Test that parameters can skip directly to BP3 if address is out of range."""
    dut._log.info("=" * 80)
    dut._log.info("TEST: Skip directly to BP3 (address in BP3's range)")
    dut._log.info("=" * 80)

    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    # Reset
    dut.ui_in.value = 0
    dut.ena.value = 1
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)

    # Test parameters - address starts in BP3's range
    threshold = 5  # Low threshold to get candidates
    addr = 3200  # In BP3's range (3072-4095)
    stride = 100
    lambda_val = 10

    # Compute expected candidates using Python reference
    _, expected_candidates = compute_python_sieve(
        [(addr, stride, lambda_val)], TOTAL_SIEVE_SIZE, threshold
    )
    dut._log.info(f"Python expects {len(expected_candidates)} candidates above threshold {threshold}")
    dut._log.info(f"Expected addresses: {sorted(expected_candidates)}")

    # Initialize
    dut.ui_in.value = MODE_INIT
    await ClockCycles(dut.clk, 2)
    await load_threshold(dut, threshold)

    # Wait for ALL BPs to complete initialization (clears RAM to zeros)
    for _ in range(200000):
        await ClockCycles(dut.clk, 100)
        if (int(dut.bp0_busy.value) == 0 and int(dut.bp1_busy.value) == 0 and
            int(dut.bp2_busy.value) == 0 and int(dut.bp3_busy.value) == 0):
            break

    # Switch to SIEVE mode
    dut.ui_in.value = MODE_SIEVE
    await ClockCycles(dut.clk, 5)

    dut._log.info(f"Loading params: addr={addr}, stride={stride}, lambda={lambda_val}")
    dut._log.info(f"Address {addr} should skip BP0, BP1, and BP2, go directly to BP3")
    await load_bp_parameters(dut, addr, stride, lambda_val)

    # Collect reports using the standard pipeline function
    hw_candidates = await run_sieving_pipeline(dut, max_cycles=200000)

    dut._log.info(f"Hardware found {len(hw_candidates)} candidates: {sorted(hw_candidates)}")
    dut._log.info(f"Python expected {len(expected_candidates)} candidates")

    # Verify exact match
    assert hw_candidates == expected_candidates, (
        f"Candidate mismatch!\n"
        f"  Hardware: {sorted(hw_candidates)}\n"
        f"  Python:   {sorted(expected_candidates)}\n"
        f"  Missing:  {sorted(expected_candidates - hw_candidates)}\n"
        f"  Extra:    {sorted(hw_candidates - expected_candidates)}"
    )

    dut._log.info("=" * 80)
    dut._log.info("SKIP TO BP3 TEST PASSED")
    dut._log.info("=" * 80)


# =============================================================================
# Full Factorization Test using 4-BP Pipeline via UIO Communication
# =============================================================================

# Factorization test parameters
N = 10403  # = 101 × 103
FACTOR_BASE_BOUND = 100
SIEVE_SIZE = 4096  # 4KB total across 4 BPs (1KB each)
THRESHOLD = 14


async def run_sieving_pipeline(dut, max_cycles=500000):
    """Run sieving across the 4-BP pipeline and collect report addresses.

    Collects reports from each BP individually, distinguishing between local
    reports (generated by that BP) and forwarded reports (from left neighbor).

    Local reports have addresses in [0, LOCAL_SIZE) because the BP uses relative
    addressing. Forwarded reports have addresses >= LOCAL_SIZE because each hop
    adds LOCAL_SIZE.

    Returns:
        Set of unique global report addresses
    """
    reports = set()
    idle_cycles = 0
    required_idle_cycles = 50  # Wait for parameters to finish propagating

    for _ in range(max_cycles):
        await ClockCycles(dut.clk, 1)

        # BP0's reports: always local (no left neighbor, bp_report_valid_in = 0)
        # BP0's relative addresses ARE global (BP0 starts at global 0)
        if int(dut.bp0_report_valid.value) == 1:
            report_addr = int(dut.bp0.bp_core.bp_report_addr_out.value)
            reports.add(report_addr)

        # BP1's reports: local if addr < LOCAL_SIZE, forwarded otherwise
        if int(dut.bp1_report_valid.value) == 1:
            report_addr = int(dut.bp1.bp_core.bp_report_addr_out.value)
            if report_addr < LOCAL_SIZE:
                # Local report: convert to global by adding BP1's base offset
                global_addr = report_addr + LOCAL_SIZE
                reports.add(global_addr)
            # else: forwarded report, already counted at BP0

        # BP2's reports: local if addr < LOCAL_SIZE
        if int(dut.bp2_report_valid.value) == 1:
            report_addr = int(dut.bp2.bp_core.bp_report_addr_out.value)
            if report_addr < LOCAL_SIZE:
                global_addr = report_addr + 2 * LOCAL_SIZE
                reports.add(global_addr)

        # BP3's reports: local if addr < LOCAL_SIZE
        if int(dut.bp3_report_valid.value) == 1:
            report_addr = int(dut.bp3.bp_core.bp_report_addr_out.value)
            if report_addr < LOCAL_SIZE:
                global_addr = report_addr + 3 * LOCAL_SIZE
                reports.add(global_addr)

        # Check if any inter_bp is transmitting or receiving parameters
        params_in_flight = (
            int(dut.bp0_inter_bp_transmitting.value) == 1 or
            int(dut.bp1_inter_bp_transmitting.value) == 1 or
            int(dut.bp2_inter_bp_transmitting.value) == 1 or
            int(dut.bp1_inter_bp_receiving.value) == 1 or
            int(dut.bp2_inter_bp_receiving.value) == 1 or
            int(dut.bp3_inter_bp_receiving.value) == 1
        )

        # Check if pipeline is done (all BPs idle AND no params in flight)
        if int(dut.pipeline_busy.value) == 0 and not params_in_flight:
            idle_cycles += 1
            if idle_cycles >= required_idle_cycles:
                break
        else:
            idle_cycles = 0

    return reports


@cocotb.test()
async def test_param_latch_timing(dut):
    """Debug test: trace exact timing of param latching from inter_bp to params_module."""
    dut._log.info("=" * 80)
    dut._log.info("DEBUG: Tracing exact param latch timing")
    dut._log.info("=" * 80)

    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    # Reset
    dut.ui_in.value = 0
    dut.ena.value = 1
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)

    # Initialize
    dut.ui_in.value = MODE_INIT
    await ClockCycles(dut.clk, 2)
    await load_threshold(dut, 14)

    # Wait for ALL BPs to complete initialization (clears RAM to zeros)
    for _ in range(200000):
        await ClockCycles(dut.clk, 100)
        if (int(dut.bp0_busy.value) == 0 and int(dut.bp1_busy.value) == 0 and
            int(dut.bp2_busy.value) == 0 and int(dut.bp3_busy.value) == 0):
            break

    # Switch to SIEVE mode
    dut.ui_in.value = MODE_SIEVE
    await ClockCycles(dut.clk, 5)

    # Load a simple param set that will cross to BP1
    # addr=0, stride=5, lambda=10 - will sieve 0,5,10,...,1020 in BP0 then forward
    addr = 0
    stride = 5
    lambda_val = 10
    dut._log.info(f"Loading params: addr={addr}, stride={stride}, lambda={lambda_val}")
    await load_bp_parameters(dut, addr, stride, lambda_val)

    # Watch cycle-by-cycle from the start - look for BP0 transmitting and BP1 receiving
    dut._log.info("Watching for BP0 transmitting and BP1 receiving params...")
    bp0_transmitting_seen = False
    bp1_receiving_seen = False
    bp1_param_valid_seen = False
    bp1_start_from_neighbor_seen = False
    bp1_params_ready_seen = False
    trace_cycle = 0

    for cycle in range(200000):
        await ClockCycles(dut.clk, 1)

        # Read BP0 serializer state
        bp0_transmitting = int(dut.bp0_inter_bp_transmitting.value)

        # Read BP1 inter_bp state
        bp1_receiving = int(dut.bp1_inter_bp_receiving.value)
        bp1_inter_bp_param_valid = int(dut.bp1.inter_bp.bp_param_valid_out.value)
        bp1_inter_bp_stride = int(dut.bp1.inter_bp.bp_stride_out.value)
        bp1_inter_bp_lambda = int(dut.bp1.inter_bp.bp_lambda_out.value)

        bp1_bp_core_state = int(dut.bp1_state.value)
        bp1_start_from_neighbor = int(dut.bp1_start_from_neighbor.value)
        bp1_bp_param_valid_in = int(dut.bp1_bp_param_valid_in.value)

        bp1_params_state = int(dut.bp1_params_state.value)
        bp1_params_stride = int(dut.bp1_params_stride.value)
        bp1_params_lambda = int(dut.bp1_params_lambda.value)
        bp1_params_ready = int(dut.bp1_params_ready.value)

        # Log when BP0 starts transmitting
        if bp0_transmitting == 1 and not bp0_transmitting_seen:
            dut._log.info(f"  Cycle {cycle}: BP0 inter_bp STARTED transmitting")
            bp0_transmitting_seen = True
            trace_cycle = cycle

        # Log when BP1 starts receiving
        if bp1_receiving == 1 and not bp1_receiving_seen:
            dut._log.info(f"  Cycle {cycle}: BP1 inter_bp STARTED receiving")
            bp1_receiving_seen = True

        # Log when inter_bp param_valid goes high
        if bp1_inter_bp_param_valid == 1 and not bp1_param_valid_seen:
            dut._log.info(f"  Cycle {cycle}: inter_bp.bp_param_valid_out=1 FIRST SEEN")
            dut._log.info(f"    inter_bp outputs: stride={bp1_inter_bp_stride}, lambda={bp1_inter_bp_lambda}")
            dut._log.info(f"    bp_core state={bp1_bp_core_state}, params_state={bp1_params_state}")
            bp1_param_valid_seen = True

        # Log when start_from_neighbor goes high
        if bp1_start_from_neighbor == 1 and not bp1_start_from_neighbor_seen:
            dut._log.info(f"  Cycle {cycle}: start_from_neighbor=1 FIRST SEEN")
            dut._log.info(f"    inter_bp outputs: stride={bp1_inter_bp_stride}, lambda={bp1_inter_bp_lambda}")
            dut._log.info(f"    bp_core state={bp1_bp_core_state}, params_state={bp1_params_state}")
            bp1_start_from_neighbor_seen = True

        # Log when params_ready pulses
        if bp1_params_ready == 1 and not bp1_params_ready_seen:
            dut._log.info(f"  Cycle {cycle}: params_ready=1 (params_module latched values)")
            dut._log.info(f"    params_module latched: stride={bp1_params_stride}, lambda={bp1_params_lambda}")
            dut._log.info(f"    inter_bp outputs at this time: stride={bp1_inter_bp_stride}, lambda={bp1_inter_bp_lambda}")
            match_stride = bp1_params_stride == bp1_inter_bp_stride
            match_lambda = bp1_params_lambda == bp1_inter_bp_lambda
            dut._log.info(f"    MATCH: stride={match_stride}, lambda={match_lambda}")
            bp1_params_ready_seen = True

        # After seeing params_ready, check if BP1 starts sieving and exit
        if bp1_params_ready_seen and bp1_bp_core_state == 6:  # ST_SIEVE_READ
            dut._log.info(f"  Cycle {cycle}: BP1 started sieving (state=6)")
            break

        # Timeout after seeing everything we need
        if bp1_params_ready_seen and cycle > trace_cycle + 50:
            break

        # After BP0 starts transmitting, continue for at least 100 cycles to see full reception
        if bp0_transmitting_seen and cycle > trace_cycle + 100:
            dut._log.info(f"  Cycle {cycle}: Timeout - 100 cycles after transmission started")
            dut._log.info(f"    bp1_inter_bp_param_valid={bp1_inter_bp_param_valid}")
            dut._log.info(f"    bp1_receiving={bp1_receiving}")
            dut._log.info(f"    pipeline_busy={int(dut.pipeline_busy.value)}")
            break

        # Only use pipeline idle as fallback before transmission starts
        if not bp0_transmitting_seen and cycle > 1000 and int(dut.pipeline_busy.value) == 0:
            dut._log.info(f"  Cycle {cycle}: Pipeline idle before transmission - something wrong")
            break

    # Final check
    bp1_params_stride_final = int(dut.bp1_params_stride.value)
    bp1_params_lambda_final = int(dut.bp1_params_lambda.value)

    # Expected BP1 params: stride should be 5 (same as input), lambda=10
    # Address should be decremented by LOCAL_SIZE
    dut._log.info(f"\nFinal params_module values: stride={bp1_params_stride_final}, lambda={bp1_params_lambda_final}")
    dut._log.info(f"Expected: stride={stride}, lambda={lambda_val}")

    if bp1_params_stride_final != stride or bp1_params_lambda_final != lambda_val:
        dut._log.error(f"MISMATCH! params_module has wrong values")
        dut._log.error(f"  Expected stride={stride}, got {bp1_params_stride_final}")
        dut._log.error(f"  Expected lambda={lambda_val}, got {bp1_params_lambda_final}")
    else:
        dut._log.info("SUCCESS! params_module has correct values")

    dut._log.info("=" * 80)


@cocotb.test()
async def test_debug_single_prime(dut):
    """Debug test: trace exactly what happens with a single prime across BPs."""
    dut._log.info("=" * 80)
    dut._log.info("DEBUG: Tracing single prime p=2 across 4 BPs")
    dut._log.info("=" * 80)

    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    # Reset
    dut.ui_in.value = 0
    dut.ena.value = 1
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)

    # Initialize with threshold=14
    dut.ui_in.value = MODE_INIT
    await ClockCycles(dut.clk, 2)
    await load_threshold(dut, 14)

    for _ in range(200000):
        await ClockCycles(dut.clk, 100)
        # Wait for ALL BPs to finish initialization
        if (int(dut.bp0_busy.value) == 0 and
            int(dut.bp1_busy.value) == 0 and
            int(dut.bp2_busy.value) == 0 and
            int(dut.bp3_busy.value) == 0):
            break

    dut._log.info(f"All BPs initialized. BP0={int(dut.bp0_busy.value)}, BP1={int(dut.bp1_busy.value)}, BP2={int(dut.bp2_busy.value)}, BP3={int(dut.bp3_busy.value)}")

    # Check BP1's RAM at address 10 (corresponds to global 1034)
    # BP1's RAM[10] should be 0 after init
    bp1_ram_10_before = int(dut.bp1_ram.data[10 >> 2].value)
    bp1_ram_10_byte = (bp1_ram_10_before >> (8 * (10 & 0x3))) & 0xFF
    dut._log.info(f"BP1 RAM[10] before sieving: {bp1_ram_10_byte}")

    # Switch to SIEVE mode
    dut.ui_in.value = MODE_SIEVE
    await ClockCycles(dut.clk, 5)

    # Load params for p=2: addr=0, stride=2, lambda=1
    addr = 0
    stride = 2
    lambda_val = 1
    dut._log.info(f"Loading params: addr={addr}, stride={stride}, lambda={lambda_val}")
    await load_bp_parameters(dut, addr, stride, lambda_val)

    # Collect reports and trace BP activity
    reports = []
    bp1_sieve_addrs = []
    bp0_forwarded = False
    bp1_received = False
    bp1_started = False
    # Reset function attributes used for one-time debug output
    for attr in ['bp0_captured', 'bp1_done_receiving', 'bp1_wait_params', 'bp1_check_range', 'trace_start']:
        if hasattr(test_debug_single_prime, attr):
            delattr(test_debug_single_prime, attr)

    for cycle in range(100000):
        await ClockCycles(dut.clk, 1)

        # Track BP0 forwarding
        bp0_state = int(dut.bp0_state.value)
        if bp0_state == 10 and not bp0_forwarded:  # ST_SIEVE_FORWARD
            bp0_addr = int(dut.bp0_addr.value)
            bp0_forward_addr_reg = int(dut.bp0_forward_addr_reg.value)
            bp0_bp_addr_out = int(dut.bp0_bp_addr_out.value)
            bp0_tx_shift_reg = int(dut.bp0_tx_shift_reg.value)
            dut._log.info(f"  Cycle {cycle}: BP0 in FORWARD state")
            dut._log.info(f"    global_addr_reg={bp0_addr}, forward_addr_reg={bp0_forward_addr_reg}")
            dut._log.info(f"    bp_addr_out={bp0_bp_addr_out}, tx_shift_reg[19:0]={bp0_tx_shift_reg & 0xFFFFF}")
            bp0_forwarded = True

        # Track when BP0 inter_bp captures value (transmitting goes high)
        bp0_transmitting = int(dut.bp0_inter_bp_transmitting.value)
        if bp0_transmitting == 1 and not hasattr(test_debug_single_prime, 'bp0_captured'):
            test_debug_single_prime.bp0_captured = True
            bp0_tx_shift_reg = int(dut.bp0_tx_shift_reg.value)
            bp0_forward_addr_reg = int(dut.bp0_forward_addr_reg.value)
            bp0_bp_addr_out = int(dut.bp0_bp_addr_out.value)
            bp0_bp_stride_out = int(dut.bp0_bp_stride_out.value)
            bp0_bp_lambda_out = int(dut.bp0_bp_lambda_out.value)
            bp0_params_stride = int(dut.bp0_params_stride.value)
            bp0_params_lambda = int(dut.bp0_params_lambda.value)
            # Extract values from tx_shift_reg: {lambda[7:0], stride[19:0], addr[19:0]}
            tx_addr = bp0_tx_shift_reg & 0xFFFFF
            tx_stride = (bp0_tx_shift_reg >> 20) & 0xFFFFF
            tx_lambda = (bp0_tx_shift_reg >> 40) & 0xFF
            dut._log.info(f"  Cycle {cycle}: BP0 inter_bp STARTED transmitting")
            dut._log.info(f"    tx_shift_reg: addr={tx_addr}, stride={tx_stride}, lambda={tx_lambda}")
            dut._log.info(f"    bp_*_out: addr={bp0_bp_addr_out}, stride={bp0_bp_stride_out}, lambda={bp0_bp_lambda_out}")
            dut._log.info(f"    params_*: stride={bp0_params_stride}, lambda={bp0_params_lambda}")
            dut._log.info(f"    forward_addr_reg={bp0_forward_addr_reg}")

        # Track BP1 receiving
        if int(dut.bp1_inter_bp_receiving.value) == 1 and not bp1_received:
            dut._log.info(f"  Cycle {cycle}: BP1 inter_bp receiving")
            bp1_received = True

        # Track when BP1 stops receiving (deserialization complete)
        if bp1_received and int(dut.bp1_inter_bp_receiving.value) == 0 and not hasattr(test_debug_single_prime, 'bp1_done_receiving'):
            test_debug_single_prime.bp1_done_receiving = True
            bp1_inter_bp_addr = int(dut.bp1.inter_bp.bp_addr_out.value)
            bp1_inter_bp_stride = int(dut.bp1.inter_bp.bp_stride_out.value)
            bp1_inter_bp_lambda = int(dut.bp1.inter_bp.bp_lambda_out.value)
            bp1_rx_shift_reg = int(dut.bp1_rx_shift_reg.value)
            # Extract values from rx_shift_reg
            rx_addr = bp1_rx_shift_reg & 0xFFFFF
            rx_stride = (bp1_rx_shift_reg >> 20) & 0xFFFFF
            rx_lambda = (bp1_rx_shift_reg >> 40) & 0xFF
            dut._log.info(f"  Cycle {cycle}: BP1 inter_bp DONE receiving")
            dut._log.info(f"    rx_shift_reg: addr={rx_addr}, stride={rx_stride}, lambda={rx_lambda}")
            dut._log.info(f"    bp_*_out (before extraction): addr={bp1_inter_bp_addr}, stride={bp1_inter_bp_stride}, lambda={bp1_inter_bp_lambda}")
            # Log the next 15 cycles to trace state machine
            test_debug_single_prime.trace_start = cycle

        # Verbose logging for cycles after receiving completes
        if hasattr(test_debug_single_prime, 'trace_start'):
            if cycle >= test_debug_single_prime.trace_start and cycle < test_debug_single_prime.trace_start + 15:
                bp1_state = int(dut.bp1_state.value)
                bp1_bp_param_valid = int(dut.bp1_inter_bp_param_valid.value)
                bp1_global_addr = int(dut.bp1.bp_core.addr_reg.value)
                bp1_inter_bp_addr = int(dut.bp1.inter_bp.bp_addr_out.value)
                dut._log.info(f"  TRACE {cycle}: state={bp1_state}, bp_param_valid={bp1_bp_param_valid}, global_addr={bp1_global_addr}, inter_bp_addr={bp1_inter_bp_addr}")

        # Track BP1 param valid (when inter_bp delivers deserialized params)
        if int(dut.bp1_inter_bp_param_valid.value) == 1 and not bp1_started:
            bp1_addr = int(dut.bp1.inter_bp.bp_addr_out.value)
            bp1_stride = int(dut.bp1.inter_bp.bp_stride_out.value)
            bp1_lambda = int(dut.bp1.inter_bp.bp_lambda_out.value)
            bp1_bp_state = int(dut.bp1_state.value)
            bp1_bp_addr_in = int(dut.bp1.bp_addr_in.value)
            dut._log.info(f"  Cycle {cycle}: BP1 bp_param_valid_in=1")
            dut._log.info(f"    inter_bp outputs: addr={bp1_addr}, stride={bp1_stride}, lambda={bp1_lambda}")
            dut._log.info(f"    bp_core.bp_addr_in={bp1_bp_addr_in}, state={bp1_bp_state}")
            bp1_started = True
            # Also check what the params_module sees
            bp1_params_addr = int(dut.bp1.bp_core.params_module.addr_out.value)
            bp1_params_bp_addr_in = int(dut.bp1.bp_core.params_module.bp_addr_in.value)
            dut._log.info(f"    params_module: bp_addr_in={bp1_params_bp_addr_in}, addr_out={bp1_params_addr}")

        # Track BP1 state transitions
        bp1_state = int(dut.bp1_state.value)

        # When BP1 enters WAIT_PARAMS
        if bp1_state == 4 and not hasattr(test_debug_single_prime, 'bp1_wait_params'):  # ST_SIEVE_WAIT_PARAMS
            test_debug_single_prime.bp1_wait_params = True
            bp1_params_ready = int(dut.bp1.bp_core.params_module.params_ready.value)
            bp1_params_addr = int(dut.bp1.bp_core.params_module.addr_out.value)
            dut._log.info(f"  Cycle {cycle}: BP1 in ST_SIEVE_WAIT_PARAMS")
            dut._log.info(f"    params_ready={bp1_params_ready}, addr_out={bp1_params_addr}")

        # When BP1 enters CHECK_RANGE
        if bp1_state == 5 and not hasattr(test_debug_single_prime, 'bp1_check_range'):  # ST_SIEVE_CHECK_RANGE
            test_debug_single_prime.bp1_check_range = True
            bp1_global_addr = int(dut.bp1.bp_core.addr_reg.value)
            bp1_params_addr = int(dut.bp1.bp_core.params_addr.value)
            dut._log.info(f"  Cycle {cycle}: BP1 in ST_SIEVE_CHECK_RANGE")
            dut._log.info(f"    global_addr_reg={bp1_global_addr}, params_addr={bp1_params_addr}")

        # When BP1 is sieving
        if bp1_state == 6:  # ST_SIEVE_READ
            bp1_addr = int(dut.bp1_addr.value)
            if bp1_addr not in bp1_sieve_addrs:
                bp1_sieve_addrs.append(bp1_addr)
                if len(bp1_sieve_addrs) <= 5:
                    dut._log.info(f"  Cycle {cycle}: BP1 sieving addr={bp1_addr}")
                    # On first sieve, show detailed debug info
                    if len(bp1_sieve_addrs) == 1:
                        bp1_global_addr = int(dut.bp1.bp_core.addr_reg.value)
                        bp1_params_addr = int(dut.bp1.bp_core.params_module.addr_out.value)
                        bp1_inter_bp_addr = int(dut.bp1.inter_bp.bp_addr_out.value)
                        dut._log.info(f"    global_addr_reg={bp1_global_addr}")
                        dut._log.info(f"    params_module.addr_out={bp1_params_addr}")
                        dut._log.info(f"    inter_bp.bp_addr_out={bp1_inter_bp_addr}")

        # Collect reports
        if int(dut.bp0_report_valid.value) == 1:
            report_addr = int(dut.bp0.bp_core.bp_report_addr_out.value)
            if report_addr not in reports:
                reports.append(report_addr)

        if int(dut.bp1_report_valid.value) == 1:
            report_addr = int(dut.bp1.bp_core.bp_report_addr_out.value)
            if report_addr not in reports:
                reports.append(report_addr)
                dut._log.info(f"  BP1 report at cycle {cycle}: addr={report_addr}")

        if int(dut.pipeline_busy.value) == 0:
            # Add grace period to allow BP1 to start processing
            await ClockCycles(dut.clk, 100)
            if int(dut.pipeline_busy.value) == 0:
                dut._log.info(f"  Cycle {cycle}: Pipeline idle, exiting loop")
                break

    # Check BP1's RAM at various addresses after sieving
    # RAM[0] corresponds to global addr 1024 (should be sieved)
    # RAM[2] corresponds to global addr 1026 (should be sieved)
    for local_addr in [0, 2, 4, 6, 8, 10]:
        ram_word = int(dut.bp1_ram.data[local_addr >> 2].value)
        ram_byte = (ram_word >> (8 * (local_addr & 0x3))) & 0xFF
        global_addr = 1024 + local_addr
        dut._log.info(f"BP1 RAM[{local_addr}] (global {global_addr}): {ram_byte}")

    # Check a few more RAM locations
    for local_addr in [10, 69, 171, 186, 340, 435]:
        global_addr = 1024 + local_addr
        ram_val = int(dut.bp1_ram.data[local_addr >> 2].value)
        ram_byte = (ram_val >> (8 * (local_addr & 0x3))) & 0xFF
        dut._log.info(f"BP1 RAM[{local_addr}] (global {global_addr}): {ram_byte}")

    dut._log.info(f"BP1 sieved {len(bp1_sieve_addrs)} unique addresses")
    dut._log.info(f"First 10 BP1 addresses: {sorted(bp1_sieve_addrs)[:10]}")
    dut._log.info(f"Total reports: {len(reports)}")

    dut._log.info("=" * 80)


@cocotb.test()
async def test_factor_10403_four_bps(dut):
    """Factor N = 10403 = 101 × 103 using 4-BP hardware pipeline via UIO.

    This test exercises the full quadratic sieve algorithm using four
    block processors communicating via the uio[7:4] 2-bit serial protocol,
    demonstrating ASIC-to-ASIC communication for distributed sieving.
    """
    dut._log.info("=" * 80)
    dut._log.info("QUADRATIC SIEVE - FOUR BP PIPELINE VIA UIO")
    dut._log.info("Factor N = 10403 = 101 × 103")
    dut._log.info("=" * 80)

    # Start clock
    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    # Reset
    dut._log.info("\n1. Resetting hardware (4 BPs wired via uio)...")
    dut.ui_in.value = 0
    dut.ena.value = 1
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)
    dut._log.info("   ✓ Reset complete")

    # Initialize quadratic sieve algorithm
    dut._log.info(f"\n2. Initializing QuadraticSieve(N={N}, bound={FACTOR_BASE_BOUND}, size={SIEVE_SIZE})...")
    qs = QuadraticSieve(N, FACTOR_BASE_BOUND, SIEVE_SIZE)
    dut._log.info(f"   N = {N}")
    dut._log.info(f"   sqrt(N) = {qs.sqrt_N}")
    dut._log.info(f"   Factor base size: {len(qs.factor_base)}")
    dut._log.info(f"   Factor base: {qs.factor_base}")
    dut._log.info(f"   Sieve size: {SIEVE_SIZE} (4 BPs × 1KB)")
    dut._log.info("   ✓ Algorithm initialized")

    # Run Python sieve for comparison
    dut._log.info(f"\n2b. Running Python sieve for comparison...")
    python_sieve = [0] * qs.sieve_size
    for p in qs.factor_base:
        if p == -1:
            continue
        params = qs.compute_bp_parameters(p)
        for addr, stride, lambda_val in params:
            x = addr
            while x < qs.sieve_size:
                python_sieve[x] += lambda_val
                x += stride
    python_candidates = set(i for i, val in enumerate(python_sieve) if val > THRESHOLD)
    dut._log.info(f"   Python found {len(python_candidates)} candidates above threshold {THRESHOLD}")
    dut._log.info(f"   First 10 Python candidates: {sorted(list(python_candidates))[:10]}")

    # Initialize all BPs (load threshold, clear RAM to zeros)
    dut._log.info(f"\n3. Initializing all BPs with threshold={THRESHOLD}...")
    dut.ui_in.value = MODE_INIT
    await ClockCycles(dut.clk, 2)

    # Load threshold
    await load_threshold(dut, THRESHOLD)
    dut._log.info(f"   ✓ Threshold loaded: {THRESHOLD}")

    # Wait for ALL BPs to complete initialization (clears RAM to zeros)
    init_cycles = 0
    max_init_cycles = 500000
    while True:
        await ClockCycles(dut.clk, 100)
        init_cycles += 100
        # All BPs must be not busy
        if (int(dut.bp0_busy.value) == 0 and int(dut.bp1_busy.value) == 0 and
            int(dut.bp2_busy.value) == 0 and int(dut.bp3_busy.value) == 0):
            break
        if init_cycles >= max_init_cycles:
            dut._log.error(f"   ✗ Initialization timeout ({init_cycles} cycles)")
            assert False, "Initialization timeout"

    dut._log.info(f"   ✓ All BPs initialized ({init_cycles} cycles)")

    # Verify thresholds
    bp0_thresh = int(dut.bp0_threshold.value)
    bp1_thresh = int(dut.bp1_threshold.value)
    bp2_thresh = int(dut.bp2_threshold.value)
    bp3_thresh = int(dut.bp3_threshold.value)
    dut._log.info(f"   Thresholds: BP0={bp0_thresh}, BP1={bp1_thresh}, BP2={bp2_thresh}, BP3={bp3_thresh}")

    # Switch to SIEVE mode
    dut.ui_in.value = MODE_SIEVE
    await ClockCycles(dut.clk, 5)

    # Process each prime in factor base
    dut._log.info(f"\n4. Processing {len(qs.factor_base)} primes from factor base...")
    dut._log.info(f"   Sieve spans: BP0=[0-1023], BP1=[1024-2047], BP2=[2048-3071], BP3=[3072-4095]")

    all_reports = set()
    primes_processed = 0

    # Trace first few primes in detail
    trace_primes = True
    primes_traced = 0

    for prime_idx, prime in enumerate(qs.factor_base):
        if prime == -1:
            continue  # Skip -1

        # Compute BP parameters for this prime
        params_list = qs.compute_bp_parameters(prime)

        if not params_list:
            continue

        for param_idx, (addr, stride, lambda_val) in enumerate(params_list):
            # Load parameters - they will automatically flow through the pipeline
            await load_bp_parameters(dut, addr, stride, lambda_val)

            # For the first few primes, trace what addresses get sieved in BP1
            if trace_primes and primes_traced < 3:
                # Wait for BP0 to finish sieving and forward to BP1
                # This can take thousands of cycles (BP0 sieves many addresses)
                for _ in range(100000):
                    await ClockCycles(dut.clk, 1)
                    # BP0 finished sieving when it goes idle
                    if int(dut.bp0_busy.value) == 0:
                        break

                # Now wait for BP1 to receive params from BP0
                for _ in range(100):
                    await ClockCycles(dut.clk, 1)
                    if int(dut.bp1_busy.value) == 1:
                        break

                # Check if BP1 received params
                if int(dut.bp1_busy.value) == 1:
                    bp1_params_addr = int(dut.bp1.bp_core.params_module.addr_out.value)
                    bp1_params_stride = int(dut.bp1.bp_core.params_module.stride_out.value)
                    bp1_params_lambda = int(dut.bp1.bp_core.params_module.lambda_out.value)
                    bp1_global_addr = int(dut.bp1.bp_core.addr_reg.value)

                    # Also check inter_bp outputs for debugging
                    bp1_inter_bp_stride = int(dut.bp1_inter_bp_stride.value)
                    bp1_inter_bp_lambda = int(dut.bp1_inter_bp_lambda.value)
                    dut._log.info(f"   DEBUG: inter_bp outputs: stride={bp1_inter_bp_stride}, lambda={bp1_inter_bp_lambda}")

                    # Calculate expected BP1 start address (relative addressing)
                    # BP0 sieves until addr >= LOCAL_SIZE, then forwards addr - LOCAL_SIZE
                    expected_global_addr = addr
                    while expected_global_addr < LOCAL_SIZE:
                        expected_global_addr += stride
                    expected_bp1_start = expected_global_addr - LOCAL_SIZE  # Relative address for BP1
                    expected_match = "✓" if bp1_params_addr == expected_bp1_start else f"✗ (expected {expected_bp1_start})"

                    dut._log.info(f"   TRACE p={prime}: BP1 params: addr={bp1_params_addr} {expected_match}, stride={bp1_params_stride}, lambda={bp1_params_lambda}")

            # Collect reports from the pipeline
            reports = await run_sieving_pipeline(dut, max_cycles=1000000)
            all_reports.update(reports)

        primes_processed += 1
        primes_traced += 1
        if primes_processed % 5 == 0:
            dut._log.info(f"   Processed {primes_processed}/{len(qs.factor_base)-1} primes, {len(all_reports)} candidates so far")

    dut._log.info(f"\n   ✓ Sieving complete: {len(all_reports)} total candidates reported")
    dut._log.info(f"   First 10 candidates: {sorted(list(all_reports))[:10]}")

    # Analyze which BP range each candidate falls into
    bp0_candidates = [a for a in all_reports if a < LOCAL_SIZE]
    bp1_candidates = [a for a in all_reports if LOCAL_SIZE <= a < 2*LOCAL_SIZE]
    bp2_candidates = [a for a in all_reports if 2*LOCAL_SIZE <= a < 3*LOCAL_SIZE]
    bp3_candidates = [a for a in all_reports if 3*LOCAL_SIZE <= a < 4*LOCAL_SIZE]
    dut._log.info(f"   Candidates by BP: BP0={len(bp0_candidates)}, BP1={len(bp1_candidates)}, BP2={len(bp2_candidates)}, BP3={len(bp3_candidates)}")

    # Compare with Python expectations
    dut._log.info(f"\n4b. Comparing hardware results to Python sieve...")
    hardware_set = set(all_reports)
    matches = hardware_set & python_candidates
    only_python = python_candidates - hardware_set
    only_hardware = hardware_set - python_candidates

    dut._log.info(f"   Hardware found: {len(hardware_set)} candidates")
    dut._log.info(f"   Python found:   {len(python_candidates)} candidates")
    dut._log.info(f"   Matches:        {len(matches)}")
    dut._log.info(f"   Only in Python: {len(only_python)} - MISSING FROM HARDWARE")
    dut._log.info(f"   Only in HW:     {len(only_hardware)}")

    # Assert exact match between hardware and Python
    assert hardware_set == python_candidates, (
        f"Candidate mismatch!\n"
        f"  Hardware: {len(hardware_set)} candidates\n"
        f"  Python:   {len(python_candidates)} candidates\n"
        f"  Missing:  {sorted(only_python)}\n"
        f"  Extra:    {sorted(only_hardware)}"
    )

    # Read actual RAM values for comparison
    dut._log.info(f"\n4c. Comparing actual RAM values...")

    # Helper to read RAM byte from a BP
    def read_ram_byte(bp_ram, local_addr):
        word_addr = local_addr >> 2
        byte_offset = local_addr & 0x3
        word = int(bp_ram.data[word_addr].value)
        return (word >> (8 * byte_offset)) & 0xFF

    # Check some addresses from each category
    addresses_to_check = set()
    if only_hardware:
        addresses_to_check.update(sorted(list(only_hardware))[:5])  # First 5 unexpected
    if only_python:
        addresses_to_check.update(sorted(list(only_python))[:5])   # First 5 missing

    for addr in sorted(addresses_to_check):
        if addr < LOCAL_SIZE:
            hw_value = read_ram_byte(dut.bp0_ram, addr)
            bp = "BP0"
            local = addr
        elif addr < 2 * LOCAL_SIZE:
            hw_value = read_ram_byte(dut.bp1_ram, addr - LOCAL_SIZE)
            bp = "BP1"
            local = addr - LOCAL_SIZE
        elif addr < 3 * LOCAL_SIZE:
            hw_value = read_ram_byte(dut.bp2_ram, addr - 2 * LOCAL_SIZE)
            bp = "BP2"
            local = addr - 2 * LOCAL_SIZE
        else:
            hw_value = read_ram_byte(dut.bp3_ram, addr - 3 * LOCAL_SIZE)
            bp = "BP3"
            local = addr - 3 * LOCAL_SIZE

        py_value = python_sieve[addr] if addr < len(python_sieve) else 0
        status = "MATCH" if hw_value == py_value else "DIFFER"
        category = "UNEXPECTED" if addr in only_hardware else "MISSING"
        dut._log.info(f"   {category} addr={addr} ({bp}[{local}]): HW={hw_value}, Python={py_value} - {status}")

    if only_hardware:
        extra_sorted = sorted(list(only_hardware))
        dut._log.info(f"   UNEXPECTED in HW: {extra_sorted}")
        # Check Python sieve values for extra addresses
        for addr in extra_sorted[:10]:
            if addr < len(python_sieve):
                dut._log.info(f"      addr={addr}: python_sieve={python_sieve[addr]}, threshold={THRESHOLD}")

    if only_python:
        missing_sorted = sorted(list(only_python))
        dut._log.info(f"   Missing candidates: {missing_sorted}")
        # Analyze which BP range the missing candidates fall into
        missing_bp0 = [a for a in only_python if a < LOCAL_SIZE]
        missing_bp1 = [a for a in only_python if LOCAL_SIZE <= a < 2*LOCAL_SIZE]
        missing_bp2 = [a for a in only_python if 2*LOCAL_SIZE <= a < 3*LOCAL_SIZE]
        missing_bp3 = [a for a in only_python if 3*LOCAL_SIZE <= a < 4*LOCAL_SIZE]
        dut._log.info(f"   Missing by BP: BP0={len(missing_bp0)}, BP1={len(missing_bp1)}, BP2={len(missing_bp2)}, BP3={len(missing_bp3)}")
        if missing_bp0:
            dut._log.info(f"   Missing from BP0: {sorted(missing_bp0)}")
        if missing_bp1:
            dut._log.info(f"   Missing from BP1: {sorted(missing_bp1)}")
        if missing_bp2:
            dut._log.info(f"   Missing from BP2: {sorted(missing_bp2)}")
        if missing_bp3:
            dut._log.info(f"   Missing from BP3: {sorted(missing_bp3)}")

        # Check Python sieve values for missing addresses
        dut._log.info(f"   Python sieve values for missing addresses:")
        for addr in missing_sorted[:10]:
            dut._log.info(f"      addr={addr}: python_sieve={python_sieve[addr]}, threshold={THRESHOLD}")

    # Verify smooth numbers
    dut._log.info("\n5. Verifying smooth relations...")
    smooth_relations = []
    seen_factors = set()
    smooth_count = 0
    not_smooth_count = 0

    for addr in sorted(all_reports):
        factors = qs.verify_smooth(addr)
        if factors is not None:
            factors_tuple = tuple(sorted(factors))
            if factors_tuple not in seen_factors:
                seen_factors.add(factors_tuple)
                rel = Relation(x=addr, addr=addr, value=0, factors=factors)
                smooth_relations.append(rel)
                smooth_count += 1
                if smooth_count <= 5:
                    dut._log.info(f"   ✓ addr={addr}: smooth! Factors: {factors}")
        else:
            not_smooth_count += 1

    dut._log.info(f"\n   Checked {len(all_reports)} candidates: {smooth_count} smooth, {not_smooth_count} not smooth")
    dut._log.info(f"   Found {len(smooth_relations)} unique smooth relations (after deduplication)")
    dut._log.info(f"   Need at least {len(qs.factor_base)} for matrix")

    if len(smooth_relations) < len(qs.factor_base):
        dut._log.error(f"   ERROR: Not enough smooth relations!")
        dut._log.error(f"   Found {len(smooth_relations)}, need {len(qs.factor_base)}")
        assert False, f"Not enough smooth relations: {len(smooth_relations)} < {len(qs.factor_base)}"

    # Build matrix
    dut._log.info("\n6. Building exponent matrix over GF(2)...")
    matrix = qs.build_matrix(smooth_relations)
    dut._log.info(f"   Matrix shape: {matrix.shape[0]} relations × {matrix.shape[1]} primes")
    dut._log.info("   ✓ Matrix built")

    # Find null space
    dut._log.info("\n7. Finding null space via Gaussian elimination...")
    null_vectors = find_null_space_gf2(matrix)

    if null_vectors is None or len(null_vectors) == 0:
        dut._log.error("   ERROR: No null vectors found")
        assert False, "No null vectors found"

    dut._log.info(f"   ✓ Found {len(null_vectors)} null vector(s)")

    # Extract factors
    dut._log.info("\n8. Extracting factors from null vectors...")
    factors_found = None

    for idx, null_vec in enumerate(null_vectors):
        dut._log.info(f"   Trying null vector {idx + 1}...")
        factors = qs.extract_factors(smooth_relations, null_vec)

        if factors is not None:
            dut._log.info(f"   ✓ Factors found: {factors[0]} × {factors[1]}")
            factors_found = factors
            break
        else:
            dut._log.info(f"   ✗ Trivial factorization")

    # Verify result
    dut._log.info("\n" + "=" * 80)
    if factors_found is not None:
        p, q = factors_found
        dut._log.info(f"SUCCESS! N = {p} × {q}")
        dut._log.info(f"Verification: {p} × {q} = {p * q}")
        assert p * q == N, f"Factorization incorrect: {p} × {q} = {p * q} ≠ {N}"
        assert p != 1 and q != 1, "Trivial factors"
        dut._log.info("✓✓✓ FACTORIZATION VERIFIED (4-BP PIPELINE VIA UIO) ✓✓✓")
    else:
        dut._log.error("ERROR: Could not extract non-trivial factors")
        assert False, "Could not extract non-trivial factors from any null vector"
    dut._log.info("=" * 80)


@cocotb.test()
async def test_concurrent_multi_prime_pipeline(dut):
    """Test that multiple primes can be sieving concurrently across the pipeline.

    This test verifies that:
    1. Multiple primes can be loaded in sequence
    2. Different BPs process different primes simultaneously (true pipelining)
    3. Final sieve values match the Python reference

    The key insight is that while BP0 is sieving prime P2, BP1 may still be
    processing forwarded parameters from prime P1. This overlap demonstrates
    proper pipelining behavior.
    """
    dut._log.info("=" * 80)
    dut._log.info("TEST: Concurrent Multi-Prime Pipeline")
    dut._log.info("Verify multiple primes can be in-flight across BPs simultaneously")
    dut._log.info("=" * 80)

    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    # Full reset - hold reset longer and ensure clean state
    dut.ui_in.value = 0
    dut.ena.value = 1
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 20)  # Longer reset
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)

    # Verify reset cleared the initialized flags by checking BP states
    dut._log.info(f"After reset: BP0 state={int(dut.bp0_state.value)}, BP1 state={int(dut.bp1_state.value)}")

    # Test parameters: multiple primes that will span multiple BPs
    # Use small strides to ensure lots of sieve hits per BP (more time for overlap)
    threshold = 10
    primes_to_test = [
        # (addr, stride, lambda_val) - designed to create concurrent activity
        (0, 7, 5),      # Small stride - lots of hits, will span all BPs
        (1, 11, 4),     # Different starting point, medium stride
        (3, 13, 6),     # Another offset, spans all BPs
        (5, 17, 3),     # Larger stride, still spans multiple BPs
        (2, 23, 7),     # Even larger stride
    ]

    # Compute expected sieve values using Python reference
    _, expected_candidates = compute_python_sieve(primes_to_test, TOTAL_SIEVE_SIZE, threshold)
    dut._log.info(f"Python expects {len(expected_candidates)} candidates above threshold {threshold}")

    # Initialize all BPs - this clears RAM to zeros
    dut._log.info("1. Initializing all BPs (clearing RAM)...")
    dut.ui_in.value = MODE_INIT
    await ClockCycles(dut.clk, 2)
    await load_threshold(dut, threshold)

    # Wait for all BPs to complete initialization
    init_wait_cycles = 0
    for _ in range(200000):
        await ClockCycles(dut.clk, 100)
        init_wait_cycles += 100
        if (int(dut.bp0_busy.value) == 0 and int(dut.bp1_busy.value) == 0 and
            int(dut.bp2_busy.value) == 0 and int(dut.bp3_busy.value) == 0):
            break
    dut._log.info(f"   All BPs initialized after {init_wait_cycles} cycles")

    # Verify thresholds were loaded correctly
    bp0_thresh = int(dut.bp0_threshold.value)
    dut._log.info(f"   Threshold loaded: BP0={bp0_thresh} (expected {threshold})")
    assert bp0_thresh == threshold, f"BP0 threshold mismatch: {bp0_thresh} != {threshold}"

    # Switch to SIEVE mode
    dut.ui_in.value = MODE_SIEVE
    await ClockCycles(dut.clk, 5)

    # Track concurrent activity statistics
    max_concurrent_bps = 0
    cycles_with_multiple_bps_busy = 0
    total_cycles_monitored = 0
    reports = set()  # Collect reports throughout the test

    # Helper to collect reports from all BPs
    def collect_reports():
        if int(dut.bp0_report_valid.value) == 1:
            report_addr = int(dut.bp0.bp_core.bp_report_addr_out.value)
            reports.add(report_addr)
        if int(dut.bp1_report_valid.value) == 1:
            report_addr = int(dut.bp1.bp_core.bp_report_addr_out.value)
            if report_addr < LOCAL_SIZE:
                reports.add(report_addr + LOCAL_SIZE)
        if int(dut.bp2_report_valid.value) == 1:
            report_addr = int(dut.bp2.bp_core.bp_report_addr_out.value)
            if report_addr < LOCAL_SIZE:
                reports.add(report_addr + 2 * LOCAL_SIZE)
        if int(dut.bp3_report_valid.value) == 1:
            report_addr = int(dut.bp3.bp_core.bp_report_addr_out.value)
            if report_addr < LOCAL_SIZE:
                reports.add(report_addr + 3 * LOCAL_SIZE)

    # Load primes and monitor for concurrent activity
    # The strategy: load a prime, then while waiting for BP0 to finish (and forward),
    # monitor for concurrent activity. Once BP0 is idle, immediately load next prime.
    # This should create overlap where BP1+ are still processing forwarded params
    # while BP0 starts on a new prime.
    dut._log.info(f"2. Loading {len(primes_to_test)} primes and monitoring for concurrent activity...")

    for idx, (addr, stride, lambda_val) in enumerate(primes_to_test):
        dut._log.info(f"   Loading prime {idx+1}: addr={addr}, stride={stride}, lambda={lambda_val}")
        await load_bp_parameters(dut, addr, stride, lambda_val)

        # Monitor while this prime is being processed
        # Continue until BP0 is idle (finished local sieving and forwarding)
        bp0_was_busy = False
        bp0_idle_cycles = 0
        for cycle in range(200000):
            await ClockCycles(dut.clk, 1)
            total_cycles_monitored += 1

            # Collect any reports generated during this cycle
            collect_reports()

            # Count how many BPs are busy simultaneously
            bp0_busy = int(dut.bp0_busy.value)
            bp1_busy = int(dut.bp1_busy.value)
            bp2_busy = int(dut.bp2_busy.value)
            bp3_busy = int(dut.bp3_busy.value)
            concurrent_count = bp0_busy + bp1_busy + bp2_busy + bp3_busy

            if bp0_busy:
                bp0_was_busy = True
                bp0_idle_cycles = 0
            elif bp0_was_busy:
                bp0_idle_cycles += 1

            if concurrent_count > max_concurrent_bps:
                max_concurrent_bps = concurrent_count
                dut._log.info(f"   Cycle {cycle}: New max concurrent BPs: {concurrent_count} "
                             f"(BP0={bp0_busy}, BP1={bp1_busy}, BP2={bp2_busy}, BP3={bp3_busy})")

            if concurrent_count >= 2:
                cycles_with_multiple_bps_busy += 1

            # Wait for BP0 to finish and stay idle for a few cycles
            # This allows BP1+ to start processing forwarded params before we load next prime
            if bp0_was_busy and bp0_idle_cycles >= 5:
                # BP0 has been idle for a few cycles - check if other BPs are busy
                if bp1_busy or bp2_busy or bp3_busy:
                    dut._log.info(f"   Prime {idx+1}: BP0 done, other BPs still busy: "
                                 f"BP1={bp1_busy}, BP2={bp2_busy}, BP3={bp3_busy}")
                break

    dut._log.info(f"\n3. All primes loaded. Waiting for pipeline to drain...")

    # Collect reports while pipeline finishes - also monitor during drain
    idle_cycles = 0
    required_idle_cycles = 50

    for _ in range(500000):
        await ClockCycles(dut.clk, 1)
        total_cycles_monitored += 1

        # Collect any reports generated during this cycle
        collect_reports()

        # Count concurrent BPs during drain phase too
        bp0_busy = int(dut.bp0_busy.value)
        bp1_busy = int(dut.bp1_busy.value)
        bp2_busy = int(dut.bp2_busy.value)
        bp3_busy = int(dut.bp3_busy.value)
        concurrent_count = bp0_busy + bp1_busy + bp2_busy + bp3_busy

        if concurrent_count > max_concurrent_bps:
            max_concurrent_bps = concurrent_count

        if concurrent_count >= 2:
            cycles_with_multiple_bps_busy += 1

        # Check if pipeline is done
        params_in_flight = (
            int(dut.bp0_inter_bp_transmitting.value) == 1 or
            int(dut.bp1_inter_bp_transmitting.value) == 1 or
            int(dut.bp2_inter_bp_transmitting.value) == 1 or
            int(dut.bp1_inter_bp_receiving.value) == 1 or
            int(dut.bp2_inter_bp_receiving.value) == 1 or
            int(dut.bp3_inter_bp_receiving.value) == 1
        )

        if int(dut.pipeline_busy.value) == 0 and not params_in_flight:
            idle_cycles += 1
            if idle_cycles >= required_idle_cycles:
                break
        else:
            idle_cycles = 0

    hw_candidates = reports

    # Report statistics
    dut._log.info(f"\n4. Pipeline activity statistics:")
    dut._log.info(f"   Max concurrent BPs busy: {max_concurrent_bps}")
    dut._log.info(f"   Cycles with >=2 BPs busy: {cycles_with_multiple_bps_busy}")
    dut._log.info(f"   Total cycles monitored: {total_cycles_monitored}")
    if total_cycles_monitored > 0:
        pct_concurrent = 100.0 * cycles_with_multiple_bps_busy / total_cycles_monitored
        dut._log.info(f"   Concurrency rate: {pct_concurrent:.1f}%")

    # Verify that we achieved true pipelining (at least 2 BPs working simultaneously)
    assert max_concurrent_bps >= 2, (
        f"Expected at least 2 BPs to be busy simultaneously, but max was {max_concurrent_bps}. "
        f"This suggests the pipeline is not operating concurrently."
    )
    dut._log.info(f"   ✓ Achieved concurrent operation: up to {max_concurrent_bps} BPs busy simultaneously")

    # Verify results match Python reference
    dut._log.info(f"\n5. Verifying sieve results...")
    dut._log.info(f"   Hardware found {len(hw_candidates)} candidates")
    dut._log.info(f"   Python expected {len(expected_candidates)} candidates")

    # Detailed comparison
    matches = hw_candidates & expected_candidates
    only_hw = hw_candidates - expected_candidates
    only_python = expected_candidates - hw_candidates

    dut._log.info(f"   Matches: {len(matches)}")
    if only_hw:
        dut._log.info(f"   Only in HW (unexpected): {len(only_hw)}")
        dut._log.info(f"   First few unexpected: {sorted(list(only_hw))[:10]}")
    if only_python:
        dut._log.info(f"   Only in Python (missing): {len(only_python)}")
        dut._log.info(f"   First few missing: {sorted(list(only_python))[:10]}")

    assert hw_candidates == expected_candidates, (
        f"Candidate mismatch with concurrent multi-prime pipeline!\n"
        f"  Hardware: {sorted(hw_candidates)}\n"
        f"  Python:   {sorted(expected_candidates)}\n"
        f"  Missing:  {sorted(only_python)}\n"
        f"  Extra:    {sorted(only_hw)}"
    )

    dut._log.info("=" * 80)
    dut._log.info("CONCURRENT MULTI-PRIME PIPELINE TEST PASSED")
    dut._log.info(f"Successfully processed {len(primes_to_test)} primes with up to "
                 f"{max_concurrent_bps} BPs working concurrently")
    dut._log.info("=" * 80)


@cocotb.test()
async def test_two_primes_simple(dut):
    """Minimal test with just 2 primes to verify basic multi-prime operation."""
    dut._log.info("=" * 80)
    dut._log.info("TEST: Two Primes Simple")
    dut._log.info("=" * 80)

    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    # Reset
    dut.ui_in.value = 0
    dut.ena.value = 1
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 20)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)

    # Just two primes with distinct patterns
    threshold = 15
    primes_to_test = [
        (100, 50, 10),   # Same as test_sieve_single_bp
        (200, 100, 10),  # Another simple prime
    ]

    # Compute expected results
    _, expected_candidates = compute_python_sieve(primes_to_test, TOTAL_SIEVE_SIZE, threshold)
    dut._log.info(f"Python expects {len(expected_candidates)} candidates")

    # Initialize
    dut.ui_in.value = MODE_INIT
    await ClockCycles(dut.clk, 2)
    await load_threshold(dut, threshold)

    for _ in range(200000):
        await ClockCycles(dut.clk, 100)
        if (int(dut.bp0_busy.value) == 0 and int(dut.bp1_busy.value) == 0 and
            int(dut.bp2_busy.value) == 0 and int(dut.bp3_busy.value) == 0):
            break

    # Check RAM is cleared
    word = int(dut.bp0_ram.data[25].value)  # Check address 100
    ram_val = (word >> (8 * (100 & 0x3))) & 0xFF
    dut._log.info(f"After init: BP0 RAM[100] = {ram_val}")

    dut.ui_in.value = MODE_SIEVE
    await ClockCycles(dut.clk, 5)

    # Process primes one at a time (same pattern as passing tests)
    all_reports = set()
    for idx, (addr, stride, lambda_val) in enumerate(primes_to_test):
        dut._log.info(f"Loading prime {idx+1}: addr={addr}, stride={stride}, lambda={lambda_val}")
        await load_bp_parameters(dut, addr, stride, lambda_val)
        reports = await run_sieving_pipeline(dut, max_cycles=500000)
        dut._log.info(f"  Got {len(reports)} reports from this prime")
        all_reports.update(reports)

    hw_candidates = all_reports

    dut._log.info(f"Hardware found {len(hw_candidates)} candidates")
    dut._log.info(f"Python expected {len(expected_candidates)} candidates")

    if hw_candidates != expected_candidates:
        dut._log.error(f"Missing: {sorted(expected_candidates - hw_candidates)}")
        dut._log.error(f"Extra: {sorted(hw_candidates - expected_candidates)}")

    assert hw_candidates == expected_candidates, "Two primes test failed"

    dut._log.info("TWO PRIMES SIMPLE TEST PASSED")
    dut._log.info("=" * 80)


@cocotb.test()
async def test_back_to_back_prime_loading(dut):
    """Test loading primes sequentially, each after the previous completes.

    This test verifies the pipeline correctly processes multiple primes
    when they are loaded one after another, waiting for each prime to
    complete processing through the entire pipeline before loading the next.
    """
    dut._log.info("=" * 80)
    dut._log.info("TEST: Sequential Multi-Prime Pipeline")
    dut._log.info("=" * 80)

    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    # Helper to read RAM byte
    def read_ram_byte_early(bp_ram, local_addr):
        word_addr = local_addr >> 2
        byte_offset = local_addr & 0x3
        word = int(bp_ram.data[word_addr].value)
        return (word >> (8 * byte_offset)) & 0xFF

    # Check RAM BEFORE reset to see stale values from previous tests
    dut._log.info("RAM values BEFORE reset (from previous tests):")
    before_values = []
    for addr in range(20):
        val = read_ram_byte_early(dut.bp0_ram, addr)
        if val != 0:
            before_values.append((addr, val))
    if before_values:
        dut._log.info(f"  Non-zero values: {before_values}")
    else:
        dut._log.info("  All first 20 addresses are 0")

    # Full reset - hold reset longer and ensure clean state
    dut.ui_in.value = 0
    dut.ena.value = 1
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 20)  # Longer reset
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)

    # Verify reset cleared the initialized flags by checking BP states
    dut._log.info(f"After reset: BP0 state={int(dut.bp0_state.value)}, BP1 state={int(dut.bp1_state.value)}")
    dut._log.info(f"After reset: BP0 initialized={int(dut.bp0_initialized.value)}")

    # Generate a sequence of primes for testing
    threshold = 12
    primes_to_test = [
        (0, 3, 4),
        (1, 3, 4),
        (0, 5, 3),
        (2, 5, 3),
        (0, 7, 3),
        (3, 7, 3),
        (0, 11, 3),
        (5, 11, 3),
    ]

    # Compute expected results
    _, expected_candidates = compute_python_sieve(primes_to_test, TOTAL_SIEVE_SIZE, threshold)
    dut._log.info(f"Testing with {len(primes_to_test)} prime parameters")
    dut._log.info(f"Python expects {len(expected_candidates)} candidates")

    # Initialize - this should clear RAM to zeros
    dut._log.info("Initializing (clearing RAM to zeros)...")
    dut.ui_in.value = MODE_INIT
    await ClockCycles(dut.clk, 2)
    await load_threshold(dut, threshold)

    # Check busy signals right after threshold loading
    await ClockCycles(dut.clk, 5)
    bp0_busy_start = int(dut.bp0_busy.value)
    bp1_busy_start = int(dut.bp1_busy.value)
    bp2_busy_start = int(dut.bp2_busy.value)
    bp3_busy_start = int(dut.bp3_busy.value)
    dut._log.info(f"After threshold load: busy BP0={bp0_busy_start}, BP1={bp1_busy_start}, BP2={bp2_busy_start}, BP3={bp3_busy_start}")

    # Wait for ALL BPs to complete initialization
    init_wait_cycles = 0
    for _ in range(200000):
        await ClockCycles(dut.clk, 100)
        init_wait_cycles += 100
        if (int(dut.bp0_busy.value) == 0 and int(dut.bp1_busy.value) == 0 and
            int(dut.bp2_busy.value) == 0 and int(dut.bp3_busy.value) == 0):
            break
    dut._log.info(f"Initialization complete after {init_wait_cycles} cycles")

    # Check if initialization actually completed for all BPs
    bp0_init = int(dut.bp0_initialized.value)
    bp1_init = int(dut.bp1_initialized.value)
    bp2_init = int(dut.bp2_initialized.value)
    bp3_init = int(dut.bp3_initialized.value)
    dut._log.info(f"After init: BP0_init={bp0_init}, BP1_init={bp1_init}, BP2_init={bp2_init}, BP3_init={bp3_init}")

    # Verify thresholds were loaded correctly
    bp0_thresh = int(dut.bp0_threshold.value)
    bp1_thresh = int(dut.bp1_threshold.value)
    bp2_thresh = int(dut.bp2_threshold.value)
    bp3_thresh = int(dut.bp3_threshold.value)
    dut._log.info(f"Thresholds: BP0={bp0_thresh}, BP1={bp1_thresh}, BP2={bp2_thresh}, BP3={bp3_thresh}")
    assert bp0_thresh == threshold, f"BP0 threshold mismatch: {bp0_thresh} != {threshold}"

    # Verify RAM was cleared by reading a few addresses
    def read_ram_byte(bp_ram, local_addr):
        word_addr = local_addr >> 2
        byte_offset = local_addr & 0x3
        word = int(bp_ram.data[word_addr].value)
        return (word >> (8 * byte_offset)) & 0xFF

    # Check ALL first 20 RAM locations for detailed debugging
    dut._log.info("Checking first 20 RAM addresses after initialization:")
    non_zero_addrs = []
    for addr in range(20):
        val = read_ram_byte(dut.bp0_ram, addr)
        if val != 0:
            non_zero_addrs.append((addr, val))
            dut._log.warning(f"  RAM[{addr}] = {val}")

    if non_zero_addrs:
        dut._log.warning(f"Found {len(non_zero_addrs)} non-zero addresses in first 20: {non_zero_addrs}")
    else:
        dut._log.info("  All first 20 addresses are 0 (good!)")

    # Also check a sampling across the entire RAM
    test_addrs = [100, 200, 300, 500, 700, 900, 1000]
    ram_ok = True
    for addr in test_addrs:
        val = read_ram_byte(dut.bp0_ram, addr)
        if val != 0:
            dut._log.warning(f"BP0 RAM[{addr}] = {val} (expected 0)")
            ram_ok = False
    if ram_ok:
        dut._log.info(f"RAM sample addresses {test_addrs} are all 0 (good!)")

    dut.ui_in.value = MODE_SIEVE
    await ClockCycles(dut.clk, 5)

    # Load primes one at a time, waiting for pipeline to drain between each
    dut._log.info("Loading primes sequentially...")
    all_reports = set()

    for idx, (addr, stride, lambda_val) in enumerate(primes_to_test):
        dut._log.info(f"  Prime {idx+1}/{len(primes_to_test)}: addr={addr}, stride={stride}, lambda={lambda_val}")
        await load_bp_parameters(dut, addr, stride, lambda_val)

        # Collect reports for this prime
        reports = await run_sieving_pipeline(dut, max_cycles=500000)
        all_reports.update(reports)

    hw_candidates = all_reports

    dut._log.info(f"Hardware found {len(hw_candidates)} candidates")
    dut._log.info(f"Python expected {len(expected_candidates)} candidates")

    # Verify
    assert hw_candidates == expected_candidates, (
        f"Sequential loading produced incorrect results!\n"
        f"  Missing: {sorted(expected_candidates - hw_candidates)}\n"
        f"  Extra:   {sorted(hw_candidates - expected_candidates)}"
    )

    dut._log.info("=" * 80)
    dut._log.info("SEQUENTIAL MULTI-PRIME PIPELINE TEST PASSED")
    dut._log.info("=" * 80)
