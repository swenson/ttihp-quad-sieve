# SPDX-FileCopyrightText: 2026 Christopher Swenson
# SPDX-License-Identifier: Apache-2.0

"""
Small quadratic sieve test - Factor N = 10403 = 101 × 103

This is a quick validation test using a 4KB sieve array
and a small factor base (primes up to 100). Expected runtime: ~15 seconds.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles
import sys
import os

# Import our QS algorithm
sys.path.insert(0, os.path.dirname(__file__))
from qs_algorithm import QuadraticSieve, Relation
from test_qs_algorithm_unit import find_null_space_gf2

# Mode definitions
MODE_IDLE = 0b00
MODE_INIT = 0b01
MODE_SIEVE = 0b10
MODE_REPORT = 0b11

# Test parameters (matching working Python test)
N = 10403  # = 101 × 103
FACTOR_BASE_BOUND = 100  # Factor base bound
SIEVE_SIZE = 4096  # Sieve array size (larger to find more relations)
THRESHOLD = 14  # Threshold tuned after fixing p=2 parameter bug


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

    # Send chunk 0 to trigger transition from IDLE to SIEVE_WAIT_PARAMS -> LOADING
    dut.ui_in.value = (chunks[0] << 3) | (data_valid << 2) | mode
    await ClockCycles(dut.clk, 1)

    # Clear data_valid briefly while waiting for state machine
    dut.ui_in.value = MODE_SIEVE
    await ClockCycles(dut.clk, 1)

    # Wait for params module to enter LOADING state (state=1 in params module)
    for _ in range(10):
        state = int(dut.user_project.bp_core.state.value)
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


async def run_sieving_and_collect_reports(dut, max_cycles=100000):
    """Run sieving and collect report addresses.

    Returns:
        List of report addresses where threshold was exceeded
    """
    reports = []

    # Set mode to SIEVE (should already be set, but make sure)
    dut.ui_in.value = MODE_SIEVE

    # Monitor internal_report_valid and internal_report_addr
    for _ in range(max_cycles):
        await ClockCycles(dut.clk, 1)

        # Check if report is valid
        if int(dut.internal_report_valid.value) == 1:
            report_addr = int(dut.internal_report_addr.value)
            reports.append(report_addr)

        # Check if BP is done (not busy)
        busy = int(dut.uo_out.value) & 0x01
        if busy == 0:
            # BP finished sieving
            break
    else:
        dut._log.error(f"Sieve cycle did not finish after {max_cycles}")
        assert False

    return reports


@cocotb.test()
async def test_factor_10403(dut):
    """Factor N = 10403 = 101 × 103 using hardware quadratic sieve."""
    dut._log.info("=" * 80)
    dut._log.info("QUADRATIC SIEVE SMALL TEST - FACTOR N = 10403")
    dut._log.info("=" * 80)

    # Start clock
    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    # Reset
    dut._log.info("\n1. Resetting hardware...")
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    dut.ena.value = 1
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)
    dut._log.info("   ✓ Reset complete")

    # Initialize quadratic sieve algorithm
    dut._log.info(
        f"\n2. Initializing QuadraticSieve(N={N}, bound={FACTOR_BASE_BOUND}, size={SIEVE_SIZE})..."
    )
    qs = QuadraticSieve(N, FACTOR_BASE_BOUND, SIEVE_SIZE)
    dut._log.info(f"   N = {N}")
    dut._log.info(f"   sqrt(N) = {qs.sqrt_N}")
    dut._log.info(f"   Factor base size: {len(qs.factor_base)}")
    dut._log.info(f"   Factor base: {qs.factor_base}")
    dut._log.info(f"   Sieve offset: {qs.sieve_offset}")
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

    python_candidates = [i for i, val in enumerate(python_sieve) if val > THRESHOLD]
    dut._log.info(
        f"   Python found {len(python_candidates)} candidates above threshold {THRESHOLD}"
    )
    dut._log.info(f"   First 10 Python candidates: {python_candidates[:10]}")

    # Initialize SPI RAM (load threshold, then clear to zeros)
    dut._log.info(f"\n3. Initializing SPI RAM with threshold={THRESHOLD}...")
    dut.ui_in.value = MODE_INIT
    await ClockCycles(dut.clk, 2)

    # Load threshold value
    await load_threshold(dut, THRESHOLD)
    dut._log.info(f"   ✓ Threshold loaded: {THRESHOLD}")

    # Wait for INIT to complete (monitor busy bit)
    init_cycles = 0
    max_init_cycles = 200000
    while True:
        await ClockCycles(dut.clk, 100)
        init_cycles += 100
        busy = int(dut.uo_out.value) & 0x01
        if busy == 0:
            break
        if init_cycles >= max_init_cycles:
            dut._log.error(f"   ✗ SPI RAM not initialized ({init_cycles} cycles)")
            stuck_addr = int(dut.user_project.ram_addr)
            dut._log.error(f"   Stuck at addr {stuck_addr}")
            assert False
    dut._log.info(f"   ✓ SPI RAM initialized ({init_cycles} cycles)")

    # Switch to SIEVE mode
    dut.ui_in.value = MODE_SIEVE
    await ClockCycles(dut.clk, 5)

    # Process each prime in factor base and collect reports during sieving
    dut._log.info(f"\n4. Processing {len(qs.factor_base)} primes from factor base...")
    dut._log.info(f"   Factor base: {qs.factor_base}")

    all_reports = set()  # Track all unique report addresses

    for prime_idx, prime in enumerate(qs.factor_base):
        if prime == -1:
            continue  # Skip -1

        # Compute BP parameters
        params_list = qs.compute_bp_parameters(prime)

        if not params_list:
            continue

        dut._log.info(
            f"     RAM[:10] = {[int(dut.ram.data[i].value) for i in range(10)]}"
        )
        for param_idx, (addr, stride, lambda_val) in enumerate(params_list):
            await load_bp_parameters(dut, addr, stride, lambda_val)
            reports = await run_sieving_and_collect_reports(dut, max_cycles=500000)
            all_reports.update(reports)
            dut._log.info(
                f"     RAM[:10] = {[int(dut.ram.data[i].value) for i in range(10)]}"
            )

        dut._log.info(
            f"   Processed {prime_idx + 1}/{len(qs.factor_base)} primes, {len(all_reports)} candidates so far"
        )

    dut._log.info(
        f"\n   ✓ Sieving complete: {len(all_reports)} total candidates reported"
    )

    # Debug: Check some RAM values
    dut._log.info(f"\n   Debug: Checking sample RAM values (threshold={THRESHOLD})...")
    sample_addrs = [116, 138, 180, 1634, 2062, 2165, 2191, 2339, 2750, 2820]
    for addr in sample_addrs:
        ram_val = int(dut.ram.data[addr >> 2].value)
        ram_val = (ram_val >> (8 * (addr & 0x3))) & 0xFF
        status = "ABOVE" if ram_val > THRESHOLD else "below"
        dut._log.info(f"     RAM[{addr}] = {ram_val} ({status})")

    # Direct comparison: Python sieve vs Hardware RAM
    dut._log.info(f"\n4b. Comparing Python sieve array to Hardware RAM array...")

    import math

    mismatches = []
    sample_addrs = list(range(100)) + list(
        range(1000, 1050)
    )  # Sample first 100 and some from middle

    for addr in sample_addrs:
        python_value = python_sieve[addr]
        hw_value = int(dut.ram.data[addr >> 2].value)
        hw_value = (hw_value >> (8 * (addr & 0x3))) & 0xFF

        if python_value != hw_value:
            mismatches.append((addr, python_value, hw_value))

    if mismatches:
        dut._log.info(f"   Found {len(mismatches)} mismatches in sampled addresses!")
        for addr, py_val, hw_val in mismatches[:10]:
            dut._log.info(
                f"   addr={addr:4d}: Python={py_val:2d}, HW={hw_val:2d}, diff={hw_val-py_val:+3d}"
            )
    else:
        dut._log.info(f"   ✓ All {len(sample_addrs)} sampled addresses match!")

    # Check some addresses that should have been sieved
    dut._log.info(f"\n   Detailed check on specific addresses:")
    test_addrs = [3, 7, 14, 15, 29]  # Known Python candidates
    for addr in test_addrs:
        python_value = python_sieve[addr]
        hw_value = int(dut.ram.data[addr >> 2].value)
        hw_value = (hw_value >> (8 * (addr & 0x3))) & 0xFF
        match = "✓" if python_value == hw_value else "✗"
        dut._log.info(
            f"   {match} addr={addr:4d}: Python={python_value:2d}, HW={hw_value:2d}"
        )

    # Compare hardware results to Python expectations
    dut._log.info(f"\n4c. Comparing hardware results to Python sieve...")
    hardware_candidates = sorted(list(all_reports))
    python_set = set(python_candidates)
    hardware_set = set(hardware_candidates)

    matches = hardware_set & python_set
    only_python = python_set - hardware_set
    only_hardware = hardware_set - python_set

    dut._log.info(f"   Hardware found: {len(hardware_candidates)} candidates")
    dut._log.info(f"   Python found:   {len(python_candidates)} candidates")
    dut._log.info(f"   Matches:        {len(matches)}")
    dut._log.info(f"   Only in Python: {len(only_python)}")
    dut._log.info(f"   Only in HW:     {len(only_hardware)}")

    if only_python:
        dut._log.info(
            f"   Candidates only in Python (first 10): {sorted(list(only_python))[:10]}"
        )
    if only_hardware:
        dut._log.info(
            f"   Candidates only in Hardware (first 10): {sorted(list(only_hardware))[:10]}"
        )

    # Verify smooth numbers (with deduplication to avoid structural null vectors)
    dut._log.info("\n5. Verifying smooth relations...")
    smooth_relations = []
    seen_factors = set()  # Deduplicate relations
    smooth_count = 0
    not_smooth_count = 0

    for addr in sorted(all_reports):
        factors = qs.verify_smooth(addr)
        if factors is not None:
            # Deduplicate based on prime factorization
            factors_tuple = tuple(sorted(factors))
            if factors_tuple not in seen_factors:
                seen_factors.add(factors_tuple)
                rel = Relation(x=addr, addr=addr, value=0, factors=factors)
                smooth_relations.append(rel)
                smooth_count += 1
                if smooth_count <= 10:  # Only show first 10
                    dut._log.info(f"   ✓ addr=0x{addr:04x}: smooth! Factors: {factors}")
        else:
            not_smooth_count += 1

    dut._log.info(
        f"\n   Checked {len(all_reports)} candidates: {smooth_count} smooth, {not_smooth_count} not smooth"
    )
    dut._log.info(
        f"   Found {len(smooth_relations)} unique smooth relations (after deduplication)"
    )
    dut._log.info(f"   Need at least {len(qs.factor_base)} for matrix")

    if len(smooth_relations) < len(qs.factor_base):
        dut._log.error(f"   ERROR: Not enough smooth relations!")
        dut._log.error(f"   Found {len(smooth_relations)}, need {len(qs.factor_base)}")
        dut._log.info("=" * 80)
        assert (
            False
        ), f"Not enough smooth relations: {len(smooth_relations)} < {len(qs.factor_base)}"

    # Build matrix
    dut._log.info("\n6. Building exponent matrix over GF(2)...")
    matrix = qs.build_matrix(smooth_relations)
    dut._log.info(
        f"   Matrix shape: {matrix.shape[0]} relations × {matrix.shape[1]} primes"
    )
    dut._log.info("   ✓ Matrix built")

    # Find null space (using corrected left null space finder)
    dut._log.info("\n7. Finding null space via Gaussian elimination...")
    null_vectors = find_null_space_gf2(matrix)

    if null_vectors is None or len(null_vectors) == 0:
        dut._log.error("   ERROR: No null vectors found (matrix has full rank)")
        dut._log.info("=" * 80)
        assert False, "No null vectors found"

    dut._log.info(f"   ✓ Found {len(null_vectors)} null vector(s)")

    # Try each null vector to extract factors
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
        dut._log.info("✓✓✓ FACTORIZATION VERIFIED ✓✓✓")
    else:
        dut._log.error("ERROR: Could not extract non-trivial factors")
        dut._log.info("=" * 80)
        assert False, "Could not extract non-trivial factors from any null vector"
    dut._log.info("=" * 80)
