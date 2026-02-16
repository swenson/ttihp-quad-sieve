# SPDX-FileCopyrightText: 2026 Christopher Swenson
# SPDX-License-Identifier: Apache-2.0

"""
Medium quadratic sieve test - Factor N = 604229 = 107 × 5647

This is a medium-scale validation test using a 4KB sieve array
and a moderate factor base (primes up to 250). Expected runtime: ~20 seconds.
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

# Test parameters
N = 604229  # = 773 × 781
FACTOR_BASE_BOUND = 500  # Medium factor base (increased for threshold accumulation)
SIEVE_SIZE = 8192  # 8KB sieve array (increased for more coverage)
THRESHOLD = (
    20  # Optimized threshold: max sieve value is 26, smooth relations range 9-26
)


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

    # Send chunk 0 to trigger transition from IDLE to SIEVE_WAIT_PARAMS
    dut.ui_in.value = (chunks[0] << 3) | (data_valid << 2) | mode
    await ClockCycles(dut.clk, 1)

    # Clear data_valid briefly while waiting for state machine
    dut.ui_in.value = MODE_SIEVE
    await ClockCycles(dut.clk, 1)

    # Wait for BP to enter SIEVE_WAIT_PARAMS state (state=4)
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
async def test_factor_604229(dut):
    """Factor N = 604229 = 107 × 5647 using hardware quadratic sieve."""
    dut._log.info("=" * 80)
    dut._log.info("QUADRATIC SIEVE MEDIUM TEST - FACTOR N = 604229")
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
    dut._log.info(f"   Factor base (first 10): {qs.factor_base[:10]}")
    dut._log.info(f"   Sieve offset: {qs.sieve_offset}")
    dut._log.info("   ✓ Algorithm initialized")

    # Initialize SPI RAM (load threshold, then clear to zeros)
    dut._log.info(f"\n3. Initializing SPI RAM with threshold={THRESHOLD}...")
    dut.ui_in.value = MODE_INIT
    await ClockCycles(dut.clk, 2)

    # Load threshold value
    await load_threshold(dut, THRESHOLD)
    dut._log.info(f"   ✓ Threshold loaded: {THRESHOLD}")

    # Wait for INIT to complete (monitor busy bit)
    init_cycles = 0
    max_init_cycles = 3000000
    while init_cycles < max_init_cycles:
        await ClockCycles(dut.clk, 100)
        init_cycles += 100
        busy = int(dut.uo_out.value) & 0x01
        if busy == 0:
            break

    dut._log.info(f"   ✓ SPI RAM initialized ({init_cycles} cycles)")

    # Switch to SIEVE mode
    dut.ui_in.value = MODE_SIEVE
    await ClockCycles(dut.clk, 5)

    # Track all reports from all primes
    all_reports = set()

    # Process each prime in factor base
    dut._log.info(f"\n4. Processing {len(qs.factor_base)} primes from factor base...")
    for prime_idx, prime in enumerate(qs.factor_base):
        if prime_idx % 5 == 0 or prime_idx < 3:
            dut._log.info(
                f"\n   Prime {prime_idx + 1}/{len(qs.factor_base)}: p = {prime}"
            )

        # Compute BP parameters
        params_list = qs.compute_bp_parameters(prime)

        if not params_list:
            if prime_idx < 3:
                dut._log.info(f"      No BP parameters for p={prime} (no square roots)")
            continue

        if prime_idx < 3:
            dut._log.info(f"      {len(params_list)} parameter set(s)")

        # Load and run each parameter set
        for param_idx, (addr, stride, lambda_val) in enumerate(params_list):
            if prime_idx < 3:
                dut._log.info(
                    f"      Set {param_idx + 1}: addr=0x{addr:04x}, stride=0x{stride:04x}, λ={lambda_val}"
                )

            # Load parameters
            await load_bp_parameters(dut, addr, stride, lambda_val)

            # Run sieving and collect reports
            reports = await run_sieving_and_collect_reports(dut, max_cycles=500000)

            if prime_idx < 3:
                dut._log.info(f"      Collected {len(reports)} reports")
            all_reports.update(reports)

    dut._log.info(f"\n   ✓ Sieving complete - {len(all_reports)} unique candidate(s)")

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
                if smooth_count <= 20:
                    dut._log.info(
                        f"   ✓ addr=0x{addr:04x}: smooth! Factors: {factors[:5]}..."
                    )
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

    # Find null space
    dut._log.info("\n7. Finding null space via Gaussian elimination...")
    null_vectors = qs.gaussian_elimination_gf2(matrix)
    dut._log.info(
        f"   qs.gaussian_elimination_gf2 returned: {len(null_vectors) if null_vectors else 0} vectors"
    )

    # Also try find_null_space_gf2 for comparison
    null_vectors2 = find_null_space_gf2(matrix)
    dut._log.info(
        f"   find_null_space_gf2 returned: {len(null_vectors2) if null_vectors2 else 0} vectors"
    )

    # Use whichever gives more vectors
    if null_vectors2 and (not null_vectors or len(null_vectors2) > len(null_vectors)):
        dut._log.info(f"   Using find_null_space_gf2 results")
        null_vectors = null_vectors2

    if null_vectors is None or len(null_vectors) == 0:
        dut._log.error("   ERROR: No null vectors found (matrix has full rank)")
        dut._log.info("=" * 80)
        assert False, "No null vectors found"

    dut._log.info(f"   ✓ Found {len(null_vectors)} null vector(s)")

    # Try each null vector to extract factors
    dut._log.info("\n8. Extracting factors from null vectors...")
    factors_found = None

    for idx, null_vec in enumerate(null_vectors):
        if idx < 5:
            dut._log.info(f"   Trying null vector {idx + 1}...")
        factors = qs.extract_factors(smooth_relations, null_vec)

        if factors is not None:
            dut._log.info(f"   ✓ Factors found: {factors[0]} × {factors[1]}")
            factors_found = factors
            break
        else:
            if idx < 5:
                dut._log.info(f"   ✗ Trivial factorization")

    # Verify result
    dut._log.info("\n" + "=" * 80)
    if factors_found is not None:
        p, q = factors_found
        dut._log.info(f"SUCCESS! N = {p} × {q}")
        dut._log.info(f"Verification: {p} × {q} = {p * q}")
        assert p * q == N, f"Factorization incorrect: {p} × {q} = {p * q} ≠ {N}"
        assert p != 1 and q != 1, "Trivial factors"
        assert p == 107 or p == 5647, f"Expected factors 107 and 5647, got {p} and {q}"
        dut._log.info("✓✓✓ FACTORIZATION VERIFIED ✓✓✓")
    else:
        dut._log.error("ERROR: Could not extract non-trivial factors")
        dut._log.info("=" * 80)
        assert False, "Could not extract non-trivial factors from any null vector"
    dut._log.info("=" * 80)
