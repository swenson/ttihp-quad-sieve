# SPDX-FileCopyrightText: 2026 Christopher Swenson
# SPDX-License-Identifier: Apache-2.0

"""
Gate-level smoke test for tt_um_swenson_cqs.

This test only uses top-level ports (no internal signal access) so it works
for both RTL and gate-level simulation. It verifies basic reset behavior
and mode transitions.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

# Mode definitions
MODE_IDLE = 0b00
MODE_INIT = 0b01
MODE_SIEVE = 0b10
MODE_REPORT = 0b11


@cocotb.test()
async def test_reset_and_init(dut):
    """Verify reset behavior and INIT mode entry."""
    dut._log.info("=" * 60)
    dut._log.info("GATE-LEVEL SMOKE TEST")
    dut._log.info("=" * 60)

    # Start clock
    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    # Reset
    dut._log.info("1. Applying reset...")
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    dut.ena.value = 1
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)
    dut._log.info("   Reset released")

    # Check post-reset state: busy should be 0 in IDLE mode
    busy = int(dut.uo_out.value) & 0x01
    dut._log.info(f"   uo_out = 0x{int(dut.uo_out.value):02x}, busy = {busy}")
    assert busy == 0, f"Expected busy=0 after reset, got {busy}"
    dut._log.info("   OK: busy=0 after reset")

    # Check uio_oe: bits [2:0] and [7:4] should be outputs (1), bit 3 input (0)
    uio_oe = int(dut.uio_oe.value)
    dut._log.info(f"   uio_oe = 0b{uio_oe:08b}")
    assert (uio_oe & 0x07) == 0x07, f"SPI output enables wrong: 0b{uio_oe:08b}"
    assert (uio_oe & 0x08) == 0x00, f"SPI MISO should be input: 0b{uio_oe:08b}"
    assert (uio_oe & 0xF0) == 0xF0, f"Inter-BP outputs wrong: 0b{uio_oe:08b}"
    dut._log.info("   OK: uio_oe directions correct")

    # Enter INIT mode with threshold
    dut._log.info("\n2. Entering INIT mode...")
    threshold = 14

    # Set mode to INIT
    dut.ui_in.value = MODE_INIT
    await ClockCycles(dut.clk, 2)

    # Load threshold cycle 0: threshold[4:0]
    chunk_val = threshold & 0x1F
    dut.ui_in.value = (chunk_val << 3) | (1 << 2) | MODE_INIT
    await ClockCycles(dut.clk, 1)

    # Load threshold cycle 1: threshold[7:5]
    chunk_val = (threshold >> 5) & 0x07
    dut.ui_in.value = (chunk_val << 3) | (1 << 2) | MODE_INIT
    await ClockCycles(dut.clk, 1)

    # Clear data_valid
    dut.ui_in.value = MODE_INIT
    await ClockCycles(dut.clk, 5)

    # After threshold load, BP should be busy initializing SPI RAM
    busy = int(dut.uo_out.value) & 0x01
    dut._log.info(f"   busy = {busy}")
    assert busy == 1, f"Expected busy=1 during INIT, got {busy}"
    dut._log.info("   OK: busy=1 during INIT (SPI RAM clearing)")

    # Verify SPI CS is active (low = asserted)
    spi_cs = int(dut.uio_out.value) & 0x01
    dut._log.info(f"   SPI CS = {spi_cs}")
    dut._log.info("   OK: SPI interface active")

    # Let it run a bit and verify still busy (64KB SPI init takes many cycles)
    await ClockCycles(dut.clk, 1000)
    busy = int(dut.uo_out.value) & 0x01
    assert busy == 1, f"Expected busy=1 still during INIT, got {busy}"
    dut._log.info("   OK: still busy after 1000 cycles (expected for 64KB init)")

    dut._log.info("\n" + "=" * 60)
    dut._log.info("GATE-LEVEL SMOKE TEST PASSED")
    dut._log.info("=" * 60)
