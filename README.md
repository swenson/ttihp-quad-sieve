![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg) ![](../../workflows/fpga/badge.svg)

# Block Processor for Quadratic Sieve Factoring

A hardware implementation of the Block Processor (BP) from the classic 1988 paper ["A Pipeline Architecture for Factoring Large Integers with the Quadratic Sieve Algorithm"](pipeline.pdf) by Carl Pomerance, J.W. Smith, and Randy Tuler.

- [Read the full documentation](docs/info.md)

## 🎯 Project Overview

This project brings a piece of computational number theory history to silicon! The quadratic sieve was one of the fastest factoring algorithms when this architecture was designed in the 1980s, and this ASIC implements the core "Block Processor" component that performs the sieving operation.

### What is the Quadratic Sieve?

The quadratic sieve is an algorithm for factoring large composite numbers. It works by:
1. Finding numbers that factor completely over a small set of primes (the "factor base")
2. Using these factorizations to construct a congruence of squares: X² ≡ Y² (mod N)
3. Computing GCD(X-Y, N) to find factors of N

The sieving stage is the computational bottleneck, making it ideal for hardware acceleration.

## 🏗️ Architecture

```
┌─────────────────────────────────────────┐
│      Block Processor Core               │
│                                         │
│  ┌──────────┐        ┌──────────────┐  │
│  │ Address  │        │    Data      │  │
│  │Arithmetic│        │  Arithmetic  │  │
│  │ A ← A+q  │        │ S[A]←S[A]+λ │  │
│  └──────────┘        └──────────────┘  │
│                                         │
│  ┌──────────────────────────────────┐  │
│  │   Control FSM                    │  │
│  │   (IDLE/INIT/SIEVE/REPORT)       │  │
│  └──────────────────────────────────┘  │
└─────────────────┬───────────────────────┘
                  │ SPI Interface
                  ▼
         ┌────────────────┐
         │  SPI RAM       │
         │  Controller    │
         └────────┬───────┘
                  │
                  ▼
         ┌────────────────┐
         │  23LC512       │
         │  64KB SRAM     │
         │  (External)    │
         └────────────────┘
```

**Key Design Decisions:**
- **External SPI RAM**: The 64KB sieve memory is stored in an external SPI SRAM chip (23LC512) since on-chip memory would be prohibitively large for Tiny Tapeout
- **Pipeline-Ready**: The BP can be chained with other BPs to form a pipeline (as described in the original paper)
- **Hardware Acceleration**: Address arithmetic (A ← A + q) and data arithmetic (S[A] ← S[A] + λ(q)) run in parallel

## 🔧 Features

- ✅ Four operating modes: IDLE, INIT, SIEVE, REPORT
- ✅ SPI RAM controller for 23LC512 or compatible 64KB SRAM
- ✅ Parallel address and data arithmetic units
- ✅ Threshold detection for reporting successful sieves
- ✅ Designed to fit Tiny Tapeout constraints (8 inputs, 8 outputs, 8 bidirectional)

## 📦 Hardware Required

To use this ASIC, you'll need:
- **23LC512 SPI SRAM** (64KB) - [Microchip part](https://www.microchip.com/en-us/product/23LC512)
- Pull-up resistor (10kΩ) for SPI CS line
- 3.3V power supply
- (Optional) Logic analyzer to observe SPI transactions

## 🚀 Quick Start

1. **Connect the external SRAM:**
   ```
   23LC512 Pin 1 (CS)   → uio[0]
   23LC512 Pin 2 (MISO) → uio[3]
   23LC512 Pin 5 (MOSI) → uio[2]
   23LC512 Pin 6 (SCK)  → uio[1]
   23LC512 Pin 4 (VSS)  → GND
   23LC512 Pin 8 (VCC)  → 3.3V
   23LC512 Pin 7 (HOLD) → 3.3V
   ```

2. **Initialize the sieve:**
   ```
   Set ui_in[1:0] = 01 (INIT mode)
   Wait for uo_out[0] to go low (not busy)
   ```

3. **Run a sieve:**
   ```
   Set ui_in[1:0] = 10 (SIEVE mode)
   Set ui_in[2] = 1 (data valid)
   Monitor uo_out[2] for reports
   ```

## 🧪 Testing

Run the comprehensive test suite:
```bash
cd test
make
```

The tests verify:
- ✓ Mode switching (IDLE, INIT, SIEVE, REPORT)
- ✓ SPI controller operation
- ✓ Address arithmetic (A ← A + q)
- ✓ Data arithmetic (S[A] ← S[A] + λ(q))
- ✓ Threshold detection
- ✓ Pin configuration

## 📚 Historical Context

In 1988, the authors of the original paper predicted that a custom hardware device costing $50,000 could factor 100-digit numbers in two weeks - an order of magnitude faster than contemporary supercomputers! They estimated that with $10 million in resources, 144-digit numbers could be factored in a year.

While modern factoring uses more advanced algorithms (like GNFS) and distributed computing has changed the landscape, the quadratic sieve architecture remains:
- An excellent teaching tool for understanding factorization algorithms
- Useful for numbers up to ~100 digits
- A beautiful example of hardware acceleration of number theory
- The foundation for understanding modern factoring methods

## 🎓 Learn More

- **Original Paper**: See [pipeline.pdf](pipeline.pdf) included in this repository
- **Tiny Tapeout**: Learn more at [tinytapeout.com](https://tinytapeout.com)
- **Documentation**: Full details in [docs/info.md](docs/info.md)

## 📊 Performance

At 10 MHz system clock:
- ~50,000 sieve updates per second
- Full 64KB initialization: ~1.3 seconds
- Each SPI RAM access: ~200 clock cycles

## 🔮 Future Enhancements

Potential improvements:
- [ ] Implement full pipeline with multiple chained BPs
- [ ] Add large prime variation optimization
- [ ] Support Montgomery's polynomial selection method
- [ ] Implement automatic threshold calculation
- [ ] Add polynomial switching capability

## 📄 License

This project is licensed under the Apache 2.0 License.

## 🙏 Acknowledgments

- Carl Pomerance, J.W. Smith, and Randy Tuler for the original 1988 paper
- The Tiny Tapeout team for making ASIC design accessible
- The cryptography and number theory community

---

*Built with [Tiny Tapeout](https://tinytapeout.com) - Democratizing chip design!*
