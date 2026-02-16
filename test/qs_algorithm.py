# SPDX-FileCopyrightText: 2026 Christopher Swenson
# SPDX-License-Identifier: Apache-2.0

"""
Quadratic Sieve Algorithm Implementation for cocotb testing.
Based on the hardware BP sieving approach.

This module implements a complete quadratic sieve factorization algorithm,
including:
- Factor base generation with Legendre symbol testing
- Tonelli-Shanks algorithm for modular square roots
- BP parameter computation for hardware sieving
- Smooth number verification via trial division
- GF(2) Gaussian elimination for finding dependencies
- Factor extraction using GCD
"""

import math
from typing import List, Tuple, Optional
import numpy as np
from dataclasses import dataclass, field


@dataclass
class Relation:
    """A smooth relation: (x + offset)² - N is B-smooth over the factor base."""

    x: int  # Offset value
    addr: int  # Address in sieve array (same as x for our implementation)
    value: int  # Sieve value that exceeded threshold
    factors: Optional[List[int]] = None  # Prime factorization (filled later)


class QuadraticSieve:
    """Quadratic Sieve factorization algorithm."""

    def __init__(self, N: int, factor_base_bound: int = 1103, sieve_size: int = 65536):
        """Initialize quadratic sieve for factoring N.

        Args:
            N: Number to factor
            factor_base_bound: Upper bound for primes in factor base
            sieve_size: Size of sieve array (default 64KB)
        """
        self.N = N
        self.sqrt_N = int(math.isqrt(N))

        self.factor_base_bound = factor_base_bound
        self.sieve_size = sieve_size
        self.factor_base = self._build_factor_base()

        # Calculate sieve interval offset
        # We'll sieve X values in range [offset, offset + sieve_size)
        # For best results, center around sqrt(N)
        self.sieve_offset = self.sqrt_N

    def _is_prime(self, n: int) -> bool:
        """Miller-Rabin primality test."""
        if n < 2:
            return False
        if n == 2 or n == 3:
            return True
        if n % 2 == 0:
            return False

        # Small prime check
        for p in [3, 5, 7, 11, 13, 17, 19, 23, 29, 31]:
            if n == p:
                return True
            if n % p == 0:
                return False

        # Write n-1 as 2^r * d
        r, d = 0, n - 1
        while d % 2 == 0:
            r += 1
            d //= 2

        # Test with multiple witnesses
        for a in [2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37]:
            if a >= n:
                continue

            x = pow(a, d, n)
            if x == 1 or x == n - 1:
                continue

            for _ in range(r - 1):
                x = pow(x, 2, n)
                if x == n - 1:
                    break
            else:
                return False

        return True

    def _legendre_symbol(self, a: int, p: int) -> int:
        """Compute Legendre symbol (a/p) using Euler's criterion.

        Returns:
            1 if a is a quadratic residue mod p
            -1 if a is a non-residue
            0 if a ≡ 0 (mod p)
        """
        result = pow(a, (p - 1) // 2, p)
        if result == p - 1:
            return -1
        return result

    def _tonelli_shanks(self, n: int, p: int) -> Optional[Tuple[int, Optional[int]]]:
        """Find square roots of n modulo p using Tonelli-Shanks algorithm.

        Returns:
            (r1, r2) where r1² ≡ r2² ≡ n (mod p), or None if no solution exists.
            If r1 == r2 (only one root), r2 will be None.
        """
        # Check if solution exists
        if self._legendre_symbol(n % p, p) != 1:
            return None

        # Handle special cases
        if p == 2:
            r = n % 2
            return (r, None)

        if p % 4 == 3:
            # Simple case: p ≡ 3 (mod 4)
            r = pow(n, (p + 1) // 4, p)
            return (r, p - r if r != p - r else None)

        # General Tonelli-Shanks algorithm for p ≡ 1 (mod 4)
        # Write p-1 = 2^s * q with q odd
        q = p - 1
        s = 0
        while q % 2 == 0:
            q //= 2
            s += 1

        # Find a quadratic non-residue z
        z = 2
        while self._legendre_symbol(z, p) != -1:
            z += 1

        # Initialize variables
        m = s
        c = pow(z, q, p)
        t = pow(n, q, p)
        r = pow(n, (q + 1) // 2, p)

        while t != 1:
            # Find least i such that t^(2^i) = 1
            i = 1
            temp = (t * t) % p
            while temp != 1:
                temp = (temp * temp) % p
                i += 1

            # Update values
            b = pow(c, 1 << (m - i - 1), p)
            m = i
            c = (b * b) % p
            t = (t * c) % p
            r = (r * b) % p

        # Return both roots
        if r != p - r:
            return (r, p - r)
        else:
            return (r, None)

    def _build_factor_base(self) -> List[int]:
        """Build factor base: primes p where N is a quadratic residue mod p.

        Returns:
            List of primes in the factor base
        """
        factor_base = []

        # Special case: -1 for tracking signs
        factor_base.append(-1)

        for p in range(2, self.factor_base_bound + 1):
            if not self._is_prime(p):
                continue

            # Check if N is quadratic residue mod p
            if p == 2:
                # 2 is always included
                factor_base.append(p)
            elif self._legendre_symbol(self.N % p, p) == 1:
                factor_base.append(p)

        return factor_base

    def compute_bp_parameters(self, prime: int) -> List[Tuple[int, int, int]]:
        """Compute BP parameters (addr, stride, lambda) for a given prime.

        For a prime p in the factor base, we need to find all X values where
        (X + offset)² ≡ N (mod p). These form an arithmetic progression with
        stride p, starting from roots r where r² ≡ N (mod p).

        Args:
            prime: A prime in the factor base

        Returns:
            List of (addr, stride, lambda) tuples. Usually 2 tuples (one per root),
            or 1 tuple if there's only one root, or 0 tuples if no roots exist.
        """
        p = prime

        # Special case: -1 and 2 don't generate sieving parameters
        if p == -1:
            return []
        if p == 2:
            # For p=2, we need x=addr+offset to be ODD (since x²≡1 mod 2 when x is odd)
            # If offset is odd, addr must be even (0); if offset is even, addr must be odd (1)
            addr = 1 - (self.sieve_offset % 2)
            return [(addr, 2, 1)]  # lambda for 2 is 1

        # Calculate lambda = floor(log2(p))
        lambda_val = int(math.log2(p))

        # Get square roots of N mod p
        roots_tuple = self._tonelli_shanks(self.N % p, p)

        if roots_tuple is None:
            return []

        root1, root2 = roots_tuple
        roots = [root1] if root2 is None else [root1, root2]

        params = []
        for root in roots:
            # We want X such that (X + offset)² ≡ N (mod p)
            # This means X + offset ≡ ±root (mod p)
            # So X ≡ ±root - offset (mod p)

            # We want the smallest non-negative X
            addr = (root - (self.sieve_offset % p)) % p
            stride = p
            params.append((addr, stride, lambda_val))

        return params

    def verify_smooth(self, addr: int) -> Optional[List[int]]:
        """Verify that Q(X) = (X + offset)² - N factors over the factor base.

        Args:
            addr: Address in sieve array (X value)

        Returns:
            List of prime factors (with multiplicity) if smooth, None otherwise
        """
        x = addr + self.sieve_offset
        q_value = x * x - self.N

        if q_value == 0:
            # Special case: exact square root
            return [self.N]

        if q_value < 0:
            # Shouldn't happen if offset >= sqrt(N)
            return None

        factors = []
        remaining = q_value

        # Try all primes in factor base
        for p in self.factor_base:
            if p == -1:
                # Track sign separately
                continue

            while remaining % p == 0:
                factors.append(p)
                remaining //= p

        # Check if completely factored
        if remaining != 1:
            return None

        return factors

    def build_matrix(self, relations: List[Relation]) -> np.ndarray:
        """Build exponent matrix over GF(2) from relations.

        Matrix M where M[i][j] = 1 if prime factor_base[j] appears
        an odd number of times in relation i.

        Args:
            relations: List of smooth relations

        Returns:
            m × n binary matrix where m = len(relations), n = len(factor_base)
        """
        m = len(relations)
        n = len(self.factor_base)

        matrix = np.zeros((m, n), dtype=np.uint8)

        for i, rel in enumerate(relations):
            if rel.factors is None:
                continue

            # Count occurrences of each prime
            for factor in rel.factors:
                try:
                    j = self.factor_base.index(factor)
                    matrix[i, j] ^= 1  # XOR for GF(2)
                except ValueError:
                    # Factor not in base (shouldn't happen for smooth numbers)
                    pass

        return matrix

    def gaussian_elimination_gf2(
        self, matrix: np.ndarray
    ) -> Optional[List[np.ndarray]]:
        """Find null space of matrix over GF(2) using Gaussian elimination.

        Args:
            matrix: m × n binary matrix

        Returns:
            List of null vectors (binary vectors v where matrix @ v = 0 in GF(2))
            or None if no null space exists
        """
        m, n = matrix.shape

        # Create augmented matrix [A | I]
        # We'll reduce A to reduced row echelon form
        aug_matrix = np.hstack([matrix.copy(), np.eye(m, dtype=np.uint8)])

        # Gaussian elimination with partial pivoting
        pivot_row = 0
        pivot_cols = []

        for col in range(n):
            # Find pivot
            found_pivot = False
            for row in range(pivot_row, m):
                if aug_matrix[row, col] == 1:
                    # Swap rows if needed
                    if row != pivot_row:
                        aug_matrix[[pivot_row, row]] = aug_matrix[[row, pivot_row]]
                    found_pivot = True
                    break

            if not found_pivot:
                # No pivot in this column, it's a free variable
                continue

            pivot_cols.append(col)

            # Eliminate other rows
            for row in range(m):
                if row != pivot_row and aug_matrix[row, col] == 1:
                    aug_matrix[row] ^= aug_matrix[pivot_row]  # XOR in GF(2)

            pivot_row += 1

        # Find free variables (columns without pivots)
        free_cols = [col for col in range(n) if col not in pivot_cols]

        if not free_cols:
            # No null space (full rank)
            return None

        # Construct null vectors
        null_vectors = []

        for free_col in free_cols:
            # Create null vector
            null_vec = np.zeros(m, dtype=np.uint8)

            # For each pivot row, determine if this relation is needed
            for pivot_idx, pivot_col in enumerate(pivot_cols):
                if pivot_idx >= m:
                    break
                if aug_matrix[pivot_idx, free_col] == 1:
                    null_vec[pivot_idx] = 1

            # Mark the free variable itself
            # We need to find which relations correspond to the free column
            # In our augmented matrix representation, we track via the identity part

            null_vectors.append(null_vec)

        # Alternative approach: extract null space from reduced form
        # Find columns that are all zeros (these give us null vectors)
        null_vectors = []
        for col in range(n):
            if np.all(aug_matrix[:pivot_row, col] == 0):
                # This column is in the null space
                # Extract the dependency from the augmented part
                null_vec = np.zeros(m, dtype=np.uint8)
                for row in range(min(pivot_row, m)):
                    if aug_matrix[row, col] == 1:
                        null_vec[row] = 1
                null_vectors.append(null_vec)

        return null_vectors if null_vectors else None

    def extract_factors(
        self, relations: List[Relation], null_vector: np.ndarray
    ) -> Optional[Tuple[int, int]]:
        """Extract factors from a null space vector using congruence of squares.

        The null vector indicates which relations to multiply together.
        If the null vector is correct, the product will be a perfect square.

        Args:
            relations: List of smooth relations
            null_vector: Binary vector indicating which relations to use

        Returns:
            (factor1, factor2) where N = factor1 * factor2, or None if trivial
        """
        # Build X = product of (x_i + offset) for selected relations (mod N)
        x_product = 1

        # Count prime factorizations for Y (should all be even)
        prime_counts = {}

        for i, bit in enumerate(null_vector):
            if i >= len(relations):
                break
            if bit == 0:
                continue

            rel = relations[i]

            # Multiply X values
            x_val = rel.addr + self.sieve_offset
            x_product = (x_product * x_val) % self.N

            # Accumulate prime factors
            if rel.factors:
                for factor in rel.factors:
                    prime_counts[factor] = prime_counts.get(factor, 0) + 1

        # Build Y from even prime powers
        y = 1
        for prime, count in prime_counts.items():
            if prime == -1:
                # Sign doesn't matter for Y²
                continue
            if count % 2 != 0:
                # This shouldn't happen if null vector is correct
                # But we'll try anyway
                pass
            # Take half the power
            y = (y * pow(prime, count // 2, self.N)) % self.N

        # Now we have X² ≡ Y² (mod N)
        # Try GCD(X ± Y, N)
        for sign in [1, -1]:
            g = math.gcd(x_product + sign * y, self.N)
            if g != 1 and g != self.N:
                # Found non-trivial factor!
                other = self.N // g
                return (min(g, other), max(g, other))

        # Trivial factorization, try next null vector
        return None
