"""Tests for the primes module. Run with: python3 -m unittest -v"""

import itertools
import unittest

from primes import (
    divisors,
    factorize,
    is_prime,
    next_prime,
    nth_prime,
    prev_prime,
    prime_factors,
    prime_generator,
    sieve,
)

PRIMES_UNDER_100 = [
    2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47,
    53, 59, 61, 67, 71, 73, 79, 83, 89, 97,
]


def naive_is_prime(n):
    """Obvious O(sqrt n) reference implementation, used as an oracle."""
    if n < 2:
        return False
    i = 2
    while i * i <= n:
        if n % i == 0:
            return False
        i += 1
    return True


class TestIsPrime(unittest.TestCase):
    def test_small_numbers(self):
        self.assertEqual([n for n in range(100) if is_prime(n)], PRIMES_UNDER_100)

    def test_matches_naive_oracle(self):
        for n in range(-10, 2000):
            self.assertEqual(is_prime(n), naive_is_prime(n), n)

    def test_negatives_and_zero_and_one(self):
        for n in (-7, -1, 0, 1):
            self.assertFalse(is_prime(n))

    def test_large_primes(self):
        for p in (1000003, 32416190071, 2**31 - 1, 2**61 - 1):
            self.assertTrue(is_prime(p), p)

    def test_large_composites(self):
        for c in (1000001, 32416190073, 2**31 - 3, 600851475143):
            self.assertFalse(is_prime(c), c)

    def test_carmichael_numbers_are_not_prime(self):
        # Fermat pseudoprimes to every coprime base; Miller-Rabin must reject.
        for c in (561, 1105, 1729, 2465, 2821, 6601, 8911, 62745, 162401):
            self.assertFalse(is_prime(c), c)

    def test_strong_pseudoprime_to_base_2(self):
        self.assertFalse(is_prime(2047))  # 23 * 89


class TestSieve(unittest.TestCase):
    def test_small_limits(self):
        self.assertEqual(sieve(-5), [])
        self.assertEqual(sieve(0), [])
        self.assertEqual(sieve(1), [])
        self.assertEqual(sieve(2), [2])
        self.assertEqual(sieve(3), [2, 3])
        self.assertEqual(sieve(4), [2, 3])

    def test_primes_under_100(self):
        self.assertEqual(sieve(100), PRIMES_UNDER_100)

    def test_limit_is_inclusive(self):
        self.assertIn(97, sieve(97))
        self.assertNotIn(97, sieve(96))

    def test_agrees_with_is_prime(self):
        limit = 10000
        self.assertEqual(sieve(limit), [n for n in range(limit + 1) if is_prime(n)])

    def test_known_prime_counts(self):
        # pi(10^k) for k = 1..6
        self.assertEqual(len(sieve(10)), 4)
        self.assertEqual(len(sieve(100)), 25)
        self.assertEqual(len(sieve(1000)), 168)
        self.assertEqual(len(sieve(10**4)), 1229)
        self.assertEqual(len(sieve(10**5)), 9592)
        self.assertEqual(len(sieve(10**6)), 78498)


class TestPrimeGenerator(unittest.TestCase):
    def test_first_primes(self):
        got = list(itertools.islice(prime_generator(), len(PRIMES_UNDER_100)))
        self.assertEqual(got, PRIMES_UNDER_100)

    def test_agrees_with_sieve(self):
        limit = 5000
        got = list(itertools.takewhile(lambda p: p <= limit, prime_generator()))
        self.assertEqual(got, sieve(limit))


class TestNthPrime(unittest.TestCase):
    def test_first_few(self):
        for i, p in enumerate(PRIMES_UNDER_100, start=1):
            self.assertEqual(nth_prime(i), p, i)

    def test_known_values(self):
        self.assertEqual(nth_prime(1000), 7919)
        self.assertEqual(nth_prime(10001), 104743)  # Project Euler 7

    def test_rejects_non_positive(self):
        for n in (0, -1):
            with self.assertRaises(ValueError):
                nth_prime(n)


class TestNeighbours(unittest.TestCase):
    def test_next_prime(self):
        self.assertEqual(next_prime(-10), 2)
        self.assertEqual(next_prime(1), 2)
        self.assertEqual(next_prime(2), 3)
        self.assertEqual(next_prime(3), 5)
        self.assertEqual(next_prime(7919), 7927)
        self.assertEqual(next_prime(1000000), 1000003)

    def test_next_prime_is_strictly_greater(self):
        for n in range(2, 500):
            p = next_prime(n)
            self.assertGreater(p, n)
            self.assertTrue(is_prime(p))
            self.assertFalse(any(is_prime(k) for k in range(n + 1, p)))

    def test_prev_prime(self):
        self.assertEqual(prev_prime(3), 2)
        self.assertEqual(prev_prime(4), 3)
        self.assertEqual(prev_prime(100), 97)
        self.assertEqual(prev_prime(1000003), 999983)

    def test_prev_prime_below_two_raises(self):
        for n in (2, 1, 0, -5):
            with self.assertRaises(ValueError):
                prev_prime(n)


class TestFactorization(unittest.TestCase):
    def test_prime_factors(self):
        self.assertEqual(prime_factors(2), [2])
        self.assertEqual(prime_factors(4), [2, 2])
        self.assertEqual(prime_factors(360), [2, 2, 2, 3, 3, 5])
        self.assertEqual(prime_factors(97), [97])

    def test_large_semiprime(self):
        self.assertEqual(prime_factors(600851475143), [71, 839, 1471, 6857])
        self.assertEqual(prime_factors(1000003 * 32416190071), [1000003, 32416190071])

    def test_product_round_trips(self):
        for n in range(2, 1000):
            factors = prime_factors(n)
            self.assertTrue(all(is_prime(p) for p in factors), n)
            product = 1
            for p in factors:
                product *= p
            self.assertEqual(product, n)

    def test_rejects_below_two(self):
        for n in (1, 0, -3):
            with self.assertRaises(ValueError):
                prime_factors(n)

    def test_factorize_exponents(self):
        self.assertEqual(factorize(360), {2: 3, 3: 2, 5: 1})
        self.assertEqual(factorize(97), {97: 1})

    def test_divisors(self):
        self.assertEqual(divisors(1), [1])
        self.assertEqual(divisors(12), [1, 2, 3, 4, 6, 12])
        self.assertEqual(divisors(97), [1, 97])
        self.assertEqual(divisors(36), [1, 2, 3, 4, 6, 9, 12, 18, 36])

    def test_divisors_brute_force(self):
        for n in range(1, 300):
            self.assertEqual(divisors(n), [d for d in range(1, n + 1) if n % d == 0], n)

    def test_divisors_rejects_non_positive(self):
        with self.assertRaises(ValueError):
            divisors(0)


if __name__ == "__main__":
    unittest.main()
