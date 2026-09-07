"""Prime number utilities: primality testing, sieving, and factorization.

Pure standard library, no dependencies. The module doubles as a CLI::

    python3 primes.py check 1000003
    python3 primes.py list 100
    python3 primes.py nth 10001
    python3 primes.py factor 600851475143
    python3 primes.py next 1000000
"""

from __future__ import annotations

import argparse
import itertools
import math
import random
import sys
from typing import Dict, Iterator, List

__all__ = [
    "is_prime",
    "sieve",
    "prime_generator",
    "nth_prime",
    "next_prime",
    "prev_prime",
    "prime_factors",
    "factorize",
    "divisors",
]

# Bases that make Miller-Rabin deterministic for every n < 3.3 * 10**24,
# which covers everything a 64-bit integer can hold and then some.
_DETERMINISTIC_BASES = (2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37)

_SMALL_PRIMES = (2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47)


def is_prime(n: int) -> bool:
    """Return True if ``n`` is prime.

    Uses trial division by small primes, then a Miller-Rabin test with a base
    set that is deterministic for any input that fits in 64 bits (and well
    beyond). Larger inputs get the same bases plus extra random ones, which
    makes a false positive vanishingly unlikely rather than impossible.
    """
    if n < 2:
        return False
    for p in _SMALL_PRIMES:
        if n % p == 0:
            return n == p

    # Write n - 1 as d * 2**s with d odd.
    d = n - 1
    s = 0
    while d % 2 == 0:
        d //= 2
        s += 1

    bases = list(_DETERMINISTIC_BASES)
    if n >= 3317044064679887385961981:
        bases += [random.randrange(2, n - 1) for _ in range(20)]

    for a in bases:
        a %= n
        if a == 0:
            continue
        x = pow(a, d, n)
        if x == 1 or x == n - 1:
            continue
        for _ in range(s - 1):
            x = x * x % n
            if x == n - 1:
                break
        else:
            return False
    return True


def sieve(limit: int) -> List[int]:
    """Return every prime ``p`` with ``p <= limit`` (sieve of Eratosthenes).

    Only odd candidates are stored, so the sieve needs about ``limit / 2``
    bytes.
    """
    if limit < 2:
        return []
    if limit < 3:
        return [2]

    # flags[i] tracks the odd number 2 * i + 1, up to the largest odd <= limit.
    size = (limit + 1) // 2
    flags = bytearray(b"\x01") * size
    flags[0] = 0  # 1 is not prime

    for i in range(1, math.isqrt(limit) // 2 + 1):
        if flags[i]:
            p = 2 * i + 1
            start = (p * p) // 2
            flags[start::p] = bytearray(len(range(start, size, p)))

    primes = [2]
    primes.extend(2 * i + 1 for i in range(1, size) if flags[i])
    return primes


def prime_generator() -> Iterator[int]:
    """Yield primes forever using an incremental sieve.

    Memory grows with the number of primes yielded, not with the size of the
    numbers tested, so this stays usable far past any fixed sieve limit.
    """
    yield 2
    # composites[c] holds the prime whose multiples marked c composite.
    composites: Dict[int, int] = {}
    for candidate in itertools.count(3, 2):
        p = composites.pop(candidate, None)
        if p is None:
            # candidate is prime; its first unmarked multiple is candidate**2.
            composites[candidate * candidate] = candidate
            yield candidate
        else:
            # Advance to the next odd multiple of p that is still free.
            step = 2 * p
            nxt = candidate + step
            while nxt in composites:
                nxt += step
            composites[nxt] = p


def nth_prime(n: int) -> int:
    """Return the ``n``-th prime, 1-indexed: ``nth_prime(1) == 2``."""
    if n < 1:
        raise ValueError("n must be >= 1")
    if n < 6:
        return (2, 3, 5, 7, 11)[n - 1]

    # Rosser's theorem: p_n < n * (ln n + ln ln n) for n >= 6.
    ln_n = math.log(n)
    limit = int(n * (ln_n + math.log(ln_n))) + 1
    primes = sieve(limit)
    return primes[n - 1]


def next_prime(n: int) -> int:
    """Return the smallest prime strictly greater than ``n``."""
    if n < 2:
        return 2
    candidate = n + 1
    if candidate % 2 == 0:
        if candidate == 2:
            return 2
        candidate += 1
    while not is_prime(candidate):
        candidate += 2
    return candidate


def prev_prime(n: int) -> int:
    """Return the largest prime strictly less than ``n``."""
    if n <= 2:
        raise ValueError("no prime below 2")
    if n == 3:
        return 2
    candidate = n - 1
    if candidate % 2 == 0:
        candidate -= 1
    while candidate > 2 and not is_prime(candidate):
        candidate -= 2
    return candidate


def _pollard_rho(n: int) -> int:
    """Return a non-trivial factor of composite, odd ``n`` (Brent's variant)."""
    if n % 2 == 0:
        return 2
    while True:
        x = random.randrange(2, n)
        y = x
        c = random.randrange(1, n)
        d = 1
        while d == 1:
            x = (x * x + c) % n
            y = (y * y + c) % n
            y = (y * y + c) % n
            d = math.gcd(abs(x - y), n)
        if d != n:
            return d


def prime_factors(n: int) -> List[int]:
    """Return the prime factors of ``n`` in ascending order, with repeats.

    ``prime_factors(360) == [2, 2, 2, 3, 3, 5]``. Trial division handles small
    factors; Pollard's rho takes over for the large ones, so numbers well past
    the reach of trial division still factor quickly.
    """
    if n < 2:
        raise ValueError("n must be >= 2")

    factors: List[int] = []
    for p in _SMALL_PRIMES:
        while n % p == 0:
            factors.append(p)
            n //= p

    stack = [n] if n > 1 else []
    while stack:
        m = stack.pop()
        if m == 1:
            continue
        if is_prime(m):
            factors.append(m)
            continue
        d = _pollard_rho(m)
        stack.append(d)
        stack.append(m // d)

    factors.sort()
    return factors


def factorize(n: int) -> Dict[int, int]:
    """Return the prime factorization of ``n`` as ``{prime: exponent}``."""
    counts: Dict[int, int] = {}
    for p in prime_factors(n):
        counts[p] = counts.get(p, 0) + 1
    return counts


def divisors(n: int) -> List[int]:
    """Return every positive divisor of ``n`` in ascending order."""
    if n < 1:
        raise ValueError("n must be >= 1")
    result = [1]
    if n == 1:
        return result
    for p, exp in factorize(n).items():
        result = [d * p**e for d in result for e in range(exp + 1)]
    result.sort()
    return result


def _format_factorization(n: int) -> str:
    parts = []
    for p, exp in sorted(factorize(n).items()):
        parts.append(str(p) if exp == 1 else f"{p}^{exp}")
    return " * ".join(parts)


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)

    p_check = sub.add_parser("check", help="test whether a number is prime")
    p_check.add_argument("n", type=int)

    p_list = sub.add_parser("list", help="list every prime up to a limit")
    p_list.add_argument("limit", type=int)
    p_list.add_argument(
        "--count", action="store_true", help="print how many instead of listing them"
    )

    p_nth = sub.add_parser("nth", help="print the nth prime (1-indexed)")
    p_nth.add_argument("n", type=int)

    p_factor = sub.add_parser("factor", help="print the prime factorization")
    p_factor.add_argument("n", type=int)

    p_next = sub.add_parser("next", help="print the next prime after a number")
    p_next.add_argument("n", type=int)

    p_prev = sub.add_parser("prev", help="print the previous prime before a number")
    p_prev.add_argument("n", type=int)

    args = parser.parse_args(argv)

    try:
        if args.command == "check":
            verdict = "prime" if is_prime(args.n) else "not prime"
            print(f"{args.n} is {verdict}")
        elif args.command == "list":
            primes = sieve(args.limit)
            print(len(primes) if args.count else " ".join(map(str, primes)))
        elif args.command == "nth":
            print(nth_prime(args.n))
        elif args.command == "factor":
            print(f"{args.n} = {_format_factorization(args.n)}")
        elif args.command == "next":
            print(next_prime(args.n))
        elif args.command == "prev":
            print(prev_prime(args.n))
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
