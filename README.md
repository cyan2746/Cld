# primes

Prime number utilities in Python — primality testing, sieving, and
factorization. Standard library only, no dependencies.

## Library

```python
from primes import is_prime, sieve, nth_prime, prime_factors, divisors

is_prime(2**61 - 1)        # True
sieve(30)                  # [2, 3, 5, 7, 11, 13, 17, 19, 23, 29]
nth_prime(10001)           # 104743
next_prime(1_000_000)      # 1000003
prime_factors(600851475143)  # [71, 839, 1471, 6857]
factorize(360)             # {2: 3, 3: 2, 5: 1}
divisors(36)               # [1, 2, 3, 4, 6, 9, 12, 18, 36]
```

`prime_generator()` yields primes indefinitely via an incremental sieve, so it
is not bounded by a fixed limit:

```python
import itertools
list(itertools.islice(prime_generator(), 5))  # [2, 3, 5, 7, 11]
```

### How it works

- **`is_prime`** — trial division by the first few primes, then Miller-Rabin
  with the base set `{2, 3, ..., 37}`, which is *deterministic* for every
  `n < 3.3e24` (so every 64-bit input gets a proven answer, Carmichael numbers
  included). Above that range, extra random bases are added.
- **`sieve`** — sieve of Eratosthenes over odd candidates only, using a
  `bytearray` and slice assignment for the striking-out. Roughly `limit / 2`
  bytes of memory; sieving to 10^7 takes well under a second.
- **`prime_factors`** — small factors by trial division, the rest by Pollard's
  rho with a Miller-Rabin check at each split, so semiprimes far beyond the
  reach of trial division still factor instantly.

## CLI

```
$ python3 primes.py check 1000003
1000003 is prime

$ python3 primes.py list 60
2 3 5 7 11 13 17 19 23 29 31 37 41 43 47 53 59

$ python3 primes.py list 1000000 --count
78498

$ python3 primes.py nth 10001
104743

$ python3 primes.py factor 600851475143
600851475143 = 71 * 839 * 1471 * 6857

$ python3 primes.py next 1000000
1000003

$ python3 primes.py prev 1000000
999983
```

## Tests

```
python3 -m unittest -v
```

29 tests: small values checked against a naive `O(sqrt n)` oracle, known values
of `pi(10^k)` and the 10001st prime, Carmichael numbers and base-2 strong
pseudoprimes, round-tripping every factorization back to its product, and
`divisors` against brute force.

## Also in this repository

`matlab/` holds an unrelated project: an **aircraft pitch / longitudinal
autopilot** design suite (root locus/PID, pole placement, LQR, LQI, observer,
nonlinear verification, comparative analysis and a programmatically built
Simulink model). See [docs/aircraft_pitch_autopilot.md](docs/aircraft_pitch_autopilot.md).
