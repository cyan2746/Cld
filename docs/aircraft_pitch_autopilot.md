# Aircraft pitch / longitudinal autopilot

Attitude-command autopilot for the classic short-period pitch model, designed
around a **pitch-up command relative to the current trim attitude** and a
**gust upset**, with hard elevator travel limits of **+25 deg** (nose-up
authority) and **-15 deg** (nose-down authority).

## Files

| file | what it is |
|---|---|
| `matlab/aircraft_pitch_autopilot.m` | the whole pipeline: model, analysis, root locus/PID, pole placement, LQR, LQI, observer, nonlinear verification, comparison, command shaping, Simulink build |
| `matlab/build_pitch_autopilot_simulink.m` | builds the Simulink model programmatically (called by the script, usable on its own) |
| `docs/aircraft_pitch_autopilot_console_output.txt` | the console output of a full run, for reference |

Run it with

```matlab
cd matlab
aircraft_pitch_autopilot
```

Needs the Control System Toolbox. Simulink is optional — if it is missing the
script says so and skips only Section 12. The script also runs on GNU Octave
with the `control` package (`pidtune` and the Simulink build are skipped).

## The command interface

Everything a user sets lives in **Section 0** of the script:

```matlab
cfg.theta_trim_deg   =  0;   % attitude the aircraft is trimmed at right now
cfg.target_theta_deg = 10;   % PITCH-UP COMMAND, measured from that trim attitude
cfg.gust_theta_deg   =  3;   % pitch upset the gust produces
cfg.gust_time_s      =  2;   % when it hits
cfg.gust_rise_s      =  0.5; % how long the gust takes to build the upset
cfg.elev_up_deg      = +25;  % elevator travel limits
cfg.elev_dn_deg      = -15;
```

So `target_theta_deg` is an **increment**: trimmed at 10 deg with
`target_theta_deg = 3` means fly to 13 deg. Trimmed at 10 deg with
`target_theta_deg = 0` means *hold* 10 deg, which is the configuration used for
the gust scenario — the gust throws the nose up to 13 deg and the autopilot has
to bring it back to 10.

Every plot shows the **absolute** attitude `theta_trim + deviation` in degrees
with the target drawn on it, and the **elevator command** next to it against
the +25 / -15 deg limits.

## Model and trim convention

```
alpha_dot = -0.313*alpha + 56.7*q   + 0.232*delta_e
q_dot     = -0.0139*alpha - 0.426*q + 0.0203*delta_e
theta_dot =  56.7*q
```

* All states are **deviations from the trim condition**; the absolute attitude
  is `theta_trim + theta`.
* `theta_dot = 56.7*q`, so the state `q` is a *scaled* pitch rate — the
  physical pitch rate is `56.7*q`. Every weight and every display in the script
  accounts for that factor explicitly.
* Column 3 of `A` is zero: attitude does not feed back into the dynamics. For
  this reduced model the design is therefore **independent of the trim
  attitude** — trimming at 0 deg or at 10 deg gives the same gains, and
  `theta_trim` only shifts the absolute attitude that is plotted. (A full model
  would change the stability derivatives with speed and load factor.)
* Positive elevator is **nose up** here; the script verifies that from the sign
  of the DC pitch-rate gain (0.1925 deg/s of pitch rate per deg of elevator, so
  4.81 deg/s at the +25 deg stop).
* The gust is injected as a **pitch-rate disturbance pulse whose area is the
  requested upset**, so the attitude is displaced by `gust_theta_deg` and stays
  displaced until the autopilot corrects it. That is physically what a gust
  does, and unlike an instantaneous state jump it does not put an infinite
  derivative into the loop.

## What limits this airframe

`theta(s)/delta_e(s) = (1.151 s + 0.1774) / (s^3 + 0.739 s^2 + 0.9215 s)` has a
**slow left-half-plane zero at s = -0.154** (time constant 6.49 s). For any
closed loop with `T(0) = 1`,

```
integral of (theta_cmd - theta) dt  =  -T'(0)  =  sum(1/|pole|) - 6.49  [per unit command]
```

* **Without** an integrator in the controller (P, LQR) one closed-loop pole is
  forced to be slower than the zero, so the step response is monotonic but
  drags a long tail — and a standing mis-trim leaves a bias.
* **With** an integrator (PID, LQI) there are two integrators in the loop,
  `T'(0) = 0` exactly, and the rise deficit is paid for with a small overshoot
  instead of a tail.

That single number explains the whole PID/LQR/LQI ranking, and Section 11 shows
the standard way out: shaping the command through a first-order filter moves
the identity by `+tau_shape` and buys a monotonic response with a much smaller
elevator kick.

## The three designs

**PID (Section 4)** — PI-D form: proportional and integral on the error,
filtered derivative on the *measurement*, so a step command produces no
derivative kick and the peak elevator demand is exactly `Kp * target_theta`.
That is what ties the gain directly to the travel limits. Anti-windup is
back-calculation, `xi_dot = e + (u_sat - u_cmd)/tau_aw`, the same law that is
implemented in the Simulink model.

**LQR (Section 6)** — Bryson's rule: every state weighted by
`1/(largest acceptable excursion)^2`, the elevator by `rho/delta_max^2`, so
`rho` is a single interpretable knob. The command enters through the attitude
channel, `u = -K*(x - [0;0;theta_cmd])`.

**LQI (Section 7)** — the LQR augmented with `z_dot = theta_cmd - theta` and
`u = -Kx*x - Kz*z` (identical to MATLAB's `lqi`). The command reaches the
elevator only through the integrator, so the elevator starts from zero and
builds up: no kick, and the least elevator of the three for a planned
manoeuvre.

**Observer (Section 8)** — only attitude is measured, so `alpha` and `q` are
estimated. `alpha` is weakly observable (it reaches `theta` only through the
`-0.0139` coefficient), so a fast estimator needs a huge gain, amplifies sensor
noise, and — the expensive part — converts an unmodelled gust into a large
elevator transient. The script audits a hand-placed Luenberger family against a
steady-state Kalman family (process noise through the elevator channel,
`Qn = qw*B*B'`) on the loop that actually flies, and picks the **fastest**
estimator that still meets the phase-margin, modulus-margin and elevator specs.

**Elevator budget** — after each design the script checks the peak elevator
demand for both specified cases: the planned command (allowed
`cfg.spec.authority` of the travel, default 90 %) and the specified gust
(allowed all of it, but it must not saturate). If either is exceeded the loop
gain is de-rated automatically and the binding case is printed. The script also
reports the largest command and the largest gust each design can take before
saturating.

## Verification (Section 9)

Every run uses the full loop: **nonlinear airframe + first-order servo +
asymmetric saturation + rate limit + anti-windup + observer**, integrated with
a fixed-step RK4 so the non-smooth limits are handled consistently and every
controller is compared on the identical time grid.

| run | what it exercises |
|---|---|
| 9a | attitude step to `target_theta` |
| 9b | gust upset while holding the target |
| 9c | command, settle, then take the gust |
| 9d | standing elevator mis-trim — separates integral action from pure regulation |
| 9e | oversized command — saturation and anti-windup |
| 9f | nose-down command — the tighter -15 deg limit |
| 9g | +/-30 % model error with the controller and observer on the nominal model |

## Results

Nonlinear verification, `+10 deg` command from `0 deg` trim, `+3 deg` gust,
elevator `+25 / -15 deg`, servo `0.1 s` / `60 deg/s`, attitude-only measurement
with the auto-selected estimator.

**Attitude step (9a)** — settling inside +/-0.20 deg:

| | rise (s) | settle (s) | overshoot | peak elevator | authority used | IAE (deg*s) |
|---|---|---|---|---|---|---|
| PID | 1.07 | 17.59 | 13.5 % | +16.78 deg | 67 % | 20.1 |
| LQR | 13.13 | 29.80 | 0.0 % | +16.77 deg | 67 % | 44.5 |
| LQI | **1.48** | **3.71** | 3.3 % | **+9.84 deg** | **39 %** | **14.7** |

**Gust rejection (9b)** — recovery inside +/-0.10 deg of the target:

| | peak excursion | recovery (s) | peak elevator | authority used | IAE (deg*s) |
|---|---|---|---|---|---|
| PID | 2.74 deg | 14.79 | -12.58 deg | 84 % | **4.41** |
| LQR | 2.99 deg | 37.88 | -1.35 deg | 9 % | 32.87 |
| LQI | 2.97 deg | **13.42** | -5.12 deg | 34 % | 6.32 |

All three let almost the whole upset through, and that is not a design failure:
a 3 deg upset built in 0.5 s is 6 deg/s of pitch rate, while full elevator can
only generate 4.81 deg/s nose-up and 2.89 deg/s nose-down. **The gust cannot be
opposed while it happens** — the designs differ only in how they recover. The
script prints that arithmetic in Section 2.

**Standing mis-trim (9d, -2 deg of elevator bias)** — final attitude error:
PID `0.000 deg`, LQI `0.000 deg`, LQR `-4.237 deg`. Analytically the bias should
cost the LQR only `bias/K_theta = -1.19 deg`; the rest comes from the observer,
which cannot see the bias, so the large pitch-rate gain amplifies its own
estimation error. Nothing integrates, so nothing removes it.

**Score card** (weighted, 0 = best on every criterion):

| | PID | LQR | LQI |
|---|---|---|---|
| phase margin | 48.3 deg | 91.9 deg | 38.1 deg |
| weighted score | 0.473 | 0.780 | **0.150** |

**Ranking: LQI, then PID, then LQR** — and the reasons are the ones Section 2
predicted:

* **LQR** has no integrator, so a closed-loop pole is forced slower than the
  plant zero: a beautifully monotone step response (0 % overshoot) that drags a
  30 s tail, and a standing error whenever the model or the trim is wrong. It
  has by far the best margins (91.9 deg) and is the right choice if regulation,
  not command tracking, is the job.
* **PID** buys the integrator with one sensor and no model. The tail becomes a
  13 % overshoot and the mis-trim error goes to zero, but it only sees attitude:
  damping has to come from a filtered derivative, which costs elevator (67 % of
  authority for the command, 84 % for the gust).
* **LQI** has the integrator *and* alpha/q feedback, so it damps the short
  period without waiting for a derivative — fastest settling (3.71 s), least
  elevator (39 %), zero steady error. It pays with an observer, a model it has
  to trust, and the smallest phase margin of the three.

Final gains printed by the script:

```
PID : Kp = 1.600  Ki = 0.700  Kd = 1.000  N = 8.0     (derivative on measurement)
LQR : K  = [+0.0936  +85.7824  +1.6771]
LQI : Kx = [-0.6481 +130.2550  +4.4642]  Kz = -3.2141
OBS : L  = [+0.7628  +0.0053  +0.7718]'  (Kalman, qw = 1e-6)
```

Section 11 then shows what a command filter buys: with `tau = 2 s` the LQI
overshoot drops from 3.3 % to 0.0 % and the elevator kick from 9.8 to 4.3 deg,
at the price of a slower, deliberate manoeuvre.

## Simulink model (Section 12)

`build_pitch_autopilot_simulink` creates one model with **one autopilot branch
per controller**, all driven by the same command, gust and mis-trim sources, so
the three designs can be compared on one pair of scopes (attitude in degrees,
elevator command in degrees with the travel limits drawn as constants).

Each branch is built from primitives so that it matches the MATLAB simulation
exactly:

```
command -> controller -> travel saturation -> [ (u_sat - delta)/tau -> rate saturation
        -> integrator with position limits ] -> airframe -> theta
                                                              |
                                       observer (theta only) <-+
```

The elevator is modelled as `delta_dot = sat_rate((sat_travel(u_cmd) - delta)/tau)`
with `delta` itself limited to `[elev_dn, elev_up]`, which is the same equation
the script integrates. Signals are logged to the workspace as
`theta_deg_PID`, `delta_cmd_deg_PID`, and so on. An existing model of the same
name is moved aside to `<name>.slx.bak` rather than deleted.

## Retuning

* Different manoeuvre or gust: change Section 0 and re-run — the elevator
  budget, the de-rating and every metric follow automatically.
* Different PID feel: `cfg.pid.*`.
* Different LQR/LQI aggressiveness: `cfg.spec.rho_lqr`, `cfg.spec.rho_lqi`, and
  the Bryson maxima `cfg.spec.*_max_*`.
* Different sensor suite: `cfg.use_observer`, `cfg.obs_method`
  (`'kalman'`/`'place'`), `cfg.obs_poles`, `cfg.theta_noise_deg`,
  `cfg.sim_noise`.
* Ideal-actuator study: `cfg.tau_act`, `cfg.elev_rate_deg`.
* Linear-only verification: `cfg.use_nonlinear = false`.
