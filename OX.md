# OX.md - Balance Stabilization Strategy & Implementation Directive

> **Purpose:** This document is the authoritative engineering plan for achieving stable two-wheel
> balancing. It records the diagnosis, the reasoning behind every decision, the tradeoffs accepted,
> and the exact code changes required. It is written so that any agent (or human) can implement it
> without re-deriving the analysis.
>
> **Status:** Approved strategy (2026-08-22). Operator-confirmed dominant failure mode:
> **"Kick-up rarely succeeds - it never gets to vertical."**
> All downstream work (gains, damping, tuner) is unreachable until this gate is passed.
>
> **Governing rules:** All work must comply with [AGENTS.md](AGENTS.md) Sections 2, 6, and 7.
> Highlights enforced throughout this plan: no hardcoded physical constants, deadman switch AND/OR
> time budget on every physical trial, `make lint && make test` on all Python edits, mandatory
> JOURNAL.md entries per experiment, incrementalism over silver bullets, and confirmation of data
> interpretation with the operator before acting on telemetry.

---

## Table of Contents

1. [Diagnosis](#1-diagnosis-why-the-robot-cannot-balance)
2. [Strategy Overview & Phase Ordering](#2-strategy-overview--phase-ordering)
3. [Phase 0 Spec - Blackbox Forensics](#3-phase-0-spec---blackbox-forensics)
4. [Phase 1 Spec - State Estimation Fixes](#4-phase-1-spec---state-estimation-fixes)
5. [Phase 2 Spec - Rock-and-Flip Kick-Up](#5-phase-2-spec---rock-and-flip-kick-up)
6. [Phase 3+ Specs - Deferred Work](#6-phase-3-specs---deferred-work)
7. [Decision & Tradeoff Log](#7-decision--tradeoff-log)
8. [Validation Checklist & Journal Protocol](#8-validation-checklist--journal-protocol)

---

## 1. Diagnosis: Why The Robot Cannot Balance

Findings are ordered by causal priority. Each cites exact evidence locations.

### F0 - CRITICAL: The kick-up pulse collapses to ~one control tick (~10 ms)

**Evidence chain:**

* `src/balance_bot/behavior/states.py` lines 193-206 (`KickupState._incremental_kickup`):
  `context.core.hw.set_motors(drive_val, drive_val)` (pulse ON) immediately followed by
  `self._sleep_with_update(context, 0.25)` (intended 0.25 s pulse window).
* `states.py` lines 90 + 112-120: `_zero_motion_enabled` is
  `MotionRequest(velocity=0.0, turn_rate=0.0, enable_control=False)` and `_sleep_with_update`
  loops `context.core.update(self._zero_motion_enabled, ...)` every 10 ms.
* `src/balance_bot/reflex/balance_core.py` lines 149-163 (idle branch): when
  `motion.enable_control` is false the core **calls `self.hw.stop()` unconditionally, every tick**.

**Consequence:** On the first loop iteration after the pulse write, `core.update` writes zero.
The motor pulse lasts at most one loop period (`loop_time = 0.01 s`, configuration.py line 164)
plus I2C latency - not the intended 250 ms. A ~10 ms torque pulse cannot traverse LEGO gear
backlash, let alone rotate the chassis ~70 degrees from bumper rest to vertical. **This is the
leading hypothesis for the operator-confirmed failure mode.**

**Falsifiable prediction (verify in Phase 0 before touching anything):** in `flight_data.csv`,
each kick-up attempt should show non-zero `left_pwm`/`right_pwm` for only 1-2 consecutive rows.
If pulses actually last many rows, F0 is wrong; Phase 2 proceeds anyway (it removes the mechanism
entirely), but energy-delivery assumptions must be revisited.

### F1 - The kick-up sequence has no closed-loop timing

Independent of F0's duration bug, `_incremental_kickup` (states.py lines 173-221) fires a blind
fixed-duration pulse and only afterwards enters `_attempt_catch`. The robot never observes when
it crosses toward vertical:

* If the pulse carries the chassis past vertical sooner than assumed, gravity already pulls it
  over the wrong way before catch engages.
* If too weak, we wait out a full >= 1.25 s cycle (1.0 s settle + 0.25 s pulse + up to 2.5 s
  catch) before trying a bigger amplitude.
* With `kickup_power_forward/backward = 0.0` defaults (configuration.py lines 112-113), the
  search starts at zero power stepping 2% per cycle - tens of cycles, i.e., minutes, while the
  global `--experiment-duration` budget cuts runs short.

AGENTS.md Appendix A prescribes the fix ("Incremental Rock-and-Check Flip" with gyro feedback);
it was never implemented. Phase 2 implements it.

### F2 - Pitch estimate is corrupted exactly during kick-up/catch dynamics

`ComplementaryFilter` (utils.py lines 59-88) blends 2% raw accelerometer angle every tick
(alpha = 0.98, HardwareConfig line 165). During rotation, the MPU-6050 sits atop the tower
(r ~= 12-15 cm above the axle; AGENTS.md Rule 5), so tangential/centripetal acceleration dominates
the accelerometer and injects garbage into pitch precisely when closed-loop flip detection needs
it most. Additionally the filter initializes at `angle = 0.0` (utils.py line 75) while the robot
rests near -20 degrees on its bumpers; the 2 s warm-up (`setup_wait`) spends its early phase
recovering from a wrong prior instead of verifying calibration.

### F3 - The D-term is effectively absent (deferred - Phase 3a)

With degree units, `Kd = 0.5` contributes 30 PWM at a violent 60 deg/s fall rate while
`Kp = 10` contributes the same from only 3 degrees of tilt. An inverted pendulum is stabilized by
its derivative term; at these ratios the loop is nearly pure P, which on a backlash-y LEGO
drivetrain produces slam-oscillate-slam (JOURNAL 2026-08-02). Documented now so nobody re-litigates
it; addressed only after the kick-up gate passes.

### F4 - The auto-tuner cannot detect physical oscillation (deferred - Phase 3c)

`ContinuousTuner` analyzes a 1 s error buffer counting zero crossings against a 15% threshold
(configuration.py lines 65-66). A 1-2 Hz oscillation yields ~2-4 crossings per window - far below
threshold - so the "reduce Kp / boost Kd" heuristic rarely fires when needed. Kd nudges of +0.05
per event (50-tick cooldown) take many minutes of crashing to matter. Conclusion: **the tuner is a
trim tool, not a tuning tool**, and must not be relied on to rescue bad initial gains.

### F5 - No horizontal velocity handling (deferred - Phase 3b)

No encoders exist (PiconZero provides none), so once balanced the robot accelerates horizontally
until a wall or fall ends the run. Current mitigation (balance-point trim in
`BalancingState.update`, states.py lines 311-318) adjusts ~0.1 deg/s - orders of magnitude too slow
against runaway. Deferred until balancing itself works.

---

## 2. Strategy Overview & Phase Ordering

| Phase | Target | Changes behavior? | Gate to proceed |
| :--- | :--- | :--- | :--- |
| 0 | Blackbox forensics script | No (read-only tooling) | Operator confirms failure-mode data |
| 1 | Filter init + gyro-dominant blending | Yes (estimation only) | Lint/tests green; optional bench trial |
| 2 | Rock-and-Flip kick-up | Yes (state machine) | Kick-up success rate materially improves in trials |
| 3a | Gain restructuring (sim-first) | Yes (Tier 2 params) | Sim evidence; then hardware ladder |
| 3b | Velocity damping term | Yes (Tier 1 addition) | Baseline balance achieved |
| 3c | Auto-tuner demotion | Config only | Any time after 3a baseline |

**Ordering rationale:** the operator confirmed kick-up is the dominant failure, so everything
downstream has never been reached. Phases 0-2 attack exactly that gate. Phases 1 precedes 2
because closed-loop flip detection (Phase 2) depends on trustworthy pitch during motion (F2).
Gains are deliberately NOT touched until the robot reliably reaches vertical - changing two
variables at once destroys attribution, violating the incrementalism rule.

---

## 3. Phase 0 Spec - Blackbox Forensics

**Goal:** Confirm or falsify F0 and classify historical kick-up failures from existing telemetry,
with zero behavior change.

**Deliverable:** `tools/analyze_blackbox.py` (new directory; stdlib only - `csv`, `statistics`,
`argparse`. Note: `make lint` runs ruff on `src tests` only, but keep the script ruff-clean
anyway; mypy runs on `src` only).

**Input format** (from `src/balance_bot/telemetry.py` lines 39-50, file `flight_data.csv`,
one row per ~10 ms tick):

```
timestamp, state, pitch_angle, pitch_rate, yaw_rate, left_pwm, right_pwm, pid_target
```

**Required behavior:**

1. Segment rows by the `state` column transitions (`IdleState`, `KickupState`, `BalancingState`,
   `CrashedState`, `FatalErrorState`). Each contiguous `KickupState` block is one attempt session.
2. Within each session, detect pulses as maximal runs of rows where
   `abs(left_pwm) > 1.0 or abs(right_pwm) > 1.0`.
3. For every pulse report:
   * pulse width in ms (rows x ~10 ms) and peak PWM -> **directly verifies/falsifies F0**
     (prediction: width <= 20 ms);
   * max `abs(pitch_angle)` reached within 1.5 s after pulse onset ("how close to vertical");
   * min `abs(pitch_angle)` over the session (distance to vertical achieved);
   * whether `abs(pitch_angle) < 15.0` was ever reached (crossover zone entry).
4. Session summary: number of pulses, amplitude progression, outcome
   (entered BalancingState / crashed / budget-expired), time consumed.
5. Aggregate: counts per failure class -
   `NO_MOVEMENT` (peak |pitch| change < 3 deg after pulses),
   `INSUFFICIENT_ENERGY` (moved but never < 30 deg from vertical),
   `WRONG_TIMING` (crossed into zone but no BalancingState entry),
   `CAUGHT_THEN_LOST` (reached BalancingState then CrashedState).
6. Output a Markdown table to stdout plus an optional `--csv out.csv` summary.

**Acceptance:** runs on the existing `flight_data.csv` without error; results are reviewed with
the operator before Phase 2 parameters are chosen.

---

## 4. Phase 1 Spec - State Estimation Fixes

Two small changes confined to estimation. No gains, no state machine edits.

### 4a. Seed the filter from the actual rest angle

* **File:** `src/balance_bot/reflex/balance_core.py`
* **Change:** add method `seed_pitch_filter(self, samples: int = 100) -> None`: loop
  `self.hw.read_imu_converted()` `samples` times (no actuation), set
  `self.filter.angle = mean(reading.pitch_angle for readings)`.
* **Call site:** top of `Agent.run()` in `src/balance_bot/behavior/agent.py`, immediately BEFORE
  the warm-up while-loop (agent.py lines ~136-145). The robot must be at rest when this runs;
  log the seeded angle.
* **Why:** eliminates the wrong-prior transient so warm-up actually conditions the filter instead
  of slewing ~20 degrees.

### 4b. Gyro-dominant blending during high angular rate

* **File:** `src/balance_bot/utils.py` (`ComplementaryFilter.update`)
* **Signature change (backward compatible):**

  ```python
  def update(self, new_angle: float, rate: float, loop_delta_time: float,
             accel_norm_g: float | None = None) -> float:
  ```

* **Logic:** compute `alpha_eff`; start from `self.alpha`; if `accel_norm_g is not None` and
  `abs(rate) > self.rate_trust_limit`, use `self.gyro_only_alpha` (see config below).
  Then blend as today with `alpha_eff`.
* **Call site:** `balance_core.py` line ~147 continues to pass `(acc_angle, gyro_rate, dt)`.
  The accel-norm argument is accepted but NOT yet passed by Tier 1 (see tradeoff T4); the
  rate gate alone is active.
* **Config:** add to `HardwareConfig` (configuration.py, frozen model - additive fields are fine):

  ```python
  gyro_rate_distrust_limit: float = 60.0   # deg/s above which accel blending pauses
  gyro_only_alpha: float = 0.999           # near-pure gyro integration weight
  ```

  These are algorithm thresholds (not physical constants of a specific assembly), consistent
  with the Tabula Rasa rule; they ship as conservative defaults and persist via
  `hardware_config.json`.

* **Tests to update/add:** `tests/test_utils.py` (existing `ComplementaryFilter` coverage):
  new cases proving (i) below the limit behavior is bit-identical to today, (ii) above the limit
  the accelerometer term is suppressed, (iii) default construction keeps old signature working.
  Run `make lint && make test`.

---

## 5. Phase 2 Spec - Rock-and-Flip Kick-Up (The Main Event)

**Goal:** Replace the blind pulse + delayed catch with a closed-loop rock-and-flip that detects
vertical crossing in real time and engages catch control synchronously.

### 5a. New core primitive: `BalanceCore.pulse()`

* **File:** `src/balance_bot/reflex/balance_core.py`
* **Add method:**

  ```python
  def pulse(self, left_pwm: float, right_pwm: float, loop_delta_time: float) -> BalanceTelemetry:
      """One tick of open-loop actuation with full state estimation (no PID)."""
  ```

* **Behavior:** read IMU, update filter (exactly like `update()` steps 1-2), write motors via
  `self.hw.set_motors(clamped values)` (reuse the same clamp bounds as lines 247-249), populate
  and return `self.current_telemetry` with `motor_output=0.0`, `crashed=False`.
* **Safety inherited for free:** `hw.set_motors` already enforces deadman disarm
  (robot_hardware.py lines 495-498) and channel mapping fail-loud. The caller must still honor
  crash-angle checks and watchdog heartbeats.
* **Why this exists (T1):** pulses MUST NOT route through `update(enable_control=False)` - that is
  precisely the F0 bug (`stop()` every idle tick) - and MUST NOT run through PID (the controller
  would fight the pulse).

### 5b. New config fields

* **File:** `src/balance_bot/configuration.py`, class `ControlConfig` (lines 104-118). Add:

  ```python
  rock_amplitude_step: float = 2.0        # PWM % added per failed rock
  rock_pulse_max_duration: float = 0.40   # s hard cap per pulse
  rock_max_pulses: int = 6                # per KickupState entry (bounded search)
  crossover_zone_deg: float = 15.0        # |pitch| entering zone triggers flip check
  min_carryover_rate: float = 40.0        # deg/s needed to commit to the flip
  rest_settle_rate: float = 5.0           # deg/s below which robot counts as settled
  ```

* Start amplitude remains `kickup_power_forward/backward` if > 0, else
  `learning_state.min_power_visible + 10.0` (discovered floor plus margin; never hardcoded).
* Update `tests/test_kickup_config.py` to cover the new fields' defaults and persistence round-trip.

### 5c. Rewrite `KickupState` internals

* **File:** `src/balance_bot/behavior/states.py`
* **Remove:** `_incremental_kickup`'s blind pulse + post-hoc `_attempt_catch` sequencing.
  Keep `_attempt_catch` (its conservative gains at lines 132-142 are correct) but call it from
  the flip trigger instead of after a fixed sleep.
* **Keep:** public interface (`update(...)` signature), attempts accounting, transitions to
  `BalancingState`/`IdleState(attempts+1)`, watchdog heartbeat on EVERY tick.

**Algorithm (one KickupState entry):**

```
settle(): loop core.update(zero motion) until |pitch_rate| < rest_settle_rate
          for >= 0.5 s AND pitch within rest band; timeout 3 s; heartbeat each tick.

amplitude = start_amplitude(context)
for pulse_index in range(rock_max_pulses):
    direction = Direction.FORWARD if core.pitch < 0 else Direction.BACKWARD  # toward vertical
    drive = amplitude * float(direction.value)          # IntEnum arithmetic is safe
    rate = RateLimiter(1 / config.loop_time)
    pulse_deadline = now + rock_pulse_max_duration
    while now < pulse_deadline:
        watchdog.heartbeat()
        telem = core.pulse(drive, drive, dt)
        # Flip trigger: entered crossover zone WITH enough energy to carry over
        if abs(telem.pitch_angle) < crossover_zone_deg and \
           sign(telem.pitch_rate) matches rotation direction and \
           abs(telem.pitch_rate) >= min_carryover_rate:
            stop motors; return _attempt_catch(context, pid.target_angle)
        dt = rate.sleep()
    core.hw.stop()
    settle()
    # Failed rock: robot back on bumper? then grow amplitude; if it never moved,
    # also grow (same effect); abort only on rock_max_pulses exhaustion.
    amplitude += rock_amplitude_step
return False
```

**Critical correctness rules for the implementer:**

1. NEVER call `core.update(..., enable_control=False)` between `set_motors(pulse)` and the next
   intended pulse tick - that path stops the motors (this exact mechanism is bug F0).
   All pulse ticks go through `core.pulse()`.
2. The flip trigger must be evaluated on the SAME tick's telemetry it actuates after - no sleeps
   between detection and `_attempt_catch`. Catch latency budget: <= 1 loop tick (10 ms).
3. Every loop iteration must call `context.watchdog.heartbeat()` when a watchdog exists, matching
   existing patterns (states.py lines 101-102, 148-149, 195-196). The global
   `--experiment-duration` cutoff remains the outer safety net (AGENTS.md Rule 4).
4. Do not touch `IdleState` thresholds or attempt counting (3 attempts) in this phase;
   attribution requires one changed variable.
5. Type safety per `.jules/agent_instructions.md`: verify signatures before use; access enum
   numeric values explicitly (`float(direction.value)`); no unused imports after refactor
   (`_wait_for_settle` may become dead code - remove it and any now-unused imports).

### 5d. Tests (new file `tests/test_rock_flip_kickup.py`)

Model tests on the mocking style of `tests/test_agent_state_machine.py`. Build a FakeCore whose
`pulse()` advances a scripted physics trajectory (rest at -20 deg; given enough cumulative
pulse-ticks at amplitude A, pitch rises through the zone with rising rate; otherwise returns to
rest). Assert:

1. Pulse width spans MULTIPLE ticks before any stop (regression test for F0).
2. Flip trigger fires catch synchronously (no multi-hundred-ms gap) once trajectory enters zone.
3. Amplitude grows by `rock_amplitude_step` after each failed rock; bounded by `rock_max_pulses`.
4. Watchdog heartbeat called every tick; transition to BalancingState on catch success;
   IdleState(attempts+1) on failure.
5. `make lint && make test` green.

---

## 6. Phase 3+ Specs - Deferred Work

Do NOT start these until Phase 2 passes its hardware gate (kick-up success rate materially
improved across multiple trials, logged in JOURNAL.md).

### 3a. Gain restructuring (sim-first)

The pybullet rig (`src/balance_bot/simulation/sim_env.py`) already has torque control, domain
randomization (torque mods, IMU offset/rotation, jitter) and a 200 Hz physics / 100 Hz control
cadence - but nothing drives it with our controller (`sim_test.py` sends zeros). Build:

* `tools/sim_gain_sweep.py`: loads `PIDController` + the same error/D-term math used by
  `BalanceCore`, runs episodes over a gain grid (Kp in {8, 10, 14, 20}, Kd in {0.5, 2, 5, 10, 20},
  Ki = 0), emulating the discovered `min_power_visible` deadband on actuator output.
* Metrics per candidate: median episode survival time, RMS pitch, horizontal travel.
* Bring only the best 2-3 sets to hardware: one variable changed per trial, conservative baseline
  (`Kp=10, Ki=0, Kd=0.5`) as fallback between trials, deadman + experiment budget always on.
* Expected outcome (hypothesis to verify, not assume): final Kd well above 0.5 (see F3).

### 3b. Velocity damping term

Add a bounded damping contribution derived from high-pass-filtered forward acceleration
(pseudo-velocity; encoders are not available). Constraints:

* It is FEEDBACK DAMPING on estimated velocity - the target angle stays fixed at calibrated
  equilibrium. This does NOT violate the tilt-chasing prohibition (AGENTS.md Rule 3 /
  JOURNAL 2026-08-02): we never move the setpoint toward measured state.
* Authority clamp (max PWM contribution ~15) so sensor drift can never dominate.
* Tier 1 implementation must remain allocation-free and deterministic.

### 3c. Auto-tuner demotion

Until 3a's baseline is physically stable, set tuner aggression effectively to zero via config
(`start_aggression_normal` -> ~0 or a new enable flag). Rationale: F4 shows it cannot see the
oscillations that matter, and simultaneous tuner drift destroys attribution of hardware trials.
Reintroduce later strictly as a trim layer. Optional future: proper Astrom-Hagglund relay
autotune offline.

---

## 7. Decision & Tradeoff Log

| # | Decision | Alternatives rejected | Reasoning |
| :- | :--- | :--- | :--- |
| T1 | New `BalanceCore.pulse()` for open-loop ticks | Reuse `update()` with a new flag | `update(enable_control=False)` stops motors every tick - the exact F0 mechanism; adding flags there risks re-introducing it. A separate primitive makes "estimate + raw drive" explicit and keeps PID out of pulses. |
| T2 | Synchronous catch inside the pulse loop | Transition to a CatchState through the agent loop | State transition costs >= 1 agent-loop round trip plus Python overhead; catch latency budget is <= 1 tick (10 ms). `_attempt_catch` already runs its own tight loop - reuse it in place. |
| T3 | Incremental amplitude search retained | Single learned kick power persisted to config | Bumper geometry and floor friction differ per assembly/session; Tabula Rasa philosophy forbids trusting one learned constant. Search is bounded (6 pulses), so worst case is still time-limited. |
| T4 | Rate-gated gyro-dominant alpha only; accel-norm gate accepted but unwired | Full dynamic-alpha using accel magnitude deviation from 1 g | Unit ambiguity: real MPU-6050 library returns g, mocks/sim may differ; wiring an unscaled gate could permanently distrust the accelerometer. Defer until REPL empirically confirms each source's scale. |
| T5 | Stdlib-only analysis script in `tools/` | pandas/numpy notebook; package module | Zero new dependencies for a forensic tool; repo has numpy only under sim extras. |
| T6 | No encoder-based velocity feedback planned | Add encoders / different motor driver | Hardware constraint: PiconZero has no encoder inputs; AGENTS.md forbids assuming hardware swaps. Accel-derived pseudo-velocity is the only software path. |
| T7 | Freeze auto-tuner during kick-up trials | Let it co-adapt | Attribution: if gains drift while we change kick-up logic, trial results become uninterpretable; also F4 shows its nudges are noise at these timescales. |
| T8 | Gains untouched until kick-up gate passes | Fix F3 simultaneously | One changed variable per phase; historical sessions show multi-variable changes cannot be attributed (AGENTS.md incrementalism rule). |

---

## 8. Validation Checklist & Journal Protocol

For EVERY implemented phase:

1. `make lint` (ruff + mypy) and `make test` green before any hardware exposure.
2. Mock-mode smoke run: `uv run balance-bot --allow-mocks`.
3. Physical trials ONLY with `--deadman` engaged AND/OR `--experiment-duration` bounded
   (prefer both; AGENTS.md living-room safety rule).
4. Append a JOURNAL.md entry per trial using the template (Date, Hypothesis, Experiment,
   Observations, Learning, Action).
5. Run `tools/analyze_blackbox.py` after each trial; present the failure-class breakdown to the
   operator and CONFIRM interpretation before choosing next parameters.
6. Revert rule: any change that does not measurably improve its gate metric gets reverted before
   the next experiment (conservative-baseline discipline).
7. Never claim "this is the final fix" - record what was learned and what remains uncertain.

**Phase gates recap:** P0 -> operator-confirmed data picture; P1 -> estimation tests green;
P2 -> kick-up success observed on hardware; P3a -> sim-selected gains survive hardware ladder;
P3b/P3c -> only after stable baseline balancing exists.

