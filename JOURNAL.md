# JOURNAL.md - Chronological Engineering & Experiment Log

> **Core Directive:** This journal is the single source of truth for all historical experiments, empirical hardware audits, bug investigations, and architectural lessons learned while developing the self-balancing robot.
> 
> **Mandatory Journaling Protocol:** Every time an agent attempts a physical calibration, modifies control loop dynamics, tests hardware drivers, or investigates a bug, you **MUST** append a new entry to this file documenting what was tried, what empirical evidence was observed, and what was learned. See [AGENTS.md Section 3](file:///Users/benhill/Desktop/hobbies/balance_bot/AGENTS.md#3-mandatory-journaling-protocol-journalmd).

## Table of Contents
1. [Purpose & Journaling Protocol](#purpose--journaling-protocol)
2. [Entry Template](#entry-template)
3. [Chronological Experiment & Audit Log](#chronological-experiment--audit-log)
   * [2025-02-18 - Refactoring Hardware Drivers with Global State](#2025-02-18---refactoring-hardware-drivers-with-global-state)
   * [2025-02-18 - Hardware Retry Logic Simplification & DRY](#2025-02-18---hardware-retry-logic-simplification--dry)
   * [2025-02-18 - Pruning Dead Vendor Code in src/](#2025-02-18---pruning-dead-vendor-code-in-src)
   * [2026-08-01 - Root Cause of Missing Attribute Error in icontract Decorators](#2026-08-01---root-cause-of-missing-attribute-error-in-icontract-decorators)
   * [2026-08-02 - The PiconZero I2C Register Retention Trap & Motor Runaway](#2026-08-02---the-piconzero-i2c-register-retention-trap--motor-runaway)
   * [2026-08-02 - LEGO Mechanical Backlash vs High Proportional Gain (Kp)](#2026-08-02---lego-mechanical-backlash-vs-high-proportional-gain-kp)
   * [2026-08-02 - The Tilt-Chasing Resonance Loop in Soft Recovery](#2026-08-02---the-tilt-chasing-resonance-loop-in-soft-recovery)
   * [2026-08-02 - Global vs. Local Safety Cutoffs (The 4-Second Chaos Rule)](#2026-08-02---global-vs-local-safety-cutoffs-the-4-second-chaos-rule)
   * [2026-08-02 - RateLimiter Busy-Wait vs time.sleep() Flakiness in CI](#2026-08-02---ratelimiter-busy-wait-vs-timesleep-flakiness-in-ci)
   * [2026-08-02 - Operational Guardrails & Next Horizon Hypotheses](#2026-08-02---operational-guardrails--next-horizon-hypotheses)

---

## Purpose & Journaling Protocol

In cyber-physical systems, code elegance is secondary to physical friction resolution. AI agents operating across distinct conversations lack memory of physical failures unless those outcomes are explicitly logged. By maintaining a structured, chronological journal of every experiment, we prevent repeating mistakes and build an empirical knowledge base for getting the robot balancing stably.

---

## Entry Template

When appending new entries to this file, use the following Markdown template:

```markdown
### YYYY-MM-DD - [Short Descriptive Title]
* **Hypothesis / Goal:** What we were trying to achieve or test.
* **Experiment / What We Tried:** The command, code change, or physical test executed.
* **Observations / Empirical Evidence:** What actually happened in hardware or logs.
* **Learning / Root Cause:** Why it happened and what principle was revealed.
* **Action / Rules Updated:** What changes were made to code, defaults, or AGENTS.md.
```

---

## Chronological Experiment & Audit Log

### 2025-02-18 - Refactoring Hardware Drivers with Global State
* **Hypothesis / Goal:** Refactor hardware driver modules (`piconzero.py`) while preserving testability in unit tests.
* **Experiment / What We Tried:** Unit testing modules that rely on module-level global state (`bus`, `DEBUG`, `RETRIES`) imported across multiple files.
* **Observations / Empirical Evidence:** Standard `unittest.mock.patch` failed when global state was initialized at import time; mocks did not cleanly isolate tests across test suites.
* **Learning / Root Cause:** When refactoring a module that relies heavily on global state and is imported by other modules, testing becomes tricky because the module is not fully reloaded between tests.
* **Action / Rules Updated:** When testing hardware drivers with global initialization, explicitly reload the module in `setUp` using `importlib.reload(module)` to ensure a clean state when the module initializes resources (`smbus.SMBus`) at the top level.

---

### 2025-02-18 - Hardware Retry Logic Simplification & DRY
* **Hypothesis / Goal:** Reduce repetitive error handling across hardware interface calls.
* **Experiment / What We Tried:** Audited error handling across `PiconZero` and `RobotHardware` methods.
* **Observations / Empirical Evidence:** Hardware drivers repeated try-except-retry error handling logic across dozens of methods, accounting for ~50% of boilerplate code.
* **Learning / Root Cause:** Repeating exception blocks around low-level I2C writes clutters the driver and leads to inconsistent error reporting.
* **Action / Rules Updated:** Refactored repeated `try...except` blocks in hardware interface code into a shared helper function `_retry(func, name)` early, halving code size while ensuring consistent retry behavior.

---

### 2025-02-18 - Pruning Dead Vendor Code in src/
* **Hypothesis / Goal:** Keep vendor-supplied driver files (`src/balance_bot/hardware/piconzero.py`) lean and maintainable.
* **Experiment / What We Tried:** Reviewed vendor files for unused methods and helper functions.
* **Observations / Empirical Evidence:** Vendor files contained extensive helper functions for lights, sensors, and peripherals not used by the balancing robot.
* **Learning / Root Cause:** Unused vendor code in `src/` increases maintenance burden, pollutes test coverage metrics, and confuses static analysis tools.
* **Action / Rules Updated:** Always check vendor files in `src/` for unused code and aggressively prune them if they are not maintained as an external dependency. Only keep motor control logic.

---

### 2026-08-01 - Root Cause of Missing Attribute Error in icontract Decorators
* **Hypothesis / Goal:** Investigate runtime error `AttributeError: 'MeasureResult' object has no attribute 'status'` during `repl.py` gyro testing.
* **Experiment / What We Tried:** Analyzed `@icontract.ensure(lambda result: result.status != 'unknown', ...)` on `drive_and_measure` in `robot_hardware.py` and `sim_hardware.py`.
* **Observations / Empirical Evidence:**
  1. `icontract` decorators accept lambda functions without generic type parameter bindings (`Callable[[T], bool]`). `mypy` treated the lambda argument `result` as `Any` and silently permitted `.status` access.
  2. Zero unit tests in `tests/` executed `drive_and_measure` or `test_gyro`, leaving the runtime contract unverified until manual REPL usage.
  3. `mypy` flagged `icontract`-decorated functions as untyped unless `--disallow-untyped-decorators` was enforced.
* **Learning / Root Cause:** Pure runtime contract decorators like `icontract` can hide static type errors inside lambdas. Without test coverage executing decorated functions, contract bugs slip through CI.
* **Action / Rules Updated:**
  * Every function decorated with `icontract` MUST have unit test coverage executing its happy path.
  * Prefer explicit helper validator functions (`def check_status(result: MeasureResult) -> bool: ...`) or Pydantic `@validate_call` where static type safety is preserved.

---

### 2026-08-02 - The PiconZero I2C Register Retention Trap & Motor Runaway
* **Hypothesis / Goal:** Ensure safe motor shutdown when the Python controller process is terminated or crashes.
* **Experiment / What We Tried:** Terminated the control process via `SIGTERM`, `SIGINT`, or `kill -9` while motors were actively driving.
* **Observations / Empirical Evidence:** The 4tronix PiconZero HAT uses an onboard ATmega328P microcontroller that holds its PWM output registers independently of the host Raspberry Pi CPU. When the host process terminated, the HAT continued driving motors indefinitely at the last commanded speed. Writing `0` to registers 0 and 1 over I2C was sometimes insufficient to override an active runaway state.
* **Learning / Root Cause:** Motor HAT microcontrollers decouple host CPU state from physical actuator state. Without an explicit hardware microcontroller reset, output pins remain energized.
* **Action / Rules Updated:** All hardware abstraction layers (`RobotHardware`) must register synchronous OS signal handlers (`SIGTERM` / `SIGINT`) and object destructors (`__del__`) that write both zero-power commands and the full board reset command (`CMD_RESET = 20` written to address `0x22`) before process termination.

---

### 2026-08-02 - LEGO Mechanical Backlash vs High Proportional Gain (Kp)
* **Hypothesis / Goal:** Test balancing responsiveness using standard aggressive proportional control (`Kp = 25.0`).
* **Experiment / What We Tried:** Ran inverted pendulum balance loop with default initial gain `Kp = 25.0`.
* **Observations / Empirical Evidence:** In a 3D-printed/LEGO Segway topology, mechanical gear backlash creates a physical deadband where the motor shaft rotates before teeth engage the wheel. At `Kp = 25.0`, a tiny 4° tilt error clamped PWM to 100%. The motor accelerated across the backlash gap, slammed into gear teeth, overshot the balance point, and caused violent oscillation.
* **Learning / Root Cause:** Stiff proportional control loops are incompatible with mechanical drivetrain backlash; high gains amplify deadband slam rather than correcting tilt.
* **Action / Rules Updated:** Never start balancing experiments at `Kp = 25.0`. Always begin at a conservative baseline (`Kp = 10.0`, `Ki = 0.0`, `Kd = 0.5`) that nudges wheels without slamming across backlash gaps.

---

### 2026-08-02 - The Tilt-Chasing Resonance Loop in Soft Recovery
* **Hypothesis / Goal:** Implement soft recovery by dynamically setting `target_angle_offset` to equal `current_pitch` when recovering from tilt error (`abs(pitch) > 5.0°`).
* **Experiment / What We Tried:** Dynamically updated `target_angle_offset` during active balancing in `BalancingState`.
* **Observations / Empirical Evidence:** Overriding the PID setpoint to match current pitch converted an inverted pendulum controller into an open-loop horizontal accelerator. Because maintaining non-zero tilt requires continuous horizontal acceleration, setting the target angle to match current wobble mathematically ordered the robot to accelerate across the floor at full speed ("jetting across the room"). As it bounced off walls, the target angle alternated signs, causing violent flailing.
* **Learning / Root Cause:** Setpoint tracking during active balance feedback loops creates positive acceleration resonance.
* **Action / Rules Updated:** Never override `target_angle_offset` during active balancing (`BalancingState`). Soft recovery ramps may only be applied once during wake-up from stationary rest; once in active balance, target angle must remain fixed at calibrated vertical equilibrium (`0.0°` plus trim).

---

### 2026-08-02 - Global vs. Local Safety Cutoffs (The 4-Second Chaos Rule)
* **Hypothesis / Goal:** Prevent robot from running open-loop or retrying indefinitely when calibration or kick-up fails.
* **Experiment / What We Tried:** Placed safety timeout logic inside individual state classes (`BalancingState`).
* **Observations / Empirical Evidence:** When the robot got trapped in earlier phases (e.g., `ProprioceptiveToddlerStep` discovery or `KickupState`), local state timeouts never fired, allowing the robot to crash into walls.
* **Learning / Root Cause:** Local state machine timeouts are bypassed when state transitions hang or loop in earlier calibration phases.
* **Action / Rules Updated:** Hard-cutoff timers (such as a 4.0-second maximum experiment limit) must be enforced at the root application/watchdog level (`main.py` / `SurvivalWatchdog`), independent of state-machine transitions, ensuring unconditional termination and hardware reset.

---

### 2026-08-02 - RateLimiter Busy-Wait vs time.sleep() Flakiness in CI
* **Hypothesis / Goal:** Validate 100Hz (`10ms` loop time) timing precision in unit tests and runtime.
* **Experiment / What We Tried:** Tested timing using standard Python `time.sleep()`.
* **Observations / Empirical Evidence:** `time.sleep()` only guarantees minimum sleep duration, not maximum. In CI environments or under CPU load (`pytest -n auto`), OS scheduler jitter caused `time.sleep()` to overshoot significantly, breaking 100Hz reflex determinism and flaking unit tests.
* **Learning / Root Cause:** Standard OS sleep functions are non-deterministic under load and cannot guarantee sub-millisecond precision required for inverted pendulum reflex loops.
* **Action / Rules Updated:** Use deterministic busy-waiting (`RateLimiter.sleep`) for real-time 100Hz execution. In unit tests, use mocked time (`patch("time.perf_counter")`) to deterministically verify rate limiter logic without flaky real-time dependencies.

---

### 2026-08-02 - Operational Guardrails & Next Horizon Hypotheses
* **Hypothesis / Goal:** Establish future safety guardrails and testable hypotheses as we advance toward robust balancing.
* **Experiment / What We Tried:** Formulated operational guardrails from empirical failure modes.
* **Observations / Empirical Evidence:**
  1. **Process Priority / Telemetry Server:** High-frequency Tier 1 loops can be starved by background tasks. Auxiliary servers must run with nice/ionice so they never preempt control:
     ```bash
     nohup nice -n 19 ionice -c 3 python3 -q -O -m http.server 8000
     ```
  2. **Gyro Failure Watchdog:** $N$ successive I2C gyro read failures indicates sensor bus lockup or disconnect $\rightarrow$ immediately halt all motors and stop.
  3. **Fall/Stall Detection:** $N$ successive motor move commands without any corresponding change in gyro rates indicates the robot has fallen over or stalled against an obstacle $\rightarrow$ immediately halt all motors and stop.
* **Learning / Root Cause:** Sensor disconnects and mechanical stalls require explicit watchdog detection independently of PID math.
* **Action / Rules Updated:** Prioritize implementing consecutive failure counters for gyro I2C reads and gyro-motor correlation checks in future `SurvivalWatchdog` enhancements.

---

### 2026-08-08 - Linting, Formatting, and Mac/Hardware-Agnostic Static Analysis
* **Hypothesis / Goal:** Establish robust, comprehensive linting (Ruff), code formatting, and strict type checking (MyPy) capable of running reliably in local macOS development environments without physical robot hardware or specialized device libraries attached.
* **Experiment / What We Tried:**
  1. Configured PyPI as the default `uv` package index (`[[tool.uv.index]] url = "https://pypi.org/simple"`) to ensure reproducible dependency syncing on macOS without failing on unauthenticated internal registries.
  2. Integrated `mypy>=1.14.0` alongside `ruff>=0.9.0` in `[dependency-groups] dev` within [`pyproject.toml`](file:///Users/benhill/Desktop/hobbies/balance_bot/pyproject.toml). Configured MyPy to ignore missing third-party hardware modules (`smbus2`, `mpu6050`, `pybullet`, `gymnasium`).
  3. Fixed missing `from typing import Any` and undefined names across `states.py`, `pipeline.py`, `balance_core.py`, and `watchdog.py`.
  4. Resolved real structural and protocol issues discovered by MyPy: added missing `is_verified()` method to `ProprioceptiveToddlerStep` (`CalibrationStep` protocol compliance), removed obsolete attribute access in `sim_hardware.py`, and ensured explicit type narrowing in `pid.py` and `robot_hardware.py`.
  5. Modernized deprecated `import glm` statements to `from pyglm import glm` across `src/` and `tests/`.
  6. Updated [`Makefile`](file:///Users/benhill/Desktop/hobbies/balance_bot/Makefile) with `lint`, `format`, `test`, `typecheck`, and `check` targets.
* **Observations / Empirical Evidence:** `make check` executes formatting verification (`ruff format --check`), linting (`ruff check`), static typing (`mypy src`), and the full unit test suite (`pytest tests/`), completing in ~4.5s with all 162 unit tests passing and 0 static analysis errors.
* **Action / Rules Updated:** Maintained `make lint`, `make format`, and `make check` workflows for local continuous verification.

---

### 2026-08-09: Cyber-Physical Bug Fixes & Control Loop Remediation
* **Hypothesis / Goal:** Resolve verified defects from [`POSSIBLE_ISSUES.md`](file:///Users/benhill/Desktop/hobbies/balance_bot/POSSIBLE_ISSUES.md): standardize deadman switch port, fix dead no-op catch checks and gain saturation during kick-up, correct inverted gyro pitch rate D-term sign in Tier 1 PID, apply stiction deadband compensation, and remove hardcoded balancing timeouts.
* **Experiment / What We Tried:**
  1. Standardized HTTP Deadman Switch port to 8090 across [`main.py`](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/main.py), [`deadman.py`](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/deadman.py), [`POSSIBLE_ISSUES.md`](file:///Users/benhill/Desktop/hobbies/balance_bot/POSSIBLE_ISSUES.md), and [`README.md`](file:///Users/benhill/Desktop/hobbies/balance_bot/README.md).
  2. Fixed dead `pass` statements in [`KickupState._attempt_catch`](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/behavior/states.py) to enable early success return upon vertical stability (`error < 5.0°` and low rate) and early abort on crash/overshoot (`error > 40.0°`).
  3. Clamped catch PID gains (`catch_kp = min(kp, 15.0)`, `ki = 0.0`, `catch_kd >= 0.5`) to eliminate LEGO gear backlash slamming and integral windup during kick-up transitions.
  4. Updated default `PIDParams.kp` from aggressive `25.0` to conservative baseline `10.0` in [`configuration.py`](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/configuration.py).
  5. Corrected gyro pitch rate D-term sign in [`PIDController.update`](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/reflex/pid.py) to `output += real_kd * measurement_rate` (preventing positive feedback damping inversion).
  6. Implemented actuator deadband feedforward compensation using `min_power_visible` in [`BalanceCore.update`](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/reflex/balance_core.py) to overcome static gear friction.
  7. Replaced hardcoded 4.0s cutoff in [`BalancingState`](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/behavior/states.py) with dynamic checks on `experiment_duration`.
* **Observations / Empirical Evidence:** Catch state now exits immediately when equilibrium is achieved or aborted on overshoot instead of spinning motors for 2.5s; gyro derivative acts in tandem with proportional drive rather than opposing it.
* **Learning / Root Cause:** Derivative on measurement in an inverted pendulum differs from standard setpoint regulation: positive angular velocity away from setpoint requires additive motor command in the direction of tilt to catch the center of gravity.
* **Action / Rules Updated:** Maintained conservative baseline gains and documented deadband feedforward scaling in Tier 1 reflex loop.
