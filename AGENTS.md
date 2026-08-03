# AGENTS.md - Self-Balancing Robot Engineering Manual & Governance

> **Core Directive:** This repository is built and maintained by autonomous AI agents. All Python code and documentation must be structured for deterministic execution, machine consumption, and physical safety in the real world.

## Table of Contents
1. [Project Philosophy & Hardware Reality](#1-project-philosophy--hardware-reality)
2. [Mandatory LLM Directives & Code Standards](#2-mandatory-llm-directives--code-standards)
3. [Mandatory Journaling Protocol (JOURNAL.md)](#3-mandatory-journaling-protocol-journalmd)
4. [System Architecture & Multi-Mind Topology](#4-system-architecture--multi-mind-topology)
5. [The Tabula Rasa Protocol (Zero-Knowledge Self-Discovery)](#5-the-tabula-rasa-protocol-zero-knowledge-self-discovery)
6. [Critical Cyber-Physical Blind Spots](#6-critical-cyber-physical-blind-spots)
7. [Immutable Empirical Hardware Rules](#7-immutable-empirical-hardware-rules)
8. [Operations, Systemd Deployment & Manual REPL](#8-operations-systemd-deployment--manual-repl)
9. [Codebase Assumptions & Reference Citations](#9-codebase-assumptions--reference-citations)

---

## 1. Project Philosophy & Hardware Reality

This repository operates a self-balancing homebrew robot with a Segway-like topology. The hardware is bottom-tier (3D-printed wheels, LEGOs, mismatched DC motors). Human intervention during operation is considered a failure of logic.

* **Training Wheels (Bumpers):** Stick out at ~20° from vertical. Resting on them is a valid state, not a crash. A crash is defined as face-planting > 50° from vertical. Transitioning from Resting to Balancing uses a "Kick-Up" maneuver.
* **Goals & Persistence:** The robot must boot via Tabula Rasa, improve over time, remember "last known good" configurations, and handle changing battery voltage levels. Upon successful self-discovery, it outputs a `pid_config.json` file to bypass calibration on future boots.
* **Living Room Safety & Experiment Bounds:** This is a real physical robot operating in a residential living room. Having it flail around uncontrollably and damage itself, people, or furniture is the worst possible outcome. All physical experiments and balancing trials **MUST be either time-bounded OR protected by a Deadman's Switch** (using both is strongly recommended).
* **Incrementalism Over Hyper-Optimism:** Never assume or claim that any single change is "the final fix that will solve it." History across 10+ sessions proves there are no silver bullets. Progress requires collecting empirical data, confirming data interpretation with the user, and making small, iterative improvements.


---

## 2. Mandatory LLM Directives & Code Standards

All AI agents working on this codebase must adhere to these non-negotiable governance rules:

| Category | Constraint |
| :--- | :--- |
| **Role & Constraints** | Follow the "Do No Harm" Rule. Do not alter runtime logic or physics constants without empirical verification. Leave hardware quirks alone. Only worry about motor control code in vendor files (`src/balance_bot/hardware/piconzero.py` has extra functions for lights/sensors; keep motor logic lean). |
| **Mandatory Guidelines** | **Before writing or modifying any code, you MUST read `.jules/agent_instructions.md`.** It contains critical instructions on verifying function signatures and type safety to prevent regressions. |
| **Mandatory Journaling** | **Every time you try something, you MUST append to `JOURNAL.md` what was tried, the empirical observations, and what was learned.** See [Section 3](#3-mandatory-journaling-protocol-journalmd). |
| **Code Modernization** | Use Python 3.14+ conventions, standard type hints, `min()/max()`, and f-strings. Use `balance_bot.utils.Vector3`. Keep Tier 1 control loop (100Hz) mathematically pure; isolate inversions in the HAL. Never optimize untestable hardware I/O. |
| **Testing & Mocks** | If testing without hardware (e.g., CI/laptop), use `uv run balance-bot --allow-mocks`. Without this flag, the code is configured to crash loudly if physical hardware is missing. |
| **Hardware Reality** | Never swallow errors (except in well-documented hardware retry loops). Explicitly handle I2C crashes (ramp power to prevent Errno 5 brownouts). No hallucinated physics (e.g., "momentum swings"). Operate under strict "Data Literalism". |
| **Bootstrapping** | Never hardcode physical constants. Remain pessimistic and verify every hardware assumption empirically. |
| **Physical Tuning** | Practice graceful degradation. Revert to conservative baseline (`Kp=10.0`, `Ki=0.0`, `Kd=0.5`) if unable to verify states with confidence. |
| **Permission & Network** | Always ask the user first before attempting network connections, SSH, or permission-sensitive operations. |
| **Evidence & Tone** | Never use speculative language or phrases like "smoking gun". Gather sufficient verifiable data to prove a theory rather than guessing. |
| **Scientific Method** | Fight the impulse to jump to an immediate solution. Make small, provable incremental improvements and empirically verify they improved behavior before moving to the next step. |

### Architectural Transformations for Autonomous Agents

1. **Architecture & Abstraction:** Favor Vertical Slice Architecture and Locality of Behavior over deeply layered DRY abstractions. Colocating dependencies for a single feature provides dense, relevant context and reduces unintended edit blast radius.
2. **Typing & Data Boundaries:** Replace implicit duck typing with exhaustive type hinting, strict Pydantic schemas (`HardwareConfig`, `LearningState`), and Design-by-Contract decorators (`icontract`). Note: Any function decorated with `icontract` MUST have unit test coverage executing its happy path to catch lambda type mismatches at runtime.
3. **Documentation & Context:** Use structured, machine-readable Context Headers (defining System Context, Business Rules, and Dependency Maps at the top of files) instead of verbose narrative docstrings. Inline comments should explain *why* code is architected a certain way to avoid architectural thrashing.
4. **Development Workflow & Verification:** Adopt Spec-Driven Development (SDD) with Verifiable Rewards over subjective human "Looks Good To Me" reviews. Enforce execution-based tests.
5. **Pre-Flight Signature & Enum Checklist:** Never guess function arguments or return types. Use `read_file` or `grep` to read definitions before calling functions or Pydantic models. For Enum types (`Direction`, `Axis`), explicitly access `.value` when performing arithmetic or comparisons.
6. **Clean Up After Refactoring:** Remove unused imports and local variables immediately. In benchmarking or timing scripts, assign side-effect instances to `_` (`_ = ClassName(...)`) to satisfy linters (`uv run ruff check .`).

### Architectural Anti-Pattern: Context-Blind Fault Intolerance
When generating control code for cyber-physical systems, AI agents frequently misapply high-level software engineering paradigms—specifically strict "fail-fast" exception handling—to low-level hardware interfaces. In a real-time physical environment, transient I/O errors (such as I2C glitches caused by motor EMI or voltage sags) are nominal operating conditions, not fatal system failures. Enforcing strict data-purity rules during a continuous actuation loop causes trivial hardware noise to collapse the entire system.
* **Rule:** Strictly separate initialization logic (where failing fast is required) from the real-time control loop (which requires absolute fault tolerance).
* **Rule:** Replace fatal exceptions in continuous loops with data quality metrics (e.g., returning cached data alongside an age/error counter) to keep the loop alive.

### Condensed LLM-Optimized Comments
Focus on maximizing information density while minimizing token consumption:
* **Acknowledge Intent, Not Logic:** Focus on *why* a block exists or its future goals rather than *what* syntax does.
* **Keyword-Heavy Phrases:** Use concise context ("// auth: check-session", "// find-max: data", "// fallback: null").

---

## 3. Mandatory Journaling Protocol (JOURNAL.md)

To maintain continuity across asynchronous agent sessions and ensure that we never lose the empirical lessons learned from physical hardware experiments, **all agents must log their experiments in `JOURNAL.md`.**

### When to Append to `JOURNAL.md`
You MUST append a new entry to `JOURNAL.md` every time you:
1. Attempt a physical calibration, kick-up test, or balancing trial.
2. Modify control loop dynamics, PID parameters, or hardware driver timing.
3. Investigate a bug, linter failure, or unexpected hardware behavior.
4. Test a new hypothesis about motor friction, battery sag, or sensor noise.

### What to Log
Every entry added to `JOURNAL.md` must follow this structure:
* **Date & Title:** Timestamp (`YYYY-MM-DD`) and concise experiment title.
* **Hypothesis / Goal:** What we were trying to achieve or test.
* **Experiment / What We Tried:** The command, code modification, or physical test executed.
* **Observations / Empirical Evidence:** What actually happened (e.g., physical oscillation, I2C reset failure, CI timing jitter).
* **Learning / Root Cause:** Why it happened and what physical or computational principle was revealed.
* **Action / Rules Updated:** How the codebase, conservative defaults, or `AGENTS.md` rules were updated to prevent future regressions.

---

## 4. System Architecture & Multi-Mind Topology

Robot control is divided into three "Minds" running at different frequencies and responsibilities (Subsumption Architecture):

```
+-------------------------------------------------------+
|                 Tier 3: The Cortex                    |
|       (Behavior, Scheduling, Intent ~ Event-Driven)   |
+-------------------------------------------------------+
                           |
                           v
+-------------------------------------------------------+
|               Tier 2: The Cerebellum                  |
|     (Adaptation, Tuning, Soft-Start ~ 10Hz Subsampled)|
+-------------------------------------------------------+
                           |
                           v
+-------------------------------------------------------+
|               Tier 1: The Brainstem                   |
|       (Reflex, 100Hz Pure PID Control & IMU Loop)     |
+-------------------------------------------------------+
```

1. **Tier 1: The Brainstem (Reflex) — `100Hz`**
   * **Goal:** Stay vertical.
   * **Responsibility:** Reads MPU-6050 IMU, executes PID control (`BalanceCore`), commands motors.
   * **Constraints:** Stateless, deterministic, safety-critical. **Zero garbage collection or async I/O permitted.**
2. **Tier 2: The Cerebellum (Adaptation) — `~10Hz` (Subsampled)**
   * **Goal:** Optimize mechanics and adapt to changing conditions.
   * **Responsibility:** Analyzes performance (oscillation, drift), tweaks PID gains (`ContinuousTuner`), finds balance equilibrium (`BalancePointFinder`), monitors battery sag (`BatteryEstimator`), and handles soft-start recovery (`RecoveryManager`).
3. **Tier 3: The Cortex (Behavior) — Low Frequency / Event-Driven**
   * **Goal:** High-level intent and orchestration.
   * **Responsibility:** Manages the agent lifecycle (`Agent`), status LEDs (`LEDController`), configuration persistence (`pid_config.json`), and safety watchdog (`SurvivalWatchdog`).

---

## 5. The Tabula Rasa Protocol (Zero-Knowledge Self-Discovery)

The robot boots with zero hardcoded assumptions about motor wiring, polarity, or MPU-6050 orientation. It deduces its configuration through pessimistic physics experiments:

* **Phase 1: Spark of Life (Presence):** Scan I2C for MPU-6050 (`0x68`) and Picon Zero (`0x22`). Halt on failure.
* **Phase 2: Sense of Down (Gravity):** User-assisted lean step. Robot records backwards lean, waits for forward flop, and infers vertical/forward axes without moving motors.
* **Phase 3/4: Stiction, Phasing, & Polarity:** Ramp PWM to find deadband (`min_power_visible`), explicitly tracking I2C glitches. Pulse motors to observe gyro around up-vector.
  * *Spinning Failure Mode:* If wheels slip causing pure Yaw instead of Pitch, rely on Gyro yaw vectors (`yaw_mag_sum` vs `yaw_mag_diff`), not linear acceleration.
  * *Forward Check:* Lean forward, pulse positive. If pitch decreases (stands up), Positive = Forward.
* **Phase 5: Absolute Identity (L/R):** Command `+PWM` on Port A, `-PWM` on Port B. By Right-Hand Rule, Gyro pointing Up = Counter-Clockwise (Left) spin $\rightarrow$ Port A = Right Motor.
* **Phase 6: Motor Trimming:** Drive forward open-loop, integrate Yaw, apply fractional multiplier to stronger motor.
* **Phase 7: True Balance Point (Effort Minimization):** Hover with `target_angle = 0.0°`. Monitor Integral term (average motor output).
  * If forward drift (nose heavy), decrease target angle (lean back).
  * If backward drift, increase target angle (lean forward).
  * Clamp to `+/- 10°`, learn slowly, save to `pid_config.json`.

### Appendix A: Future Architecture ("Proprioceptive Toddler")
A Dependency-Driven Discovery Engine replacing linear scripts with "Motor Babbling" to build a Knowledge Graph of atomic facts (`HardwareBus`, `GravityVector`, `ChassisHandedness`):
1. **Stage 1: Individual Motor Friction Search:** Ramps Left and Right motors independently until initial motion is detected (`min_L`, `min_R`). Sets baseline `min_power_visible = max(min_L, min_R)`.
2. **Stage 2: A/B Polarity Contrast:** Compares straight-line pulse `(L, R)` against spin-in-place `(L, -R)`. High yaw rate confirms turning/spin; low yaw rate confirms matched polarity.
3. **Stage 3: Incremental Rock-and-Check Flip:** Replaces hardcoded kick-up power with incremental forward-backward pulse search (`20% -> 22% -> 24%...`) to discover rear rest bumper (`+A`), front rest bumper (`-B`), and vertical peak (`0°`).

---

## 6. Critical Cyber-Physical Blind Spots

LLM agents inherently bias toward idealized software refactoring. Before proposing code modifications, double-check against these physical realities:

| Repository Component | Vulnerable Files & Functions | The LLM Blind Spot (Your Bias) | The Physical Rule (Your Constraint) |
| :--- | :--- | :--- | :--- |
| **Zero-Knowledge Bootstrapping** | `src/balance_bot/discovery/*.py`<br>`SelfDiscoveryPipeline.run` | You will view calibration routines as clunky, blocking state machines that could be simplified by hardcoding variables. | **Never hardcode physical constants.** This robot is assembled differently every time. Do not bypass discovery of motor mappings, `min_power_visible`, or IMU axes. |
| **Temporal Determinism (Brainstem)** | `src/balance_bot/reflex/balance_core.py`<br>`src/balance_bot/reflex/pid.py` | You will assume standard refactoring (adding matrix math libraries, standardizing logging, abstracting into classes) is inherently good. | **Strict 100Hz execution is mandatory.** Any microsecond of latency added to Tier 1 will cause a physical crash. Do not introduce GC-heavy operations or async calls here. |
| **Hardware Communication Nuances** | `src/balance_bot/hardware/piconzero.py`<br>`src/balance_bot/hardware/robot_hardware.py` | You will see standard I2C logic and assume generic libraries apply, or assume GPIO pins can be easily swapped. | **Hardware quirks cannot be abstracted.** The HAT blocks standard pins, requiring custom software-defined "bus 3" (pins 17/27, 100kHz). Driver requires *Byte Writes*, not Block Writes. |
| **Physical Tuning (Unverified Zone)** | `src/balance_bot/adaptation/tuner.py`<br>`src/balance_bot/adaptation/recovery.py` | You will rely on semantic log outputs and may misinterpret a safety shutdown as a stable state. | **Practice graceful degradation.** If unable to verify physical state, revert to conservative baseline (`Kp=10.0`, `Ki=0.0`, `Kd=0.5`). Avoid algorithmic slop. |
| **Battery/Power Dynamics** | `src/balance_bot/adaptation/battery.py`<br>`BatteryEstimator.update` | You will assume motors output consistent torque for a given PWM command and may try to simplify PWM compensation. | **Recognize power decay.** 50% PWM at 9V is physically different from 50% PWM at 7V. Battery estimator scaling is required to boost PWM dynamically as battery drains. |
| **System Lifecycle & Watchdog** | `src/balance_bot/watchdog.py`<br>`src/balance_bot/main.py`<br>`SurvivalWatchdog._watch` | You will view `SurvivalWatchdog` as redundant to standard `try/except` blocks or systemd restarts and attempt to deprecate it. | **Deadlocks are physical, not logical.** If the main thread hangs on a hardware read, exceptions will not fire. The Watchdog must run on a separate thread to force `KeyboardInterrupt` / exit. |
| **Sensor Noise & Filtering** | `src/balance_bot/utils.py`<br>`ComplementaryFilter.update`<br>`RateLimiter.sleep` | You will view `ComplementaryFilter` as primitive or identify busy-waiting in `RateLimiter` as poor CPU usage, proposing Kalman filters or `time.sleep()`. | **Determinism over complexity.** Advanced math or `time.sleep()` lack microsecond precision required for 100Hz loop. Keep filters cheap and the thread busy. |

---

## 7. Immutable Empirical Hardware Rules

These rules were discovered during live hardware audits and must never be violated:

### 1. The PiconZero I2C Register Retention Trap
* **Hardware Reality:** The 4tronix PiconZero HAT uses an onboard microcontroller that holds its PWM output state independently of the Raspberry Pi CPU. If the Python process terminates via `SIGTERM`, `SIGINT`, or `kill -9`, the HAT continues driving motors indefinitely at the last commanded speed. Writing `0` to speed registers is sometimes insufficient to override an active runaway state.
* **Mandatory Rule:** All hardware abstraction layers (`RobotHardware`) must register synchronous OS signal handlers (`SIGTERM` / `SIGINT`) and object destructors (`__del__`) that write both zero-power commands and the full board reset command (`CMD_RESET = 20` written to address `0x22`) before process termination.

### 2. LEGO Mechanical Backlash vs. Proportional Gain (`Kp`)
* **Hardware Reality:** In a LEGO Segway topology, gear backlash creates a physical deadband where the motor shaft rotates before teeth engage the wheel. At a stiff Proportional gain (`Kp = 25.0`), even a 4° tilt error clamps PWM to 100%, slamming across the backlash gap and causing violent oscillation.
* **Mandatory Rule:** Never start balancing experiments at `Kp = 25.0`. Always begin at the conservative baseline (`Kp = 10.0`, `Ki = 0.0`, `Kd = 0.5`).

### 3. The Tilt-Chasing Resonance Loop in Soft Recovery
* **Hardware Reality:** Overriding the PID setpoint (`target_angle_offset`) to equal `current_pitch` during active balancing (`abs(pitch) > 5.0°`) converts an inverted pendulum controller into an open-loop horizontal accelerator ("jetting across the room").
* **Mandatory Rule:** Never override `target_angle_offset` during active balancing (`BalancingState`). Soft recovery ramps may only be applied once during wake-up from stationary rest; once balancing, the target angle must remain fixed at calibrated vertical equilibrium (`0.0°` plus trim).

### 4. Global vs. Local Safety Cutoffs (The 4-Second Chaos Rule)
* **Hardware Reality:** Putting safety timeout logic inside individual state classes (`BalancingState`) leaves the robot unprotected if it gets trapped in an earlier phase (such as `ProprioceptiveToddlerStep` discovery or `KickupState`).
* **Mandatory Rule:** Hard-cutoff timers (e.g., a 4.0-second maximum experiment limit) must be enforced at the root application/watchdog level (`main.py` / `SurvivalWatchdog`), independent of state-machine transitions, ensuring the OS unconditionally terminates and resets hardware after the time budget expires.

---

## 8. Operations, Systemd Deployment & Manual REPL

### CLI Execution Flags
For standard command-line usage and interactive flags (`--allow-mocks`, `--reset-brain`, `--auto-fix`, `--repl`), see [README.md CLI Execution & Flags](file:///Users/benhill/Desktop/hobbies/balance_bot/README.md#cli-execution--flags).

### Automatic Startup (Systemd Service)
To run the robot automatically on boot with auto-restart on crash:

1. Create service file `/etc/systemd/system/balance-bot.service`:
   ```ini
   [Unit]
   Description=Balance Bot Controller

   [Service]
   User=pi
   WorkingDirectory=/home/pi/balance-bot
   ExecStart=/home/pi/.local/bin/uv run balance-bot
   Restart=always
   RestartSec=5

   [Install]
   WantedBy=multi-user.target
   ```
2. Enable and start:
   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable balance-bot.service
   sudo systemctl start balance-bot.service
   ```
3. Check status:
   ```bash
   sudo systemctl status balance-bot.service
   ```
4. **Updating Code:** When files are edited, reload the service via `sudo systemctl restart balance-bot.service`. Do not run manual instances while the background service is active (prevents "Address already in use" errors).

---

## 9. Codebase Assumptions & Reference Citations

| Parameter / Assumption | Default Value / Protocol | Reference Citation |
| :--- | :--- | :--- |
| **MPU-6050 I2C Address** | `0x68` (Bus 3: GPIO 17 SDA, 27 SCL @ 100kHz) | [robot_hardware.py](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/hardware/robot_hardware.py) |
| **PiconZero I2C Address** | `0x22` (Byte Writes Required; no Block Writes) | [piconzero.py](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/hardware/piconzero.py) |
| **Motor Channels & Range** | Motor A (`0`) & B (`1`), PWM `-100.0` to `100.0` | [wiring_check.py](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/wiring_check.py) |
| **Control Loop Frequency** | `100 Hz` (`10.0 ms` loop period) | [configuration.py](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/configuration.py) |
| **Default Baseline Gains** | `Kp = 10.0`, `Ki = 0.0`, `Kd = 0.5` | [configuration.py](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/configuration.py) |
| **Max Safe Crash Angle** | `60.0°` (Motors disarmed if pitch exceeds limit) | [configuration.py](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/configuration.py) |
| **Status LEDs** | `/sys/class/leds/led0/brightness` or `ACT` | [leds.py](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/behavior/leds.py) |
