# AGENTS.md

## 1. Project Philosophy & Hardware Reality
This is for a self-balancing homebrew robot with a Segway-like topology. The hardware is bottom-tier (3D-printed wheels, LEGOs, mismatched motors). Human intervention is a failure of logic.

**Training Wheels (Bumpers):** Stick out at ~20° from vertical. Resting on them is a valid state, not a crash. A crash is face-planting > 50°. Transitioning from Resting to Balancing uses a "Kick-Up" maneuver.

**Goals & Persistence:** The robot must boot via Tabula Rasa, improve over time, remember "last known good" configs, and handle changing battery levels. It outputs a `pid_config.json` upon successful discovery to bypass calibration on future boots.

## 2. LLM Directives & Code Standards

| Category | Constraint |
| :--- | :--- |
| **Role & Constraints** | You are a Senior Python Robotics Engineer. Follow the "Do No Harm" Rule. Do not alter runtime logic or physics constants. Leave hardware quirks alone. `src/balance_bot/hardware/piconzero.py` has extra functions (lights, sensors); only worry about motor control code. |
| **Mandatory Guidelines** | **Before writing or modifying any code, you MUST read `.jules/agent_instructions.md`.** It contains critical instructions on verifying function signatures and type safety to prevent regressions. |
| **Code Modernization** | Use `balance_bot.utils.Vector3`. Use standard type hints, `min()/max()`, and f-strings. Keep Tier 1 control loop (100Hz) mathematically pure; isolate inversions in the HAL. Never optimize untestable hardware I/O. |
| **Testing & Mocks** | If testing without hardware (e.g., CI/laptop), use `uv run balance-bot --allow-mocks`. Without this flag, the code is configured to crash loudly if hardware is missing. |
| **Hardware Reality** | Never swallow errors. Explicitly handle I2C crashes (ramp power to prevent Errno 5 brownouts). No hallucinated physics (e.g., "momentum swings"). Operate under strict "Data Literalism". |
| **Bootstrapping** | Never hardcode physical constants. Remain pessimistic. |
| **Physical Tuning** | Practice graceful degradation. Revert to conservative baseline (Kp=25.0, Ki=0.0, Kd=0.5) if unable to verify states. |

### Architectural Anti-Pattern: Context-Blind Fault Intolerance

When generating control code for cyber-physical systems, AI agents frequently misapply high-level software engineering paradigms—specifically strict "fail-fast" exception handling—to low-level hardware interfaces. In a real-time physical environment, transient I/O errors (such as I2C glitches caused by motor EMI or voltage sags) are nominal operating conditions, not fatal system failures. Enforcing strict data-purity rules during a continuous actuation loop causes trivial hardware noise to collapse the entire system. To eliminate this class of bug, agents must adhere to the following directives: strictly separate initialization logic (where failing fast is required) from the real-time control loop (which requires absolute fault tolerance); replace fatal exceptions with continuous data quality metrics (e.g., returning cached data alongside an age/error counter) to keep the loop alive; and actively inject simulated physical noise (like jitter and correlated EMI dropouts) into unit tests to verify system resilience.

## 3. Architecture Overview
*   **Layered Architecture:** `BalanceCore` (Tier 1) operates as a deterministic, high-frequency reflex loop. `Agent` (Tier 3) handles complex state.
*   **Modern Tooling:** Pydantic is used for validation/serialization (`HardwareConfig`, `LearningState`). The `pid.py` module wraps `simple-pid`. Threads handle async logging (`TelemetryBlackbox`).
*   **Hardware Abstraction:** `RobotHardware` enforces a fail-fast policy, crashing loudly if hardware is missing.

## 4. The "Tabula Rasa" Protocol (Zero-Knowledge Self-Discovery)
The robot deduces its configuration through pessimistic physics experiments. Failure = reverse polarity/increase threshold and retry.

*   **Phase 1: Spark of Life (Presence).** Scan I2C for MPU-6050 and Picon Zero. Halt on failure.
*   **Phase 2: Sense of Down (Gravity).** User-assisted lean step. Robot records backwards lean, waits for user to flop it forward, and records front lean. The robot infers vertical and forward axes and guesses the balance point range without moving motors.
*   **Phase 3/4: Stiction, Phasing, & Polarity.** Ramp PWM to find deadband, explicitly tracking any hardware I2C glitches or ignored commands. Pulse both motors to observe Gyro around Up Vector.
    *   *Spinning Failure Mode:* If the bot spins in a circle, wheels slipped, causing pure Yaw rather than Pitch. **Fix:** Rely on Gyro yaw vectors (`yaw_mag_sum` vs `yaw_mag_diff`), not linear acceleration.
    *   *Forward Check:* Lean forward, pulse positive. If pitch decreases (stands up), Positive = Forward. If it increases, invert global polarity.
*   **Phase 5: Absolute Identity (L/R).** Command +PWM on Port A, -PWM on Port B. By the Right-Hand Rule, Gyro pointing Up = Counter-Clockwise (Left) spin, meaning Port A = Right Motor.
*   **Phase 6: Motor Trimming.** Drive forward open-loop. Integrate Yaw. Apply fractional multiplier to stronger motor.
*   **Phase 7: The True Balance Point (Effort Minimization).** Hover with `target_angle = 0.0°`. Monitor Integral term (avg motor output).
    *   If positive/forward drift, robot is "nose heavy". Decrease target angle (Lean Back).
    *   If negative/backward drift, Increase target angle (Lean Forward).
    *   Clamp to safe limits (+/- 10°), learn slowly, save to `pid_config.json` containing: `i2c_bus`, `axis_map`, `motor_map`, `hardware_limits`, and `dynamics`.

## Appendix A: Future Architecture ("Proprioceptive Toddler")
A proposed Dependency-Driven Discovery Engine replacing linear scripts with "Motor Babbling." The robot builds a Knowledge Graph of atomic facts (`HardwareBus`, `GravityVector`, `ChassisHandedness`). "Trauma" (falls, contradiction) invalidates specific atoms, turning failures into data without full state-machine exits.

## **⚠️ Critical LLM Blind Spots in Cyber-Physical Systems**

As an asynchronous coding agent operating on this repository, you possess world-class capabilities in symbolic logic, test generation, dependency management, and high-level Python refactoring. However, you must recognize that you have an inherent, structural blind spot regarding the physical and temporal realities of this specific robot, and spend extra time (and tokens\!) counteracting the blind spots.

Your underlying training data heavily indexes on idealized academic robotics (where hardware kinematics are perfect and exhaustively documented) and basic software tutorials. Consequently, you are less equipped to natively understand the "scrappy," low-level hardware friction that this project handles on a daily basis. You will naturally be biased toward optimizing, cleaning up, or removing code that looks inefficient to a software engineer, but which actually serves as critical physical scaffolding for the robot's survival.

Before proposing any code modifications, you must focus heavily on the following areas and self-double-check your assumptions against the physical realities outlined below.

| Repository Component | The LLM Blind Spot (Your Bias) | The Physical Rule (Your Constraint) |
| :--- | :--- | :--- |
| **Zero-Knowledge Bootstrapping** | You will view calibration routines (like The Flop, Kick-Up, and the motor polarity checker) as clunky, blocking state-machines that could be simplified by hardcoding variables. | **Never hardcode physical constants.** This robot is assembled differently every time. Do not bypass the discovery of motor mappings, min\_power\_visible (friction thresholds), or MPU-6050 orientation arrays. This code must remain pessimistic. |
| **Temporal Determinism (The Brainstem)** | You will assume that standard Python refactoring (adding matrix math libraries, standardizing logging, abstracting functions into classes) is inherently good. | **Strict 100Hz execution is mandatory.** Any microsecond of latency added to Tier 1 will cause a physical crash. If you touch the Brainstem, you must ensure execution time is unaffected. Do not introduce garbage-collection-heavy operations or async calls here. |
| **Hardware Communication Nuances** | You will see standard I2C logic and assume generic libraries apply. You will see GPIO pin selections and assume they can be easily swapped or optimized. | **Hardware quirks cannot be abstracted.** The physical motor HAT blocks standard pins, requiring a custom software-defined "bus 3" (pins 17/27) running at \~100kHz. The driver requires *Byte Writes*, not standard *Block Writes*. Replacing these with standard libraries will sever physical control. |
| **Physical Tuning (The Unverified Zone)** | If asked to tune the Cerebellum or write self-optimizing code, you will rely on semantic log outputs. You may misinterpret a safety shutdown as a stable state because you lack hardware profiling. | **Practice graceful degradation.** If you cannot verify a physical state with absolute confidence, revert to the conservative baseline (Kp=25.0, Ki=0.0, Kd=0.5). Do not engage in infinite loops of parameter tuning without physical verification; beware of generating algorithmic slop. |

**Summary for the Agent:** You are operating in the physical world. Software elegance is strictly secondary to data driven resolutions. If code within this repository looks strangely specific, overly cautious regarding timing, heavily dependent on calibration maneuvers, or utilizes non-standard bus communication, **leave it alone**. That specific logic is the only thing keeping the robot physically upright.  Likewise, you are **coding blind**: you write a bootstrap sequence, but you can't run it directly in the hardware to see if it works in real-world situations, so **assume the world is going to be difficult in unknown ways, and always code defensively to deal with it**.
