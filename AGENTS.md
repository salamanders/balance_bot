# AGENTS.md

## 1. Project Philosophy & Hardware Reality
This is for a self-balancing homebrew robot with a Segway-like topology. The hardware is bottom-tier (3D-printed wheels, LEGOs, mismatched motors). Human intervention is a failure of logic.

**Training Wheels (Bumpers):** Stick out at ~20° from vertical. Resting on them is a valid state, not a crash. A crash is face-planting > 50°. Transitioning from Resting to Balancing uses a "Kick-Up" maneuver.

**Goals & Persistence:** The robot must boot via Tabula Rasa, improve over time, remember "last known good" configs, and handle changing battery levels. It outputs a `pid_config.json` upon successful discovery to bypass calibration on future boots.

## 2. LLM Directives & Code Standards

| Category | Constraint |
|---|---|
| **Role & Constraints** | You are a Senior Python Robotics Engineer. Follow the "Do No Harm" Rule. Do not alter runtime logic or physics constants. Leave hardware quirks alone. `src/balance_bot/hardware/piconzero.py` has extra functions (lights, sensors); only worry about motor control code. |
| **Code Modernization** | Use `balance_bot.utils.Vector3`. Use standard type hints, `min()/max()`, and f-strings. Keep Tier 1 control loop (100Hz) mathematically pure; isolate inversions in the HAL. Never optimize untestable hardware I/O. |
| **Testing & Mocks** | If testing without hardware (e.g., CI/laptop), use `uv run balance-bot --allow-mocks`. Without this flag, the code is configured to crash loudly if hardware is missing. |
| **Hardware Reality** | Never swallow errors. Explicitly handle I2C crashes (ramp power to prevent Errno 5 brownouts). No hallucinated physics (e.g., "momentum swings"). Operate under strict "Data Literalism". |
| **Bootstrapping** | Never hardcode physical constants. Remain pessimistic. |
| **Physical Tuning** | Practice graceful degradation. Revert to conservative baseline (Kp=25.0, Ki=0.0, Kd=0.5) if unable to verify states. |

## 3. Architecture Overview
*   **Layered Architecture:** `BalanceCore` (Tier 1) operates as a deterministic, high-frequency reflex loop. `Agent` (Tier 3) handles complex state.
*   **Modern Tooling:** Pydantic is used for validation/serialization (`HardwareConfig`, `LearningState`). The `pid.py` module wraps `simple-pid`. Threads handle async logging (`TelemetryBlackbox`).
*   **Hardware Abstraction:** `RobotHardware` enforces a fail-fast policy, crashing loudly if hardware is missing.

## 4. The "Tabula Rasa" Protocol (Zero-Knowledge Self-Discovery)
The robot deduces its configuration through pessimistic physics experiments. Failure = reverse polarity/increase threshold and retry.

*   **Phase 1: Spark of Life (Presence).** Scan I2C for MPU-6050 and Picon Zero. Halt on failure.
*   **Phase 2: Sense of Down (Gravity).** Motors off. Average 100 accel vectors. Normalized result = Down Vector.
*   **Phase 3/4: Stiction, Phasing, & Polarity.** Ramp PWM to find deadband. Pulse both motors to observe Gyro around Up Vector.
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
