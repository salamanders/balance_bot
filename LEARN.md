# What Was Implemented: Zero-Knowledge "Self-Discovery" Protocol

The `WiringCheck` tool has been upgraded to a fully autonomous state machine that discovers the robot's physical configuration with minimal human intervention.

## Philosophy

*   **Pessimism:** The robot verifies every discovery. If it fixes a problem (e.g., inverts a motor), it re-runs the test to prove the fix worked.
*   **Self-Correction:** The robot wakes up knowing nothing and progressively learns buses, gravity, friction, phasing, direction, and channel mapping.
*   **Human-as-Verifier:** The human is only asked for ground truth when physics cannot deduce it (Left vs Right).

## The Discovery Pipeline

### Tier 1: Hardware Connectivity
*   **`discover_buses()`**: Scans all I2C buses for the Motor Driver (0x22) and IMU (0x68). It validates connection by reading specific registers (e.g., WHO_AM_I).

### Tier 2: The Physical World (Sensors)
*   **`calibrate_static_orientation()`**:
    *   **Gravity:** Measures static acceleration to find the Vertical Axis.
    *   **Pitch:** Uses the Cross Product of "Back" and "Front" vectors (user tips the robot) to mathematically derive the Pitch Axis and its polarity.
    *   **Deduction:** Infers Forward, Yaw, and Roll axes based on the discovered Vertical and Pitch axes.

### Tier 3: Action/Reaction (Motors)
*   **`find_min_power()`**: Pulses motors to find the minimum PWM required to overcome friction, ensuring subsequent tests move the robot.
*   **`align_motors_phase()` (Pessimistic Loop)**:
    *   Drives both motors "Forward".
    *   Checks Yaw Rate. If high, motors are fighting (Spinning).
    *   **Fix:** Inverts one motor.
    *   **Verify:** Re-runs the test to confirm Yaw Rate is now low (Straight).
*   **`determine_motor_direction()` (Pessimistic Loop)**:
    *   "Kick Up" Test: Pulses motors Positive.
    *   Checks Pitch change. If lean increases, direction is wrong.
    *   **Fix:** Inverts both motors.
    *   **Verify:** Re-runs the test to confirm the robot now moves towards the upright position.

### Tier 4: The Human Anchor
*   **`ask_human_left_right()`**:
    *   Robot spins (commands Left Turn).
    *   Records Gyro Yaw Rate during the spin.
    *   Asks human: "Did I spin Left or Right?"
    *   **Channel Check:** If human input contradicts command, swaps Motor Channels (Left <-> Right) and **re-verifies**.
    *   **Gyro Calibration:** Compares Human Truth (Left/Right) with Gyro Truth (Positive/Negative) to calibrate `gyro_yaw_invert`.

### Tier 5: Dynamics
*   **`find_flop_thresholds()`**: Empirically determines the power needed to "kick up" from flat on the floor to the upright balance point.

### Tier 6: Final Verification
*   **`verify_final_configuration()`**:
    *   Autonomous drive test.
    *   Proves the robot can drive straight (Low Yaw).
    *   Proves the robot can turn Right (Negative Yaw).
    *   Fails if any physical behavior contradicts the internal model.

---

# Future Improvements (TODO)

*   [ ] **Encoder Support:** If encoders are added, use them to verify distance traveled and refine friction thresholds.
*   [ ] **Center of Mass Calibration:** Use the integrator term during balancing to auto-tune the mechanical balance point offset.
