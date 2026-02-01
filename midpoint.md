# Finding the True Balance Point

## Problem Statement

The robot currently determines its target angle ("Zero") using one of two methods:
1.  **Geometric Midpoint**: Assuming the midpoint between the two training-wheel resting points is the balance point.
2.  **Static Calibration**: The user holds the robot "vertical" and calibrates.

However, neither of these guarantees a true **Dynamic Balance Point**.
*   **Geometric Midpoint**: Fails if the robot's Center of Mass (CM) is not perfectly centered relative to the chassis geometry (e.g., uneven battery placement, messy wiring).
*   **Static Calibration**: Relies on human estimation of "vertical", which is imprecise.

If the robot's target angle is not the true balance point, the PID controller must constantly exert force to hold the robot at the "wrong" angle. This manifests as a **Constant Steady-State Error** or a **Constant Motor Offset** (Integral Windup), wasting battery and reducing stability margin.

## Proposed Solution: Effort Minimization

The robot can autonomously find its true balance point by minimizing the **Control Effort** required to stay stationary.

*   **Hypothesis**: At the true balance point, the average motor output required to maintain that angle is effectively zero (ignoring friction).
*   **Observation**:
    *   If the robot constantly drives **Forward** (Positive Motor Output) to stay upright, it implies the robot is leaning **Forward** relative to its true balance point. It is "chasing" its own Center of Mass.
    *   If the robot constantly drives **Backward** (Negative Motor Output), it implies the robot is leaning **Backward**.

### The Algorithm: "Drift and Correct"

We can implement a background process (similar to the `ContinuousTuner`) that slowly adjusts the `target_angle` based on the average motor output.

#### 1. Logic Rule
*   If **Average Motor Output > Threshold** (Positive/Forward):
    *   **Action**: *Decrease* Target Angle (Lean Back).
    *   *Why?* The robot thinks it's at 0, but physics says it's falling forward. By decreasing the target (e.g., to -1), we force the robot to lean back, bringing the CM over the wheels.
*   If **Average Motor Output < -Threshold** (Negative/Backward):
    *   **Action**: *Increase* Target Angle (Lean Forward).
    *   *Why?* The robot is fighting a backward fall. Tilting the target forward aligns the frame with the balance point.

**Formula**:
`New_Target = Current_Target - (Average_Motor_Output * Learning_Rate)`

#### 2. Sampling Phase
The robot must be in a **Stable State** to run this logic.
*   **Condition**: `RobotState == BALANCE`
*   **Condition**: `Abs(Pitch_Rate) < Stability_Threshold` (Not wobbling violently)
*   **Window**: Collect motor output samples over a sliding window (e.g., 5 seconds / 500 ticks).

#### 3. Update Frequency
*   This process should be slow. Updates should happen perhaps once every 5-10 seconds to avoid chasing transient noise (like being pushed or driving over a bump).
*   Use a `Learning_Rate` that results in very small angle changes (e.g., 0.1 degrees per update).

#### 4. Persistence
*   The calibrated `target_angle` should be saved to `pid_config.json` automatically (using the existing `RobotConfig.save()` mechanism) so the robot gets "smarter" every time it runs.

## Safety Considerations

1.  **Max Deviation Clamp**:
    *   The `target_angle` should never deviate more than +/- 10 degrees from the factory default (0). If it tries to go further, something is wrong (hardware failure, bent frame).
2.  **Movement Detection**:
    *   If the robot is intentionally driving (e.g., remote control), this calibration must be **PAUSED**. We only calibrate when the desired velocity is zero.
3.  **Oscillation**:
    *   If the calibration overshoots (Target goes 0 -> 2 -> 0 -> 2), the `Learning_Rate` is too high.

## Implementation Steps

1.  **Modify `ContinuousTuner` (or create `BalancePointFinder`)**:
    *   Add a buffer for `motor_output_history`.
    *   Add logic to calculate `mean_motor_output`.
2.  **Integrate into `main.py`**:
    *   In `_step_balance`, feed the current motor output into the finder.
    *   Apply the returned offset to `self.config.pid.target_angle`.
3.  **Config Updates**:
    *   Add `balance_calibration_rate` and `max_balance_deviation` to `TunerConfig`.

This strategy fulfills the goal: "It should improve over time as it gets more experience."
