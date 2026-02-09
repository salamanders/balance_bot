# Finding the True Balance Point (Implemented)

## Problem Statement

The robot initially determines its target angle ("Zero") using static calibration or geometric midpoint. However, the **Dynamic Balance Point** depends on the Center of Mass (CM), which may not align with the geometric center due to battery placement, wiring, or other asymmetries.

If the target angle is incorrect, the PID controller must constantly exert force to maintain that angle, resulting in a **Constant Steady-State Error** (Integral Windup) and reduced battery life.

## Solution: Effort Minimization

The robot autonomously finds its true balance point by minimizing the **Control Effort** required to stay stationary. This logic is implemented in `src/balance_bot/adaptation/tuner.py` via the `BalancePointFinder` class.

*   **Principle**: At the true balance point, the average motor output required to maintain that angle should be zero (ignoring friction).
*   **Observation**:
    *   **Forward Drift**: If the robot constantly drives **Forward** (Positive Motor Output) to stay upright, it is leaning **Forward** relative to its true balance point.
    *   **Backward Drift**: If the robot constantly drives **Backward** (Negative Motor Output), it is leaning **Backward**.

### The Algorithm: "Drift and Correct"

The `BalancePointFinder` runs in the background and slowly adjusts the `target_angle` based on the average motor output.

#### 1. Sampling Phase
The robot collects motor output samples only when it is in a **Stable State**:
*   **Condition**: The robot is balancing (not crashed).
*   **Condition**: `Abs(Pitch_Rate) < Threshold` (Not wobbling).
*   **Condition**: User is not requesting movement (Velocity target is 0).

#### 2. Analysis & Update
Once enough samples are collected (defined by `balance_check_interval`), the average motor output is calculated.

*   If **Average Motor Output > Threshold** (Positive/Forward):
    *   **Action**: *Decrease* Target Angle (Lean Back).
    *   *Effect*: Shifts the CM backward over the wheels.
*   If **Average Motor Output < -Threshold** (Negative/Backward):
    *   **Action**: *Increase* Target Angle (Lean Forward).
    *   *Effect*: Shifts the CM forward over the wheels.

**Formula**:
`New_Target = Current_Target +/- Learning_Rate`

#### 3. Safety Mechanisms
*   **Max Deviation Clamp**: The target angle is clamped to a safe range (e.g., +/- 10 degrees) to prevent dangerous tilts.
*   **Slow Learning**: The learning rate is small to ensure stability and avoid oscillation.
*   **Persistence**: The calibrated `target_angle` is saved to `pid_config.json` via the `Agent`'s configuration saving mechanism, allowing the robot to "learn" its balance point over multiple sessions.
