# Codebase Redundancy and Refactoring Analysis

This document identifies areas in the codebase that exhibit redundancy, violate the DRY (Don't Repeat Yourself) principle, or present opportunities for refactoring to reduce complexity and line count.

## 1. Tuning Rate Decay

**Observation:**
The variable `tuning_aggression` in `src/balance_bot/main.py` is initialized and then decayed over time within the main `run()` loop:

```python
# In main.py: run()
tuning_aggression = 5.0 if self.first_run else 1.0
...
while self.running:
    ...
    # Decay Aggression
    if tuning_aggression > 0.1:
        tuning_aggression *= 0.9995
```

**Status:**
-   **Duplication:** Not strictly duplicated. This decay logic exists only in `main.py`.
-   **Refactoring Opportunity:** The logic is hardcoded (magic numbers `5.0`, `0.1`, `0.9995`). This "decaying temperature" behavior is a property of the tuning strategy and could be encapsulated within the `ContinuousTuner` class (e.g., `tuner.get_current_scale()`) or defined in `TunerConfig`. This would remove state management from the main loop.

## 2. DRY Violations (Don't Repeat Yourself)

### A. Control Loop Duplication in `recover_from_rest`
The method `recover_from_rest()` in `main.py` re-implements a substantial portion of the main control loop logic found in `run()`.

**Redundant Logic:**
1.  **IMU Reading:** `reading = self.get_pitch(self.config.loop_time)`
2.  **LED Update:** `self.led.update()`
3.  **PID Calculation:** `output = self.pid.update(...)`
4.  **Motor Output:** `self.hw.set_motors(output, output)`
5.  **Crash Detection:** `if abs(self.pitch) > CRASH_ANGLE: ...`
6.  **Timing Control:** `rate.sleep()`

**Inconsistencies:**
-   **Battery Compensation:** The main loop applies battery compensation (`final_drive = output / comp_factor`). The recovery loop **does not**. This means motor performance will vary with battery voltage during recovery, unlike in normal operation.
-   **Turn Correction:** The main loop applies yaw correction. The recovery loop does not (which is acceptable, but inconsistent structure).

### B. Wait Loops
The method `_wait_for_stable_pitch()` in `main.py` implements a third, smaller loop solely to read sensors and wait.

```python
while time.monotonic() < end_time:
    self.get_pitch(self.config.loop_time)
    time.sleep(self.config.loop_time)
```

This pattern of "loop while reading sensors" is repeated three times in `main.py` (Main run, Recovery, Wait).

## 3. Refactoring Opportunities

### A. Unified Control Loop
The `run()` and `recover_from_rest()` loops could be merged. The "recovery" phase is essentially just a dynamic setpoint adjustment.

**Proposed Solution:**
Introduce a `SetpointGenerator` or `TrajectoryPlanner` that determines the current `target_angle`.
-   **Normal Mode:** Target = `config.pid.target_angle`
-   **Recovery Mode:** Target = Ramps from `current_angle` to `0`.

The main loop would then look like:
```python
target = setpoint_generator.get_target()
error = pitch - target
output = pid.update(error, ...)
drive(output)
```
This eliminates the entire `recover_from_rest` method, reducing code by ~50 lines and ensuring battery compensation applies everywhere.

### B. Centralized Battery Compensation
Currently, battery compensation is applied manually in `_step_balance`.

**Proposed Solution:**
Move this logic into a wrapper around the motors (e.g., `MotorController`) or directly into `RobotHardware`.
`self.hw.set_motors(left, right)` could automatically query `self.battery.get_compensation_factor()` and scale the output. This ensures that *any* call to set motors (from recovery, diagnostics, or main loop) is voltage-compensated.

### C. Config Cleanup
-   **Magic Numbers:** Move `tuning_aggression` start/end values and decay rate to `TunerConfig`.
-   **Legacy Migration:** `RobotConfig.load` handles legacy keys. If legacy config files are no longer in use, this code can be removed to simplify the class.
