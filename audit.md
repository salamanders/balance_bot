# Balance Bot Codebase Audit

## Executive Summary

The codebase for the self-balancing robot is highly ambitious, architecturally sound, and philosophically rigorous. It strictly adheres to a "Tabula Rasa" self-discovery model, where the robot deduces its physical configuration (I2C buses, motor phases, kinematics, and IMU orientations) entirely through physical experiments rather than hardcoded configurations. The separation of concerns between high-level behavior (`Tier 3`), adaptive logic (`Tier 2`), and low-level deterministic motor reflexes (`Tier 1`) is well thought out.

However, recent architectural changes—specifically the splitting of a monolithic `RobotConfig` into immutable `HardwareConfig` and mutable `LearningState`, and the replacement of the legacy `WiringCheck` with the robust `SelfDiscoveryPipeline`—have left the test suite in a fractured state. Currently, many tests are broken due to outdated imports and structural changes. The code will absolutely run, but the test suite currently fails to run at all.

## Code Quality & Architecture

### The Good
*   **Layered Architecture (The "Brain/Brainstem" Model):** The codebase beautifully separates concerns. `BalanceCore` (Tier 1) operates as a deterministic, high-frequency reflex loop, isolated from the complex state management of the `Agent` (Tier 3).
*   **The "Tabula Rasa" Protocol:** The self-discovery steps (`src/balance_bot/discovery/steps.py`) execute a brilliant physical logic grid to determine hardware mapping (motor phasing, axis dominance, Left/Right chirality via the Right-Hand Rule).
*   **Modern Python Tooling:** Extensive use of `pydantic` for configuration validation and serialization (`HardwareConfig`, `LearningState`), dataclasses for data transfer (`MotionRequest`, `BalanceTelemetry`), and type hinting throughout.
*   **Fail-Fast Hardware Abstraction:** `RobotHardware` enforces a fail-fast policy. If expected hardware is missing, it crashes loudly rather than silently failing, which is critical for robotics.
*   **Continuous Adaptation:** The `ContinuousTuner` and `BalancePointFinder` mechanisms allow the robot to adjust its PID values and physical center of mass offset dynamically without human intervention.
*   **Resource Management:** Good use of threads for asynchronous logging (`TelemetryBlackbox`) and config saving, preventing I/O operations from blocking the critical control loop.

### Needs Improvement
*   **Test Suite Maintenance:** The test suite is currently severely broken due to recent refactors. Files like `tests/test_agent_state_machine.py` and `tests/test_autonomous_config.py` still reference deprecated modules (`WiringCheck`) and legacy configuration classes (`RobotConfig`).
*   **Import Paths in Tests:** Many tests fail to import the source code properly (e.g., `ModuleNotFoundError: No module named 'src'`). Test files need to import from the top-level package (`balance_bot`) and rely on the PYTHONPATH or `uv` environment, or use relative imports correctly.
*   **Linting/Formatting Exceptions:** `uv run ruff check .` found 68 errors, mostly unused imports, multiple statements on a single line, and ambiguous variable names (e.g., `l` instead of `left_power`). These should be cleaned up via `ruff check --fix`.

## Documentation Status

*   **`AGENTS.md` is Excellent:** The documentation in `AGENTS.md` is outstanding. It clearly outlines the constraints (cheap hardware, 20-degree rest angles), the "Tabula Rasa" philosophy, and the exact step-by-step physical experiments the robot must perform. It provides essential context that isn't immediately obvious from reading the code.
*   **Code Documentation (Docstrings):** Docstrings are generally present and well-written, particularly in complex areas like the `BalanceCore` and `Agent` classes.
*   **Missing Legacy References:** Some test documentation and inline comments might still refer to deleted files like `SCRIPT.md` and `LEARN.md` (which have been consolidated into `AGENTS.md`).

## Identified Bugs / Broken Components

*   **Test Suite is Un-runnable:** Running `pytest` results in 11 collection errors immediately due to `ImportError`.
    *   `src` package prefix errors: Many tests use `from src.balance_bot...` instead of `from balance_bot...`
    *   `RobotConfig` removal: Tests attempt to import `RobotConfig` from `balance_bot.configuration`, which has been replaced by `HardwareConfig` and `LearningState`.
    *   `WiringCheck` removal: Tests like `test_autonomous_config.py` try to import `WiringCheck`, which was replaced by the `SelfDiscoveryPipeline`.
*   **`AGENTS.md` Code References are slightly outdated:** While the concepts in `AGENTS.md` are perfect, it references `RobotConfig.save` in the Output & Persistence section, which is now handled by `HardwareConfig.save()` and `LearningState.save()`.
*   **Nested Model Updates in `LearningState`:** In `MechanicalBacklashStep` and `KickupDynamicsStep`, the code uses `state.control.model_copy(update=...)` but passes it as a dictionary update string to `state_updates` rather than replacing the `ControlConfig` instance directly, which might lead to the pipeline incorrectly applying the update depending on how `setattr` handles nested Pydantic models.

## Actionable Recommendations

1.  ~~**Fix the Test Suite Imports:** Perform a global search and replace in the `tests/` directory:~~ [DONE]
    *   ~~Replace `from src.balance_bot` with `from balance_bot`.~~ [DONE]
    *   ~~Replace `RobotConfig` imports and instantiations with `HardwareConfig` and/or `LearningState` depending on the context of the test.~~ [DONE]
    *   ~~Rewrite `WiringCheck` tests to target the new `CalibrationStep` subclasses in `src/balance_bot/discovery/steps.py`.~~ [DONE]
2.  ~~**Run Linting Fixes:** Execute `uv run ruff check . --fix` to clean up the unused imports and stylistic errors flagged by the linter.~~ [DONE]
3.  ~~**Audit the Pipeline State Update Logic:** Review `SelfDiscoveryPipeline.run()`. When a step returns `state_updates`, ensure that `setattr(self.state, k, v)` works correctly when `v` is a nested Pydantic model (like `ControlConfig`) rather than a primitive type.~~ [DONE]
    *   *Audit Result:* Verified. `state_updates` passes the fully instantiated Pydantic model (`ControlConfig`) back to the pipeline. `setattr(self.state, 'control', new_control_config)` correctly replaces the nested model in `LearningState` without type coercion issues. Pydantic's assignment validation perfectly handles this swap.
4.  ~~**Update `AGENTS.md`:** Do a quick pass over `AGENTS.md` to update class names (e.g., changing references from `RobotConfig` to `HardwareConfig` and `LearningState`) to match the new source code reality perfectly.~~ [DONE]


## Cross-Product Expansion & Failure Modes

The `DeriveKinematicsStep` is designed to deduce physical configurations dynamically. The hardware parameter space consists of:
- **Left/Right Motor Polarity:** Standard or Reversed (changes whether +PWM moves the wheel Forward or Backward).
- **Left/Right Motor Channel Mapping:** Which I2C channel maps to which physical side.
- **IMU Orientation:** The 3D orientation of the IMU relative to the robot chassis (Pitch, Yaw, Roll, and Inversion).

### Success Modes

1. **Standard Configuration (No inverts, standard mount):**
   - Phase matches. Pitch and Forward axes identified successfully. `raw_up` properly identifies Vertical.
   - **Result: SUCCESS.** Robot balances perfectly.

2. **Single Motor Reversed Polarity (e.g., Left Motor Backwards):**
   - `DeriveKinematicsStep` detects a phase mismatch during the pulse test and aligns the 'Normal' motor to the 'Reversed' motor. Both motors now move backward on +PWM.
   - The algorithm redefines 'Forward' as 'Backward' physically. It then incorrectly deduces that Left and Right channels are swapped relative to this new direction.
   - **Result: SUCCESS.** Pitch and Yaw senses are inverted to match the reversed motors. The robot successfully balances and steers, but the physical 'Front' becomes the logical 'Back'.

3. **Both Motors Reversed Polarity:**
   - Phase matches immediately (both go backwards). Pitch and Accel signals invert.
   - **Result: SUCCESS.** The robot operates perfectly, similarly redefining its 'Front' as 'Back'.

4. **IMU Mounted Sideways / Upside Down:**
   - `analyze_dominance` dynamically reassigns axes. The calculation of physical Up (`raw_up = glm.cross(sum_accel, sum_gyro)`) is invariant to sensor orientation because it relies on the physical cross product of Lag and Pitch.
   - **Result: SUCCESS.** Axis remapping perfectly accommodates 90-degree and 180-degree physical rotations of the IMU.

### Failure Mode: Phase Mismatch with Positive Feedback (The "Spinning in a Circle" Issue)

There is a specific edge-case failure mode involving the physical environment rather than the algorithm itself, which leads directly to the robot **"spinning in a circle, but thinking it was tipping over."**

**The Catalyst:**
If the robot's wheels lack traction (e.g., slipping on a smooth surface) or the pulse power is too weak, the single-motor wiggle generates extremely low linear acceleration (`accel_delta`). If this signal falls below the noise floor or is overpowered by gravity tilt:
- `glm.dot(l_accel_delta, r_accel_delta)` may incorrectly evaluate as positive, failing to detect an actual phase mismatch.

**The Cascade:**
1. **Axis Contamination:** With opposing motors incorrectly considered 'matched', one motor pulses forward and the other backward. This produces pure physical **Yaw** rather than Pitch.
2. **Axis Swapping:** `sum_gyro` becomes dominated by Yaw rotation. The calibration logic mistakenly assigns the IMU's Pitch Axis to the physical Yaw axis.
3. **Positive Feedback Loop:**
   - When the robot attempts to balance, a slight physical forward tilt is mistakenly read as a Yaw error.
   - The PID controller attempts to correct this 'Yaw' by driving the wheels in opposite directions.
   - This opposing drive spins the robot in a circle physically.
4. **Centripetal False-Pitch:** As the robot spins rapidly in a circle, centripetal acceleration acts on the off-center IMU. This radial acceleration registers on the IMU as continuous Pitch (tipping), causing the robot to drive the motors harder, exacerbating the spin.

**Conclusion:** The observed behavior is not a logical gap in the cross-product mapping, but an environmental failure causing the initial phase-alignment to fail, leading to Pitch and Yaw axes becoming perfectly swapped in software.

**Fix:** Fixed in `src/balance_bot/discovery/steps.py` by relying on the Gyro's yaw vectors (`yaw_mag_sum` vs `yaw_mag_diff` around the Up vector) instead of linear acceleration (`accel_delta`), making the phase alignment robust to low-traction wheel slipping. [DONE]
