1. **AGENTS.md Update:**
   - Incorporate the new core directives into the `AGENTS.md` file permanently: Hardware Reality over Assumptions, Defense in Depth & I2C Brownouts, Prohibition on Hallucinated Physics, Isolate Inversions, and Strict Personal Operational Constraints.
2. **Audit & Refactor - HAL (Hardware Abstraction Layer):**
   - In `src/balance_bot/hardware/piconzero.py`, modify `_retry` to handle `OSError` by explicitly attempting to write 0 to both motor channels directly over I2C before raising the exception.
   - In `src/balance_bot/hardware/robot_hardware.py`, in `read_imu_raw`, check the `except OSError` block. Before raising the exception (when `_imu_consecutive_errors > self.hw_config.imu_max_retries`), explicitly call `self.pz.stop()` to disarm motors.
3. **Audit & Refactor - Math (`utils.py` and `pid.py`):**
   - In `src/balance_bot/utils.py`, add `def circular_difference(target: float, current: float) -> float` to handle the 180/-180 degree boundary safely.
   - In `src/balance_bot/reflex/balance_core.py`, replace `error = self.pitch - target_angle` with `circular_difference(target_angle, self.pitch)`.
   - In `src/balance_bot/reflex/pid.py`, replace `input_val = self.pid.setpoint - error` with `input_val = self.pid.setpoint - circular_difference(self.pid.setpoint, error)` (or similar depending on actual use) if applicable. Or just use it directly in `update` where `error` is calculated. Actually, in `pid.py`, `input_val = self.pid.setpoint - error` is used to get the raw process variable back for the derivative term. We should ensure the `error` passed in is calculated correctly using circular difference.
4. **Audit & Refactor - Motor Commands (`steps.py` and `robot_hardware.py`):**
   - In `src/balance_bot/discovery/steps.py`, modify `_pulse_and_measure`, `MechanicalBacklashStep.run`, and `_attempt_kick` (and `_force_posture`) to use a ramping loop (e.g., `for p in range(0, target, step): hw.set_motors(p, p); time.sleep(0.01)`) instead of setting `test_power` instantaneously.
5. **Audit & Refactor - Control Loops (`balance_core.py`):**
   - In `src/balance_bot/reflex/balance_core.py`, find and remove the backlash compensation hack (`kick_power = 40.0 * current_sign`), which uses an explicit `current_sign` orientation hack.
   - Verify any remaining `-1` multipliers and ensure they are removed, strictly moving any sign inversions to the HAL.
6. **Audit & Refactor - Exception Handling:**
   - Identify `try/except` blocks swallowing `IOError` or `OSError` in `src/balance_bot/hardware/piconzero.py` (e.g., the `cleanup` function has a bare `except OSError: pass`). Change it to halt the system and explicitly try to disarm actuators without swallowing errors silently, if it doesn't already do so correctly.
7. **Testing and Verification:**
   - Run tests (`uv run pytest`) and linting (`uv run ruff check .`) to verify functionality. Use `read_file` to confirm edits.
8. **Complete pre commit steps to ensure proper testing, verification, review, and reflection are done.**
9. **Submit changes.**
