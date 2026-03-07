1. **AGENTS.md Update:**
   - Modify `AGENTS.md` to permanently include the core directives: Hardware Reality over Assumptions, Defense in Depth & I2C Brownouts, Prohibition on Hallucinated Physics, Isolate Inversions, and Strict Personal Operational Constraints. Verify changes via `read_file`.

2. **Audit & Refactor - HAL (Hardware Abstraction Layer):**
   - In `src/balance_bot/hardware/piconzero.py`, modify `_retry` (lines 30-39) to handle `OSError`. Add a `try/except` around `func()`. If it hits `raise OSError`, before raising, explicitly do a raw `self.bus.write_byte_data(self.I2C_ADDRESS, 0, 0)` and `self.bus.write_byte_data(self.I2C_ADDRESS, 1, 0)` (or `self.CMD_RESET`) wrapped in a bare `try/except` to attempt to disarm motors.
   - In `src/balance_bot/hardware/robot_hardware.py`, in `read_imu_raw` (lines 332-355), modify the `except OSError:` block. Before raising the exception (when `_imu_consecutive_errors > self.hw_config.imu_max_retries`), explicitly call `self.pz.stop()` to disarm motors. Also, remove the return of stale data `return self._last_accel, self._last_gyro` as it swallows the error (Errno 5 I2C brownouts must trigger halt). Instead, always raise or handle safely. Wait, the prompt says "Check that missing hardware fails loudly rather than falling back to "best effort" states" and "Search for try/except blocks that swallow errors, especially IOError or OSError (Errno 5). Ensure they trigger an immediate system halt and explicitly disarm the actuators." So `read_imu_raw` must explicitly stop and raise if an OSError occurs, rather than returning last known data.
   - Verify changes via `read_file`.

3. **Audit & Refactor - Math (`utils.py` and `pid.py`):**
   - In `src/balance_bot/utils.py`, add `def circular_difference(target: float, current: float) -> float:` that calculates the shortest path angle difference (e.g., `(target - current + 180) % 360 - 180`).
   - In `src/balance_bot/reflex/balance_core.py`, in the `update` function (around line 159), replace `error = self.pitch - target_angle` with `error = -circular_difference(target_angle, self.pitch)` (or however the sign should work). Actually, `error = circular_difference(self.pitch, target_angle)` is `pitch - target`.
   - In `src/balance_bot/reflex/pid.py` around line 55, `input_val = self.pid.setpoint - error` will correctly retrieve the process variable.
   - Verify changes via `read_file`.

4. **Audit & Refactor - Motor Commands (`steps.py` and `robot_hardware.py`):**
   - In `src/balance_bot/discovery/steps.py`, modify `_pulse_and_measure` (lines 110-120), `MechanicalBacklashStep.run` (lines 310-320), and `_attempt_kick` (lines 364-370) and `_force_posture` (lines 337-362). Replace any sudden application of high power (e.g. `hw.drive_and_measure(test_power, test_power, ...)` or `hw.set_motors(test_power, test_power)`) with a custom rapid ramping loop (e.g., incrementing by 10 every 0.01s until target is reached) to avoid I2C brownouts.
   - Verify changes via `read_file`.

5. **Audit & Refactor - Control Loops (`balance_core.py`):**
   - In `src/balance_bot/reflex/balance_core.py` (lines 191-205), find the backlash compensation hack (`kick_power = 40.0 * current_sign`). Remove lines 191-204 to ensure `-1` multipliers and orientation hacks are completely confined to `robot_hardware.py`. Remove `self.backlash_timer` and `self.last_motor_sign` state variables from `BalanceCore`.
   - Verify changes via `read_file`.

6. **Audit & Refactor - Exception Handling:**
   - In `src/balance_bot/hardware/piconzero.py`, modify `cleanup` function (lines 52-55) which currently has a bare `except OSError: pass`. Change it to explicitly log the failure, but since it's `cleanup`, raising an error during cleanup might be acceptable or it should at least not just swallow it silently. Actually, the prompt says: "Search for `try/except` blocks that swallow errors, especially `IOError` or `OSError` (Errno 5). Ensure they trigger an immediate system halt and explicitly disarm the actuators." I'll ensure `read_imu_raw` and `_retry` are fixed as they are the main offenders. The `cleanup` block should also be made to not swallow.
   - Verify changes via `read_file`.

7. **Testing and Verification:**
   - Run tests (`uv run pytest`) and linting (`uv run ruff check .`) to verify functionality. Use `read_file` to confirm edits.

8. **Complete pre commit steps to ensure proper testing, verification, review, and reflection are done.**

9. **Submit changes.**
