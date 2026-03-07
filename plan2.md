1. **AGENTS.md Update:**
   - Modify `AGENTS.md` and insert the requested core directives: Hardware Reality over Assumptions, Defense in Depth & I2C Brownouts, Prohibition on Hallucinated Physics, Isolate Inversions, and Strict Personal Operational Constraints.
2. **Audit HAL:**
   - In `src/balance_bot/hardware/piconzero.py`, modify `_retry` to handle `OSError` by explicitly halting (disarming) on fail: catch `OSError`, issue a raw bus reset or set motors to 0, then raise. Ensure any swallowed I2C brownouts (Errno 5) trigger immediate halt.
   - In `src/balance_bot/hardware/robot_hardware.py`, in `read_imu_raw`, check the `except OSError` block. Instead of returning `self._last_accel` infinitely for glitches, ensure it halts actuators and fails loudly if the consecutive errors limit is hit. We need to explicitly call `self.pz.stop()` or similar before raising.
3. **Audit Math:**
   - In `src/balance_bot/utils.py`, add `def circular_difference(target: float, current: float) -> float:` to return shortest path.
   - In `src/balance_bot/reflex/balance_core.py` and `src/balance_bot/reflex/pid.py`, replace `error = self.pitch - target_angle` (and similar pitch math) with `circular_difference(target_angle, self.pitch)`.
4. **Audit Motor Commands:**
   - In `src/balance_bot/discovery/steps.py`, modify `_pulse_and_measure`, `MechanicalBacklashStep`, and `_force_posture` / `_attempt_kick` to use a ramping function (e.g. `ramp_power(hw, start, target, steps)`) instead of instantly slamming `hw.set_motors(test_power, test_power)` from 0 to 40+.
5. **Audit Control Loops:**
   - In `src/balance_bot/reflex/balance_core.py`, find the backlash compensation hack (`kick_power = 40.0 * current_sign`) and ensure it's removed or isolated, removing `-1` multipliers that are bypassing `RobotHardware`.
   - Ensure all sign logic is contained to `RobotHardware.set_motors`.
6. **Audit Exception Handling:**
   - Search for `except OSError` in `src/balance_bot/hardware/piconzero.py` and `src/balance_bot/hardware/robot_hardware.py` and strictly ensure they explicitly call `stop()` or write 0 to PWM lines before the program crashes out.
