1. **Update `AGENTS.md`**
   - Incorporate the new core directives into the `AGENTS.md` file permanently.
2. **Audit HAL (`robot_hardware.py`, `piconzero.py`)**
   - Ensure missing hardware fails loudly.
   - Look for silent error handling of IOError/OSError.
3. **Audit Math (`utils.py`, `pid.py`)**
   - Implement a function in `utils.py` for shortest-path circular difference (handling 180/-180 wrap-around).
   - Use this new circular difference function in `pid.py` when calculating `error = self.pitch - target_angle`.
4. **Audit Motor Commands (`steps.py`)**
   - Identify instances in `steps.py` (e.g. `DeriveKinematicsStep._pulse_and_measure`, `MechanicalBacklashStep`, `KickupDynamicsStep._attempt_kick`) where high instantaneous motor power is commanded. Add power ramping (defense in depth).
5. **Audit Control Loops (`balance_core.py`)**
   - Verify multipliers or sign inversions (e.g. `current_sign != self.last_motor_sign`, `kick_power = 40.0 * current_sign`) aren't bypassing standard control.
6. **Audit Exception Handling**
   - Search for `try/except OSError` blocks that just `pass` or log without disarming motors (especially in `piconzero.py`, `robot_hardware.py`, `steps.py`). Fix them to halt the system and explicitly try to disarm actuators if possible.
7. **Complete pre commit steps to ensure proper testing, verification, review, and reflection are done.**
8. **Submit changes**
