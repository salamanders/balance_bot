1. **AGENTS.md Update:**
   - Modify `AGENTS.md` to include the specific directives provided: Hardware Reality over Assumptions, Defense in Depth & I2C Brownouts, Prohibition on Hallucinated Physics, Isolate Inversions, and Strict Personal Operational Constraints.
2. **Audit & Refactor - HAL (Hardware Abstraction Layer):**
   - Check `RobotHardware` (`src/balance_bot/hardware/robot_hardware.py`) and specific driver files. Ensure it fails loudly when required hardware or configuration is missing.
   - Specifically check for any `try/except` that swallows `IOError` or `OSError` (Errno 5) without disarming/stopping actuators.
3. **Audit & Refactor - Math (`utils.py` and `pid.py`):**
   - Check pitch difference calculations in `utils.py` and `pid.py`. Use shortest-path circular difference to handle the 180/-180 degree boundary safely instead of naive subtraction.
4. **Audit & Refactor - Motor Commands (`steps.py` and `robot_hardware.py`):**
   - Identify instantaneous power spikes (like > 30-40%). Replace them with ramping loops to avoid I2C brownouts.
5. **Audit & Refactor - Control Loops (`balance_core.py`):**
   - Ensure `-1` multipliers and orientation hacks are isolated to `robot_hardware.py` and not present in Tier 1 high-frequency reflex loop (`balance_core.py`).
6. **Audit & Refactor - Exception Handling:**
   - Search for `try/except` swallowing `IOError` or `OSError`. Make sure they trigger system halt and explicit motor stop.
