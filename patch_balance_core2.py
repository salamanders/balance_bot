import re

with open('src/balance_bot/reflex/balance_core.py', 'r') as f:
    content = f.read()

# Remove state variables from init
replacement_init = """        self.pitch = 0.0"""
content = re.sub(r'        self\.pitch = 0\.0\n        self\.last_motor_sign = 1\n        self\.backlash_timer = 0\.0', replacement_init, content)

# Remove backlash compensation
replacement_update = """        # 9. Actuate
        # Apply Battery Compensation
        if battery_compensation > 0:
            left_motor /= battery_compensation
            right_motor /= battery_compensation

        self.hw.set_motors(left_motor, right_motor)"""

content = re.sub(r'        # 8a\. Backlash Compensation\n        current_sign = 1 if pid_output > 0 else -1\n\n        if current_sign != self\.last_motor_sign and abs\(pid_output\) > 2\.0:\n            # We just crossed zero! Start the slop-clearing timer\n            self\.backlash_timer = self\.learning_state\.control\.backlash_pulse_time\n            self\.last_motor_sign = current_sign\n\n        if self\.backlash_timer > 0:\n            # While in the dead-zone, inject a "Kick" to skip the slop\.\n            # We use a higher power \(e\.g\., 40\) to traverse it quickly\.\n            kick_power = 40\.0 \* current_sign\n            left_motor = kick_power\n            right_motor = kick_power\n            self\.backlash_timer -= loop_delta_time\n\n        # 9\. Actuate\n        # Apply Battery Compensation\n        if battery_compensation > 0:\n            left_motor /= battery_compensation\n            right_motor /= battery_compensation\n\n        self\.hw\.set_motors\(left_motor, right_motor\)', replacement_update, content)

with open('src/balance_bot/reflex/balance_core.py', 'w') as f:
    f.write(content)
