import re

with open('src/balance_bot/discovery/steps.py', 'r') as f:
    content = f.read()

# 1. _pulse_and_measure
replacement1 = """    def _pulse_and_measure(self, hw: RobotHardware, l_p: float, r_p: float, name: str) -> Tuple[glm.vec3, glm.vec3]:
        \"\"\"Pulse motors and return average gyro and accel vectors.\"\"\"
        print(f"  Pulsing {name}...")

        # Ramp up power
        steps = []
        ramp_duration = 0.1
        ramp_steps = 5
        for i in range(1, ramp_steps + 1):
            factor = i / ramp_steps
            steps.append((l_p * factor, r_p * factor, ramp_duration / ramp_steps))

        # Hold
        steps.append((l_p, r_p, 0.4 - ramp_duration))

        res = hw.execute_maneuver(steps)
        time.sleep(1.0) # Settle"""

content = re.sub(r'    def _pulse_and_measure\(self, hw: RobotHardware, l_p: float, r_p: float, name: str\) -> Tuple\[glm\.vec3, glm\.vec3\]:\n        \"\"\"Pulse motors and return average gyro and accel vectors\.\"\"\"\n        print\(f"  Pulsing \{name\}\.\.\."\)\n        res = hw\.drive_and_measure\(l_p, r_p, 0\.4, wait_for_stability=False\)\n        time\.sleep\(1\.0\) # Settle', replacement1, content)

# 2. MechanicalBacklashStep
replacement2 = """        # Forward Ramp
        steps_fwd = []
        ramp_steps = 5
        for i in range(1, ramp_steps + 1):
            factor = i / ramp_steps
            steps_fwd.append((test_power * factor, test_power * factor, 0.05))
        steps_fwd.append((test_power, test_power, 0.3))
        hw.execute_maneuver(steps_fwd)"""

content = re.sub(r'        # Forward\n        hw\.set_motors\(test_power, test_power\)\n        time\.sleep\(0\.3\)', replacement2, content)

# 3. _attempt_kick
replacement3 = """    def _attempt_kick(self, hw: RobotHardware, start_sign: float, p: float) -> str:
        \"\"\"Helper to attempt a kick-up maneuver. start_sign: +1 for BACK, -1 for FRONT.\"\"\"
        kick_sign = -start_sign
        setup_sign = -kick_sign

        setup_p = p * setup_sign * 0.7
        kick_p = p * kick_sign * 1.0

        # Ramp setup power
        steps = []
        ramp_steps = 5
        for i in range(1, ramp_steps + 1):
            factor = i / ramp_steps
            steps.append((setup_p * factor, setup_p * factor, 0.05))

        # Setup hold
        steps.append((setup_p, setup_p, 0.3 - 0.25))

        # Ramp kick power (fast)
        for i in range(1, 3):
            factor = i / 2.0
            steps.append((kick_p * factor, kick_p * factor, 0.02))

        # Kick hold
        steps.append((kick_p, kick_p, 0.4 - 0.04))

        hw.execute_maneuver(steps)"""

content = re.sub(r'    def _attempt_kick\(self, hw: RobotHardware, start_sign: float, p: float\) -> str:\n        \"\"\"Helper to attempt a kick-up maneuver\. start_sign: \+1 for BACK, -1 for FRONT\.\"\"\"\n        kick_sign = -start_sign\n        setup_sign = -kick_sign\n\n        setup_p = p \* setup_sign \* 0\.7\n        kick_p = p \* kick_sign \* 1\.0\n\n        hw\.execute_maneuver\(\[\n            \(setup_p, setup_p, 0\.3\),\n            \(kick_p, kick_p, 0\.4\)\n        \]\)', replacement3, content)

# 4. _force_posture
replacement4 = """            print(f"  Flop to {target_name} (Power {p:.0f})...")
            motor_p = p * target_sign

            # Ramp
            steps = []
            ramp_steps = 5
            for i in range(1, ramp_steps + 1):
                factor = i / ramp_steps
                steps.append((motor_p * factor, motor_p * factor, 0.05))
            steps.append((motor_p, motor_p, 0.4 - 0.25))
            hw.execute_maneuver(steps)
            hw.wait_for_stability(1.0)"""

content = re.sub(r'            print\(f"  Flop to \{target_name\} \(Power \{p:\.0f\}\)\.\.\."\)\n            motor_p = p \* target_sign\n\n            hw\.drive_and_measure\(motor_p, motor_p, 0\.4\)\n            hw\.wait_for_stability\(1\.0\)', replacement4, content)

with open('src/balance_bot/discovery/steps.py', 'w') as f:
    f.write(content)
