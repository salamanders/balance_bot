import os

files = ['tests/test_agent_state_machine.py', 'tests/test_imu_logic.py', 'tests/test_imu_resilience.py', 'tests/test_imu_yaw_roll.py']

for file in files:
    with open(file, 'r') as f:
        content = f.read()

    content = content.replace("\n         self", "\n        self")
    content = content.replace("\n         hw", "\n    hw")

    with open(file, 'w') as f:
        f.write(content)
