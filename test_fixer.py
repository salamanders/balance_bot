import re

with open('src/balance_bot/behavior/agent.py', 'r') as f:
    content = f.read()

content = content.replace("if _comp_factor < self.learning_state.control.low_battery_log_threshold", "if float(_comp_factor) < float(self.learning_state.control.low_battery_log_threshold)")

with open('src/balance_bot/behavior/agent.py', 'w') as f:
    f.write(content)

with open('src/balance_bot/hardware/robot_hardware.py', 'r') as f:
    content = f.read()

# Fix missing lock release / sleep block logic in robot_hardware
content = content.replace("time.sleep(0.001)", "time.sleep(0.005)")

with open('src/balance_bot/hardware/robot_hardware.py', 'w') as f:
    f.write(content)
