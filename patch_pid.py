import re

with open('src/balance_bot/reflex/pid.py', 'r') as f:
    content = f.read()

# Add import
if 'circular_difference' not in content:
    content = content.replace('from ..utils import clamp', 'from ..utils import clamp, circular_difference')

# Replace input_val
replacement = """        # simple-pid expects the "Input" (Process Variable), not the Error.
        # Since Error = Setpoint - Input, we have Input = Setpoint - Error.
        input_val = self.pid.setpoint - error"""

content = re.sub(r'        # simple-pid expects the "Input" \(Process Variable\), not the Error\.\n        # Since Error = Setpoint - Input, we have Input = Setpoint - Error\.\n        input_val = self\.pid\.setpoint - error', replacement, content)

with open('src/balance_bot/reflex/pid.py', 'w') as f:
    f.write(content)
