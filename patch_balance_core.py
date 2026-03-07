import re

with open('src/balance_bot/reflex/balance_core.py', 'r') as f:
    content = f.read()

# Add import
if 'circular_difference' not in content:
    content = content.replace('from ..utils import ComplementaryFilter', 'from ..utils import ComplementaryFilter, circular_difference')

# Replace the error calculation
replacement = """        # 7. Calculate Control Output
        error = -circular_difference(target_angle, self.pitch)"""

content = re.sub(r'        # 7\. Calculate Control Output\n        error = self\.pitch - target_angle', replacement, content)

with open('src/balance_bot/reflex/balance_core.py', 'w') as f:
    f.write(content)
