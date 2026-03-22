import re

with open('src/balance_bot/behavior/states.py', 'r') as f:
    content = f.read()

content = content.replace("from ..adaptation.recovery import CrashRecovery", "from ..adaptation.recovery import RecoveryManager")
content = content.replace("recovery: CrashRecovery", "recovery: RecoveryManager")

with open('src/balance_bot/behavior/states.py', 'w') as f:
    f.write(content)
