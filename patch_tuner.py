import re

with open('src/balance_bot/behavior/states.py', 'r') as f:
    content = f.read()

content = content.replace("from ..adaptation.tuner import PIDTuner", "from ..adaptation.tuner import ContinuousTuner")
content = content.replace("tuner: PIDTuner", "tuner: ContinuousTuner")

with open('src/balance_bot/behavior/states.py', 'w') as f:
    f.write(content)
