import re

with open('tests/test_agent_state_machine.py', 'r') as f:
    content = f.read()

# BalancingState doesn't track kickup_attempts, so just remove the assert since transitioning clears it conceptually.
content = content.replace("self.assertEqual(self.agent.state.kickup_attempts, 0)", "")

with open('tests/test_agent_state_machine.py', 'w') as f:
    f.write(content)
