import re

with open('tests/test_agent_state_machine.py', 'r') as f:
    content = f.read()

# Fix remaining kickup asserts.
# agent.kickup_attempts does not exist anymore. IdleState tracks it.
content = content.replace("self.assertEqual(self.agent.kickup_attempts, 1)", "self.assertEqual(self.agent.state.kickup_attempts, 1)")
content = content.replace("self.assertEqual(self.agent.kickup_attempts, 0)", "self.assertEqual(self.agent.state.kickup_attempts, 0)")
content = content.replace("self.assertIsInstance(self.agent.state, BalancingState)", "from balance_bot.behavior.states import BalancingState; self.assertIsInstance(self.agent.state, BalancingState)")

with open('tests/test_agent_state_machine.py', 'w') as f:
    f.write(content)
