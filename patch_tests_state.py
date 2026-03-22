import re

with open('tests/test_agent_state_machine.py', 'r') as f:
    content = f.read()

# The test uses Enum values, we need to adapt it.
content = content.replace("self.agent.state = BotState.IDLE", "from balance_bot.behavior.states import IdleState, KickupState, BalancingState, CrashedState; self.agent.state = IdleState()")
content = content.replace("self.assertEqual(self.agent.state, BotState.BALANCING)", "self.assertIsInstance(self.agent.state, BalancingState)")
content = content.replace("self.assertEqual(self.agent.state, BotState.KICKUP)", "self.assertIsInstance(self.agent.state, KickupState)")

content = content.replace("self.agent.state = BotState.KICKUP", "from balance_bot.behavior.states import KickupState; self.agent.state = KickupState()")
content = content.replace("self.assertEqual(self.agent.state, BotState.IDLE)", "from balance_bot.behavior.states import IdleState; self.assertIsInstance(self.agent.state, IdleState)")
content = content.replace("self.agent.state = BotState.BALANCING", "from balance_bot.behavior.states import BalancingState; self.agent.state = BalancingState()")
content = content.replace("self.assertEqual(self.agent.state, BotState.CRASHED)", "from balance_bot.behavior.states import CrashedState; self.assertIsInstance(self.agent.state, CrashedState)")
content = content.replace("self.agent.state = BotState.CRASHED", "from balance_bot.behavior.states import CrashedState; self.agent.state = CrashedState()")

with open('tests/test_agent_state_machine.py', 'w') as f:
    f.write(content)
