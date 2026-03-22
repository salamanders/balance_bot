with open('tests/test_agent_state_machine.py', 'r') as f:
    content = f.read()

content = content.replace("            self.assertEqual(self.agent.state.kickup_attempts, 1)", "        self.assertEqual(self.agent.state.kickup_attempts, 1)")

with open('tests/test_agent_state_machine.py', 'w') as f:
    f.write(content)
