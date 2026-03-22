with open('src/balance_bot/behavior/agent.py', 'r') as f:
    content = f.read()
content = content.replace("if type(next_state) != type(self.state):", "if type(next_state) is not type(self.state):")
with open('src/balance_bot/behavior/agent.py', 'w') as f:
    f.write(content)
