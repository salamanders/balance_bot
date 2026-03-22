import re

with open('tests/test_agent_state_machine.py', 'r') as f:
    content = f.read()

# Fix crashed state time check logic.
# The CrashedState stores self.crash_time in its own instance.
content = content.replace("self.agent.last_crash_time = time.monotonic() - 3.0", "self.agent.state.crash_time = time.monotonic() - 4.0")

# Fix _incremental_kickup mock since it's now on KickupState, not Agent.
content = content.replace("with patch.object(self.agent, '_incremental_kickup'", "with patch.object(self.agent.state, '_incremental_kickup'")

with open('tests/test_agent_state_machine.py', 'w') as f:
    f.write(content)
