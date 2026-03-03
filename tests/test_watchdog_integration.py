import pytest
from unittest.mock import MagicMock
from balance_bot.behavior.agent import Agent
from balance_bot.watchdog import SurvivalWatchdog

def test_agent_watchdog_panic():
    watchdog = MagicMock(spec=SurvivalWatchdog)
    # Simulate that the watchdog triggered the interrupt
    watchdog.triggered = True

    agent = Agent(watchdog=watchdog)

    # Mock core to raise KeyboardInterrupt immediately
    agent.core = MagicMock()
    agent.core.pitch = 0.0
    agent.core.update.side_effect = KeyboardInterrupt

    # Skip warmup
    agent.learning_state.timing.setup_wait = 0
    agent.running = True

    # Expect RuntimeError "Watchdog Panic"
    with pytest.raises(RuntimeError, match="Watchdog Panic"):
        agent.run()

def test_agent_keyboard_interrupt_clean_exit():
    watchdog = MagicMock(spec=SurvivalWatchdog)
    # Watchdog did NOT trigger it (User pressed Ctrl+C)
    watchdog.triggered = False

    agent = Agent(watchdog=watchdog)

    agent.core = MagicMock()
    agent.core.pitch = 0.0
    agent.core.update.side_effect = KeyboardInterrupt

    agent.learning_state.timing.setup_wait = 0
    agent.running = True

    # Should NOT raise RuntimeError, should catch and exit gracefully
    try:
        agent.run()
    except RuntimeError:
        pytest.fail("Agent raised RuntimeError for normal KeyboardInterrupt")
    except Exception as e:
        pytest.fail(f"Agent raised unexpected exception: {e}")
