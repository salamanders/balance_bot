from typing import Any
from unittest.mock import MagicMock, patch

import pytest

from balance_bot.behavior.agent import Agent
from balance_bot.watchdog import SurvivalWatchdog


@patch("balance_bot.behavior.agent.BalanceCore")
def test_agent_watchdog_panic(_mock_balance_core: Any) -> None:
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


@patch("balance_bot.behavior.agent.BalanceCore")
def test_agent_keyboard_interrupt_clean_exit(_mock_balance_core: Any) -> None:
    watchdog = MagicMock(spec=SurvivalWatchdog)
    # Watchdog did NOT trigger it (User pressed Ctrl+C)
    watchdog.triggered = False

    agent = Agent(watchdog=watchdog)

    agent.core = MagicMock()
    agent.core.pitch = 0.0

    agent.blackbox = MagicMock()
    agent.led = MagicMock()
    agent.io_executor = MagicMock()
    agent.learning_state = MagicMock()

    # We want it to try to save if it was dirty
    agent.config_dirty = True

    agent.learning_state.timing.setup_wait = 0
    agent.running = True

    # Inject KeyboardInterrupt in the main loop from RateLimiter.sleep
    with patch("balance_bot.behavior.agent.RateLimiter.sleep", side_effect=KeyboardInterrupt):
        # Should NOT raise RuntimeError, should catch and exit gracefully
        try:
            agent.run()
        except RuntimeError:
            pytest.fail("Agent raised RuntimeError for normal KeyboardInterrupt")
        except Exception as e:
            pytest.fail(f"Agent raised unexpected exception: {e}")

    # Verify clean exit calls in finally block
    agent.blackbox.stop.assert_called_once()
    agent.core.cleanup.assert_called_once()
    agent.led.signal_off.assert_called_once()
    agent.learning_state.save.assert_called_once()
    agent.io_executor.shutdown.assert_called_once_with(wait=True)
