from typing import Any
from unittest.mock import patch
from balance_bot.watchdog import SurvivalWatchdog

def test_watchdog_triggers_on_timeout() -> None:
    """Test that the watchdog triggers when time exceeds timeout."""
    # monotonic calls:
    # 1. __init__ sets last_heartbeat
    # 2. _watch checks time
    time_values = [0.0, 20.0]
    def fake_monotonic() -> Any:
        if time_values:
            return time_values.pop(0)
        return 20.0

    with patch('balance_bot.watchdog.time.monotonic', side_effect=fake_monotonic), \
         patch('balance_bot.watchdog.time.sleep'), \
         patch('balance_bot.watchdog._thread.interrupt_main') as mock_interrupt:

        watchdog = SurvivalWatchdog(timeout=15.0)
        watchdog.thread.join(timeout=1.0)

        assert watchdog.triggered is True
        mock_interrupt.assert_called_once()

def test_watchdog_does_not_trigger_when_heartbeat_called() -> None:
    """Test that heartbeat resets the timer and prevents trigger."""
    # monotonic calls:
    # 1. __init__ sets last_heartbeat = 0.0
    # 2. heartbeat sets last_heartbeat = 10.0
    # 3. _watch checks time = 11.0 (11.0 - 10.0 < 15.0)
    time_values = [0.0, 10.0, 11.0]
    def fake_monotonic() -> Any:
        if time_values:
            return time_values.pop(0)
        return 11.0

    with patch('balance_bot.watchdog.time.monotonic', side_effect=fake_monotonic), \
         patch('balance_bot.watchdog.time.sleep') as mock_sleep, \
         patch('balance_bot.watchdog._thread.interrupt_main') as mock_interrupt:

        # We need a reference to watchdog inside the sleep mock.
        # So we use a closure correctly.
        watchdog = None
        def fake_sleep(sec: Any) -> Any:
            if watchdog is not None:
                watchdog.stop()

        mock_sleep.side_effect = fake_sleep

        watchdog = SurvivalWatchdog(timeout=15.0)
        watchdog.heartbeat()

        watchdog.thread.join(timeout=1.0)

        assert watchdog.triggered is False
        mock_interrupt.assert_not_called()

def test_watchdog_stop() -> None:
    """Test that stop() exits the thread gracefully."""
    with patch('balance_bot.watchdog.time.monotonic', return_value=0.0), \
         patch('balance_bot.watchdog.time.sleep') as mock_sleep, \
         patch('balance_bot.watchdog._thread.interrupt_main') as mock_interrupt:

        watchdog = None
        def fake_sleep(sec: Any) -> Any:
            if watchdog is not None:
                watchdog.stop()

        mock_sleep.side_effect = fake_sleep

        watchdog = SurvivalWatchdog(timeout=15.0)
        watchdog.thread.join(timeout=1.0)

        assert watchdog.running is False
        assert watchdog.triggered is False
        mock_interrupt.assert_not_called()
