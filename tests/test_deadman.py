"""
Unit tests for HTTP Deadman's Switch and Watchdog safety integrations.
"""

import time
from typing import Any
from unittest.mock import MagicMock, patch

from balance_bot.configuration import HardwareConfig, LearningState, PIDParams
from balance_bot.deadman import DeadmanServer
from balance_bot.hardware.robot_hardware import RobotHardware
from balance_bot.watchdog import SurvivalWatchdog


def test_deadman_initial_state() -> None:
    """Verify deadman server initializes in a disarmed state."""
    server = DeadmanServer(port=0)  # OS-assigned port
    assert not server.is_alive()
    assert not server.is_armed
    assert not server.estop_triggered


def test_deadman_heartbeat_and_timeout() -> None:
    """Verify heartbeat arms deadman switch and times out after threshold."""
    server = DeadmanServer(port=0, timeout=0.5)

    # Receive heartbeat -> should be alive
    server.receive_heartbeat()
    assert server.is_armed
    assert server.is_alive()

    # Wait for timeout -> should expire
    time.sleep(0.6)
    assert not server.is_alive()


def test_deadman_explicit_release() -> None:
    """Verify explicit release immediately disarms deadman."""
    server = DeadmanServer(port=0, timeout=5.0)
    server.receive_heartbeat()
    assert server.is_alive()

    server.release()
    assert not server.is_alive()
    assert not server.is_armed


def test_deadman_estop() -> None:
    """Verify emergency stop disarms deadman and blocks future heartbeats until reset."""
    server = DeadmanServer(port=0, timeout=5.0)
    server.receive_heartbeat()
    assert server.is_alive()

    server.estop()
    assert not server.is_alive()
    assert server.estop_triggered

    # Further heartbeats should be ignored while E-Stopped
    server.receive_heartbeat()
    assert not server.is_alive()

    # Reset E-Stop
    server.reset_estop()
    assert not server.estop_triggered
    assert not server.is_alive()

    # Now heartbeat should work again
    server.receive_heartbeat()
    assert server.is_alive()


def test_deadman_status_dict() -> None:
    """Verify status dictionary formats telemetry correctly."""
    server = DeadmanServer(port=0, timeout=2.0)
    server.posture_provider = lambda: "BALANCING"
    server.pitch_provider = lambda: 1.234

    status = server.get_status_dict()
    assert status["posture"] == "BALANCING"
    assert status["pitch"] == 1.234
    assert not status["is_alive"]


def test_watchdog_experiment_timeout() -> None:
    """Verify SurvivalWatchdog triggers interrupt when experiment budget is exceeded."""
    with patch("_thread.interrupt_main") as mock_interrupt:
        watchdog = SurvivalWatchdog(timeout=10.0, experiment_duration=0.3)
        time.sleep(0.5)
        mock_interrupt.assert_called_once()
        watchdog.stop()


def test_watchdog_estop_trigger() -> None:
    """Verify SurvivalWatchdog triggers interrupt when Deadman E-Stop is engaged."""
    deadman = MagicMock()
    deadman.estop_triggered = False

    with patch("_thread.interrupt_main") as mock_interrupt:
        watchdog = SurvivalWatchdog(timeout=10.0, deadman_server=deadman)
        time.sleep(0.2)
        assert not watchdog.triggered

        # Trigger E-Stop
        deadman.estop_triggered = True
        time.sleep(0.4)
        mock_interrupt.assert_called_once()
        watchdog.stop()


def test_robot_hardware_disarms_on_deadman_inactive(monkeypatch: Any) -> None:
    """Verify RobotHardware.set_motors enforces (0, 0) output when deadman is inactive."""
    monkeypatch.setenv("ALLOW_MOCK_FALLBACK", "1")

    deadman = MagicMock()
    deadman.is_alive.return_value = False

    hw_config = HardwareConfig(motor_l=0, motor_r=1)
    learning_state = LearningState(pid=PIDParams())

    hw = RobotHardware(hw_config, learning_state, deadman_server=deadman)
    hw.pz = MagicMock()

    # Attempt to command forward drive
    hw.set_motors(50.0, 50.0)

    # Hardware driver should have received (0, 0) disarm command
    hw.pz.set_motors.assert_called_with(0, 0)
