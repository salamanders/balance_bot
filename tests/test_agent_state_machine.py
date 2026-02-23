import unittest
from unittest.mock import MagicMock, patch, PropertyMock
import time
from src.balance_bot.behavior.agent import Agent, BotState, HardwareConfig

class TestAgentStateMachine(unittest.TestCase):
    def setUp(self):
        # Patch dependencies before Agent init
        self.config_patcher = patch('src.balance_bot.behavior.agent.HardwareConfig')
        self.core_patcher = patch('src.balance_bot.behavior.agent.BalanceCore')
        self.tuner_patcher = patch('src.balance_bot.behavior.agent.ContinuousTuner')
        self.finder_patcher = patch('src.balance_bot.behavior.agent.BalancePointFinder')
        self.battery_patcher = patch('src.balance_bot.behavior.agent.BatteryEstimator')
        self.recovery_patcher = patch('src.balance_bot.behavior.agent.RecoveryManager')
        self.led_patcher = patch('src.balance_bot.behavior.agent.LedController')
        self.setup_logging_patcher = patch('src.balance_bot.behavior.agent.setup_logging')
        self.check_force_calib_patcher = patch('src.balance_bot.behavior.agent.check_force_calibration_flag')

        self.mock_config_cls = self.config_patcher.start()
        self.mock_core_cls = self.core_patcher.start()
        self.mock_tuner_cls = self.tuner_patcher.start()
        self.mock_finder_cls = self.finder_patcher.start()
        self.mock_battery_cls = self.battery_patcher.start()
        self.mock_recovery_cls = self.recovery_patcher.start()
        self.mock_led_cls = self.led_patcher.start()
        self.setup_logging_patcher.start()
        self.mock_check_force = self.check_force_calib_patcher.start()

        self.mock_check_force.return_value = False

        # Mock Config Instance
        self.mock_config = MagicMock()
        self.mock_config.loop_time = 0.01
        self.mock_config.timing.setup_wait = 0.0
        self.mock_config.timing.battery_log_interval = 100
        self.mock_config.timing.tuning_log_interval = 100
        self.mock_config.pid.target_angle = 0.0
        self.mock_config.pid.kp = 1.0
        self.mock_config.pid.ki = 0.0
        self.mock_config.pid.kd = 0.0
        self.mock_config.crash_angle = 50.0
        self.mock_config.control.kickup_power_forward = 50.0
        self.mock_config.control.kickup_power_backward = 50.0
        self.mock_config.control.low_battery_log_threshold = 0.5

        self.mock_config_cls.load.return_value = self.mock_config

        self.agent = Agent()

        # Configure Tuner Mock return value
        self.agent.tuner.get_current_scale.return_value = 1.0

        # Stop the infinite loop by default
        self.agent.running = False

    def tearDown(self):
        self.config_patcher.stop()
        self.core_patcher.stop()
        self.tuner_patcher.stop()
        self.finder_patcher.stop()
        self.battery_patcher.stop()
        self.recovery_patcher.stop()
        self.led_patcher.stop()
        self.setup_logging_patcher.stop()
        self.check_force_calib_patcher.stop()

    def run_agent_once(self):
        """Run the agent loop exactly once."""
        self.agent.running = True

        # We patch RateLimiter.sleep to stop the loop after one call
        with patch('src.balance_bot.behavior.agent.RateLimiter') as mock_rate:
            mock_rate_instance = mock_rate.return_value
            def stop_loop():
                self.agent.running = False
                return 0.01
            mock_rate_instance.sleep.side_effect = stop_loop

            # We also need to mock core.update to return a dummy telemetry
            mock_telemetry = MagicMock()
            mock_telemetry.pitch_angle = 0.0
            mock_telemetry.pitch_rate = 0.0
            mock_telemetry.motor_output = 0.0
            mock_telemetry.crashed = False
            self.agent.core.update.return_value = mock_telemetry

            self.agent.run()
            return mock_telemetry

    def test_idle_to_balancing(self):
        """Test transition from IDLE to BALANCING when upright."""
        self.agent.state = BotState.IDLE
        # Mock pitch to be 5.0 (Upright)
        type(self.agent.core).pitch = PropertyMock(return_value=5.0)

        self.run_agent_once()

        self.assertEqual(self.agent.state, BotState.BALANCING)
        self.assertEqual(self.agent.kickup_attempts, 0)

    def test_idle_to_kickup(self):
        """Test transition from IDLE to KICKUP when resting."""
        self.agent.state = BotState.IDLE
        # Mock pitch to be 60.0 (Resting)
        type(self.agent.core).pitch = PropertyMock(return_value=60.0)
        self.agent.kickup_attempts = 0

        self.run_agent_once()

        self.assertEqual(self.agent.state, BotState.KICKUP)

    def test_kickup_success(self):
        """Test KICKUP state executes sequence and transitions to BALANCING on success."""
        self.agent.state = BotState.KICKUP
        # Mock pitch for direction check (e.g. Back)
        type(self.agent.core).pitch = PropertyMock(return_value=-60.0)

        with patch.object(self.agent, '_incremental_kickup', return_value=True) as mock_kick:
            self.run_agent_once()

            mock_kick.assert_called()
            self.assertEqual(self.agent.state, BotState.BALANCING)
            self.assertEqual(self.agent.kickup_attempts, 0)

    def test_kickup_failure(self):
        """Test KICKUP state transitions to IDLE on failure."""
        self.agent.state = BotState.KICKUP
        self.agent.kickup_attempts = 0
        # Mock pitch for direction check
        type(self.agent.core).pitch = PropertyMock(return_value=-60.0)

        with patch.object(self.agent, '_incremental_kickup', return_value=False) as mock_kick:
            self.run_agent_once()

            mock_kick.assert_called()
            self.assertEqual(self.agent.state, BotState.IDLE)
            self.assertEqual(self.agent.kickup_attempts, 1)

    def test_balancing_to_crashed(self):
        """Test BALANCING transitions to CRASHED if pitch exceeds limit."""
        self.agent.state = BotState.BALANCING

        # Last telemetry needs to be set (or the first loop iteration logic needs to be robust)
        # In the first loop, last_telemetry is None.
        # But core.update returns telemetry at end of loop.
        # The transition check happens at START of loop using last_telemetry.

        # So we need to simulate TWO iterations?
        # Or mock core.pitch (fallback if last_telemetry is None).
        # The code uses: current_pitch = last_telemetry.pitch_angle if last_telemetry else self.core.pitch

        type(self.agent.core).pitch = PropertyMock(return_value=60.0) # > 50.0 crash angle

        self.run_agent_once()

        self.assertEqual(self.agent.state, BotState.CRASHED)
        self.agent.core.hw.stop.assert_called()

    def test_crashed_timeout(self):
        """Test CRASHED transitions to IDLE after timeout."""
        self.agent.state = BotState.CRASHED
        self.agent.last_crash_time = time.monotonic() - 3.0 # 3 seconds ago

        self.run_agent_once()

        self.assertEqual(self.agent.state, BotState.IDLE)

if __name__ == '__main__':
    unittest.main()
