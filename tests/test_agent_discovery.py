import unittest
from unittest.mock import MagicMock, patch, call
import sys
import os

# Adjust path to import src
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../src")))

from balance_bot.behavior.agent import Agent
from balance_bot.enums import Orientation

class TestAgentDiscovery(unittest.TestCase):

    def setUp(self):
        # Patch dependencies that Agent.__init__ uses
        self.patches = [
            patch("balance_bot.behavior.agent.setup_logging"),
            patch("balance_bot.behavior.agent.CONFIG_FILE"),
            patch("balance_bot.behavior.agent.RobotConfig"),
            patch("balance_bot.behavior.agent.BalanceCore"),
            patch("balance_bot.behavior.agent.ContinuousTuner"),
            patch("balance_bot.behavior.agent.BalancePointFinder"),
            patch("balance_bot.behavior.agent.BatteryEstimator"),
            patch("balance_bot.behavior.agent.RecoveryManager"),
            patch("balance_bot.behavior.agent.LedController"),
        ]

        for p in self.patches:
            p.start()

    def tearDown(self):
        for p in self.patches:
            p.stop()

    def test_perform_discovery_success_back_start(self):
        """Test a successful discovery sequence starting from BACK."""
        # Arrange
        agent = Agent()

        # Mock internal methods
        agent._wait_for_settle = MagicMock()
        agent._measure_stable_angle = MagicMock(side_effect=[-40.0, 40.0]) # Back angle, Front angle
        agent._incremental_flop = MagicMock(side_effect=[55.0, 65.0]) # Power1 (Back->Front), Power2 (Front->Back)
        agent._incremental_kickup = MagicMock()

        # Mock initial state (Pitch < -5.0 means BACK start)
        agent.core.pitch = -10.0

        # Act
        agent._perform_discovery()

        # Assert
        # 1. Start side detection
        # Logic: -10 pitch -> BACK start.

        # 2. Measure Limits
        # First measurement is start side (Back) -> -40.0
        # Second measurement is other side (Front) -> 40.0
        self.assertEqual(agent._measure_stable_angle.call_count, 2)

        # 3. Flops
        # First flop: Back -> Front (Other side). Power 55.0
        # Second flop: Front -> Back (Start side). Power 65.0
        self.assertEqual(agent._incremental_flop.call_count, 2)
        agent._incremental_flop.assert_has_calls([
            call(target_side=Orientation.FRONT),
            call(target_side=Orientation.BACK)
        ])

        # 4. Config Update
        # Midpoint = (-40 + 40) / 2 = 0.0
        self.assertEqual(agent.config.pid.target_angle, 0.0)

        # Powers:
        # Start=BACK.
        # Power1 (55.0) was Back->Front (Forward direction).
        # Power2 (65.0) was Front->Back (Backward direction).
        self.assertEqual(agent.config.control.kickup_power_forward, 55.0)
        self.assertEqual(agent.config.control.kickup_power_backward, 65.0)

        # 5. Kickup
        # Should use Power1 (55.0) because we are back at start (BACK).
        agent._incremental_kickup.assert_called_once_with(target_angle=0.0, start_power=55.0)

        # 6. Flags
        self.assertTrue(agent.config_dirty)
        self.assertFalse(agent.first_run)

    def test_perform_discovery_success_front_start(self):
        """Test a successful discovery sequence starting from FRONT."""
        # Arrange
        agent = Agent()

        # Mock internal methods
        agent._wait_for_settle = MagicMock()
        agent._measure_stable_angle = MagicMock(side_effect=[40.0, -40.0]) # Front angle, Back angle
        agent._incremental_flop = MagicMock(side_effect=[65.0, 55.0]) # Power1 (Front->Back), Power2 (Back->Front)
        agent._incremental_kickup = MagicMock()

        # Mock initial state (Pitch > 5.0 means FRONT start)
        agent.core.pitch = 10.0

        # Act
        agent._perform_discovery()

        # Assert
        # 1. Start side detection -> FRONT

        # 2. Measure Limits
        # First measurement is start side (Front) -> 40.0
        # Second measurement is other side (Back) -> -40.0

        # 3. Flops
        # First flop: Front -> Back (Other side). Power 65.0
        # Second flop: Back -> Front (Start side). Power 55.0
        self.assertEqual(agent._incremental_flop.call_count, 2)
        agent._incremental_flop.assert_has_calls([
            call(target_side=Orientation.BACK),
            call(target_side=Orientation.FRONT)
        ])

        # 4. Config Update
        # Midpoint = (40 + -40) / 2 = 0.0
        self.assertEqual(agent.config.pid.target_angle, 0.0)

        # Powers:
        # Start=FRONT.
        # Power1 (65.0) was Front->Back (Backward direction).
        # Power2 (55.0) was Back->Front (Forward direction).
        self.assertEqual(agent.config.control.kickup_power_backward, 65.0)
        self.assertEqual(agent.config.control.kickup_power_forward, 55.0)

        # 5. Kickup
        # Should use Power1 (65.0) because we are back at start (FRONT).
        agent._incremental_kickup.assert_called_once_with(target_angle=0.0, start_power=65.0)

if __name__ == "__main__":
    unittest.main()
