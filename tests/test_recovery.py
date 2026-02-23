import unittest
from unittest.mock import MagicMock
from balance_bot.adaptation.recovery import RecoveryManager
from balance_bot.configuration import ControlConfig, STARTUP_RAMP_SPEED

class TestRecoveryManager(unittest.TestCase):
    def setUp(self):
        # Default configuration
        self.recovery = RecoveryManager()

    def test_startup_normal_kp_high_lean(self):
        """Test that high Kp (>1.0) and high lean (>5.0) triggers recovery."""
        # Initial state: Not recovering
        self.assertFalse(self.recovery.recovering)

        # Call with conditions meeting the hardcoded threshold
        current_pitch = 10.0
        current_kp = 25.0 # > 1.0
        is_crashed = False

        result = self.recovery.update(is_crashed, current_pitch, current_kp)

        # Expect recovery to trigger
        self.assertTrue(self.recovery.recovering)
        # Should return the current pitch *processed by ramp*
        expected = current_pitch - STARTUP_RAMP_SPEED
        self.assertAlmostEqual(result, expected)

    def test_startup_low_kp_ignored(self):
        """Test that low Kp (<1.0) prevents recovery (learning mode)."""
        current_pitch = 10.0
        current_kp = 0.5 # < 1.0
        is_crashed = False

        result = self.recovery.update(is_crashed, current_pitch, current_kp)

        self.assertFalse(self.recovery.recovering)
        self.assertIsNone(result)

    def test_startup_low_lean_ignored(self):
        """Test that low lean (<=5.0) prevents recovery (already upright)."""
        current_pitch = 4.0 # <= 5.0
        current_kp = 25.0
        is_crashed = False

        result = self.recovery.update(is_crashed, current_pitch, current_kp)

        self.assertFalse(self.recovery.recovering)
        self.assertIsNone(result)

    def test_crash_resets_recovery(self):
        """Test that a crash signal resets recovery state."""
        # Force recovery state
        self.recovery.recovering = True

        result = self.recovery.update(is_crashed=True, current_pitch=0.0, current_kp=10.0)

        self.assertFalse(self.recovery.recovering)
        self.assertIsNone(result)

    def test_custom_config_thresholds(self):
        """Test that custom configuration changes the thresholds."""
        # Custom config: stricter Kp, looser lean
        custom_config = ControlConfig(
            soft_recovery_kp_threshold=50.0,
            upright_threshold=15.0
        )
        recovery = RecoveryManager(config=custom_config)

        # Case 1: Old threshold would trigger, new one shouldn't (Kp too low)
        # Kp=25.0 (Old > 1.0, New < 50.0)
        result = recovery.update(is_crashed=False, current_pitch=20.0, current_kp=25.0)
        self.assertFalse(recovery.recovering)
        self.assertIsNone(result)

        # Case 2: Old threshold would trigger, new one shouldn't (Pitch too low)
        # Pitch=10.0 (Old > 5.0, New < 15.0)
        result = recovery.update(is_crashed=False, current_pitch=10.0, current_kp=60.0)
        self.assertFalse(recovery.recovering)
        self.assertIsNone(result)

        # Case 3: Both meet new thresholds
        result = recovery.update(is_crashed=False, current_pitch=20.0, current_kp=60.0)
        self.assertTrue(recovery.recovering)
        expected = 20.0 - STARTUP_RAMP_SPEED
        self.assertAlmostEqual(result, expected)

if __name__ == '__main__':
    unittest.main()
