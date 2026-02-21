import sys
import unittest
import time
from unittest.mock import MagicMock, call, patch

# Mock smbus before importing piconzero (for non-Pi systems)
if "smbus" not in sys.modules:
    sys.modules["smbus"] = MagicMock()

# Add src to path
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from balance_bot.hardware import piconzero

class TestPiconZeroInternals(unittest.TestCase):
    def setUp(self):
        # Reset piconzero globals
        piconzero.DEBUG = False
        piconzero.RETRIES = 10
        # Mock the bus object inside piconzero module
        piconzero.bus = MagicMock()
        # Also mock print to avoid clutter
        # (Actually, print_function future import in py2 might make this tricky, but py3 is fine)

    def test_init_sets_debug_global(self):
        """Test that init(True) sets the module-level DEBUG variable."""
        # Mock time.sleep to avoid waiting
        with patch('time.sleep') as mock_sleep:
            piconzero.init(debug=True)
            self.assertTrue(piconzero.DEBUG, "Global DEBUG should be True after init(True)")
            # Verify the sleep call (should be 0.1s now)
            mock_sleep.assert_called_with(0.1)

    def test_set_motor_success(self):
        piconzero.setMotor(0, 100)
        piconzero.bus.write_byte_data.assert_called_with(0x22, 0, 100)

    def test_set_motor_retry_success(self):
        # Fail 2 times then succeed
        piconzero.bus.write_byte_data.side_effect = [OSError("Fail1"), OSError("Fail2"), None]

        # Mock time.sleep
        with patch('time.sleep') as mock_sleep:
            piconzero.setMotor(0, 100)
            self.assertEqual(piconzero.bus.write_byte_data.call_count, 3)
            # Should have slept twice (once for each retry)
            self.assertEqual(mock_sleep.call_count, 2)
            mock_sleep.assert_called_with(0.005)

    def test_set_motor_failure(self):
        # Fail all times
        piconzero.bus.write_byte_data.side_effect = OSError("Fail")

        with patch('time.sleep') as mock_sleep:
            with self.assertRaises(OSError):
                piconzero.setMotor(0, 100)
            self.assertEqual(piconzero.bus.write_byte_data.call_count, piconzero.RETRIES)
            # Should have slept RETRIES times
            self.assertEqual(mock_sleep.call_count, piconzero.RETRIES)

    def test_cleanup(self):
        # Mock time.sleep because cleanup calls sleep(0.001)
        with patch('time.sleep'):
            piconzero.cleanup()
            piconzero.bus.write_byte_data.assert_called_with(0x22, 20, 0)

if __name__ == "__main__":
    unittest.main()
