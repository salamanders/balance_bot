import sys
import unittest
from unittest.mock import MagicMock, call

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
        piconzero.init(debug=True)
        self.assertTrue(piconzero.DEBUG, "Global DEBUG should be True after init(True)")

    def test_set_motor_success(self):
        piconzero.setMotor(0, 100)
        piconzero.bus.write_byte_data.assert_called_with(0x22, 0, 100)

    def test_set_motor_retry_success(self):
        # Fail 2 times then succeed
        piconzero.bus.write_byte_data.side_effect = [OSError("Fail1"), OSError("Fail2"), None]
        piconzero.setMotor(0, 100)
        self.assertEqual(piconzero.bus.write_byte_data.call_count, 3)

    def test_set_motor_failure(self):
        # Fail all times
        piconzero.bus.write_byte_data.side_effect = OSError("Fail")
        with self.assertRaises(OSError):
            piconzero.setMotor(0, 100)
        self.assertEqual(piconzero.bus.write_byte_data.call_count, piconzero.RETRIES)

    def test_read_input_success(self):
        piconzero.bus.read_word_data.return_value = 1234
        val = piconzero.readInput(0)
        self.assertEqual(val, 1234)
        piconzero.bus.read_word_data.assert_called_with(0x22, 1)

    def test_set_input_config_complex(self):
        # Channel 0, Mode 4 (DutyCycle), Period 2000
        # Should write byte then write word
        # Note: The original code logic for Period writing is inside the retry loop.
        # My refactor needs to preserve this.

        # We simulate success on the first try
        piconzero.setInputConfig(0, 4, period=2000)

        # Check calls
        # write_byte_data(addr, INCFG0+0, 4) -> (0x22, 14, 4)
        # write_word_data(addr, INPERIOD0+0, 2000) -> (0x22, 21, 2000)

        # Note: In original code, it does:
        # bus.write_byte_data...
        # if value==4 or 5: bus.write_word_data...
        # return

        # If I use a helper, I need to ensure both happen.

        piconzero.bus.write_byte_data.assert_called_with(0x22, 14, 4)
        piconzero.bus.write_word_data.assert_called_with(0x22, 21, 2000)

    def test_cleanup(self):
        piconzero.cleanup()
        piconzero.bus.write_byte_data.assert_called_with(0x22, 20, 0)

if __name__ == "__main__":
    unittest.main()
