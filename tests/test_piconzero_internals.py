import sys
import unittest
from unittest.mock import MagicMock, patch

# Clean up sys.modules to force fresh import of piconzero
if "balance_bot.hardware.piconzero" in sys.modules:
    del sys.modules["balance_bot.hardware.piconzero"]

# Mock smbus2 before importing piconzero (for non-Pi systems)
if "smbus2" not in sys.modules:
    try:
        import smbus2 # noqa: F401
    except ImportError:
        sys.modules["smbus2"] = MagicMock()

# Add src to path
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from balance_bot.hardware.piconzero import PiconZero

class TestPiconZeroInternals(unittest.TestCase):

    @patch('balance_bot.hardware.piconzero.smbus.SMBus')
    @patch('balance_bot.hardware.piconzero.time.sleep')
    def test_init_sets_debug_instance(self, mock_sleep, _mock_smbus):
        pz = PiconZero(bus_number=1)

        pz.init(debug=True)
        self.assertTrue(pz.debug, "Instance debug should be True after init(True)")
        # Verify the sleep call (should be 0.1s now)
        mock_sleep.assert_any_call(0.1)

    @patch('balance_bot.hardware.piconzero.smbus.SMBus')
    def test_set_motor_success(self, mock_smbus_cls):
        mock_bus = mock_smbus_cls.return_value
        pz = PiconZero()

        pz.set_motor(0, 100)
        mock_bus.write_byte_data.assert_called_with(0x22, 0, 100)

    @patch('balance_bot.hardware.piconzero.smbus.SMBus')
    @patch('balance_bot.hardware.piconzero.time.sleep')
    def test_set_motor_retry_success(self, mock_sleep, mock_smbus_cls):
        mock_bus = mock_smbus_cls.return_value
        # Fail 2 times then succeed
        mock_bus.write_byte_data.side_effect = [OSError("Fail1"), OSError("Fail2"), None]

        pz = PiconZero()

        pz.set_motor(0, 100)
        self.assertEqual(mock_bus.write_byte_data.call_count, 3)
        # Should have slept twice (once for each retry)
        self.assertGreaterEqual(len(mock_sleep.mock_calls), 2)
        mock_sleep.assert_called_with(0.005)

    @patch('balance_bot.hardware.piconzero.smbus.SMBus')
    @patch('balance_bot.hardware.piconzero.time.sleep')
    def test_set_motor_failure(self, mock_sleep, mock_smbus_cls):
        mock_bus = mock_smbus_cls.return_value
        # Fail all times
        mock_bus.write_byte_data.side_effect = OSError("Fail")

        pz = PiconZero()

        with self.assertRaises(OSError):
            pz.set_motor(0, 100)
        # 10 retries + 2 explicit disarm attempts on failure = 12
        self.assertEqual(mock_bus.write_byte_data.call_count, 12)
        # Should have slept RETRIES times
        self.assertGreaterEqual(len(mock_sleep.mock_calls), pz.retries)

    @patch('balance_bot.hardware.piconzero.smbus.SMBus')
    @patch('balance_bot.hardware.piconzero.time.sleep')
    def test_cleanup(self, _mock_sleep, mock_smbus_cls):
        mock_bus = mock_smbus_cls.return_value
        pz = PiconZero()

        pz.cleanup()
        mock_bus.write_byte_data.assert_called_with(0x22, 20, 0)

if __name__ == "__main__":
    unittest.main()
