import unittest
import sys
from unittest.mock import patch, MagicMock
import os

# Clean up sys.modules to force fresh import of piconzero
if "balance_bot.hardware.piconzero" in sys.modules:
    del sys.modules["balance_bot.hardware.piconzero"]

# Mock smbus2 before importing any module that uses it
if "smbus2" not in sys.modules:
    try:
        import smbus2 # noqa: F401
    except ImportError:
        sys.modules["smbus2"] = MagicMock()

# Ensure src is in path
sys.path.insert(0, os.path.abspath('src'))

from balance_bot.hardware.piconzero import PiconZero

class TestPiconZeroDriver(unittest.TestCase):

    @patch('balance_bot.hardware.piconzero.smbus.SMBus')
    def test_set_motors_calls_individual_writes(self, mock_smbus_cls):
        """
        Test that set_motors calls set_motor for each motor individually.
        """
        mock_bus = mock_smbus_cls.return_value
        pz = PiconZero(bus_number=1)

        pz.set_motors(50, -50)

        # Filter calls to write_byte_data for motor registers 0 and 1
        # Address 0x22
        # Calls: (0x22, motor, value)
        motor_calls = [
            c for c in mock_bus.write_byte_data.call_args_list
            if c.args[0] == 0x22 and c.args[1] in (0, 1)
        ]

        self.assertTrue(len(motor_calls) >= 2, f"Expected 2 writes, got {len(motor_calls)}")

        # Check for specific motor values
        found_motor0 = any(c.args[1] == 0 and c.args[2] == 50 for c in motor_calls)
        found_motor1 = any(c.args[1] == 1 and c.args[2] == (-50 & 0xFF) for c in motor_calls)

        self.assertTrue(found_motor0, "Did not find write for Motor 0 with 50")
        self.assertTrue(found_motor1, "Did not find write for Motor 1 with -50")

    @patch('balance_bot.hardware.piconzero.smbus.SMBus')
    def test_bus_switching(self, mock_smbus_cls):
        """Test that initializing with a different bus works."""
        PiconZero(bus_number=0)

        # Check that smbus.SMBus(0) was called
        mock_smbus_cls.assert_called_with(0)

    @patch('balance_bot.hardware.piconzero.smbus.SMBus')
    def test_set_retries(self, _mock_smbus_cls):
        """Test that set_retries updates the instance retry count."""
        pz = PiconZero()
        pz.set_retries(5)
        self.assertEqual(pz.retries, 5)

if __name__ == '__main__':
    unittest.main()
