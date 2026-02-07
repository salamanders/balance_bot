import unittest
import sys
import smbus
from unittest.mock import MagicMock, patch
import os

# Ensure src is in path
sys.path.insert(0, os.path.abspath('src'))

class TestPiconZeroDriver(unittest.TestCase):
    def setUp(self):
        # Force reload of piconzero module to avoid test pollution from test_i2c_config.py
        # which mocks this module at top-level.
        if 'balance_bot.hardware.piconzero' in sys.modules:
            del sys.modules['balance_bot.hardware.piconzero']

        # Also need to make sure 'balance_bot' package is available
        # It should be via sys.path

        from balance_bot.hardware.piconzero import PiconZero, REG_MOTOR_A
        self.PiconZero = PiconZero
        self.REG_MOTOR_A = REG_MOTOR_A

        # Patch smbus.SMBus globally
        self.mock_bus = MagicMock()
        self.patcher = patch('smbus.SMBus', return_value=self.mock_bus)
        self.patcher.start()

        self.picon = self.PiconZero(bus_number=1)

    def tearDown(self):
        self.patcher.stop()

    def test_set_motors_calls_individual_writes(self):
        """
        Test that set_motors calls set_motor for each motor individually,
        resulting in two write_byte_data calls, NOT a block write.
        """
        self.picon.set_motors(50, -50)

        # Filter calls to write_byte_data for motor registers 0 and 1
        motor_calls = [
            call for call in self.mock_bus.write_byte_data.call_args_list
            if call.args[1] in (0, 1)
        ]

        self.assertEqual(len(motor_calls), 2, "Expected 2 individual byte writes for motors")

        # strict check on arguments
        self.mock_bus.write_byte_data.assert_any_call(0x22, 0, 50)
        self.mock_bus.write_byte_data.assert_any_call(0x22, 1, -50)

    def test_set_motors_does_not_use_block_write(self):
        """
        Test that set_motors does NOT use write_i2c_block_data.
        """
        self.picon.set_motors(50, -50)

        # This should be 0 calls to block write for motor registers
        block_calls = [
            call for call in self.mock_bus.write_i2c_block_data.call_args_list
            if call.args[1] == self.REG_MOTOR_A
        ]
        self.assertEqual(len(block_calls), 0, "Should not use block write for motors")

    def test_retry_on_write_byte(self):
        """Test that write_byte retries on failure and eventually succeeds."""
        self.picon.set_retries(3)
        # Fail twice, succeed on third attempt
        self.mock_bus.write_byte_data.side_effect = [OSError("Fail 1"), OSError("Fail 2"), None]

        self.picon._write_byte(0x10, 0xFF)

        self.assertEqual(self.mock_bus.write_byte_data.call_count, 3)

    def test_retry_fail_logs_error(self):
        """Test that write_byte retries and logs error on total failure."""
        self.picon.set_retries(3)
        self.mock_bus.write_byte_data.side_effect = [OSError("Fail 1"), OSError("Fail 2"), OSError("Fail 3")]

        with self.assertLogs('balance_bot.hardware.piconzero', level='ERROR') as cm:
            self.picon._write_byte(0x10, 0xFF)

        self.assertTrue(any("Failed to write byte" in msg for msg in cm.output))

if __name__ == '__main__':
    unittest.main()
