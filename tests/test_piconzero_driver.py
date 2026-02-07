import unittest
import sys
from unittest.mock import patch
import os

# Ensure src is in path
sys.path.insert(0, os.path.abspath('src'))

class TestPiconZeroDriver(unittest.TestCase):
    def setUp(self):
        # Patch smbus.SMBus so we don't hit real hardware
        self.patcher = patch('smbus.SMBus')
        self.MockSMBus = self.patcher.start()

        # Reload modules to ensure mocks take effect and we get a fresh state
        if 'balance_bot.hardware.piconzero' in sys.modules:
            del sys.modules['balance_bot.hardware.piconzero']
        if 'balance_bot.hardware.piconzero_adapter' in sys.modules:
            del sys.modules['balance_bot.hardware.piconzero_adapter']

        from balance_bot.hardware.piconzero_adapter import PiconZeroAdapter
        from balance_bot.hardware import piconzero as pz_module

        self.PiconZeroAdapter = PiconZeroAdapter
        self.pz_module = pz_module

        # Instantiate adapter with default bus 1
        self.adapter = self.PiconZeroAdapter(bus_number=1)

    def tearDown(self):
        self.patcher.stop()

    def test_set_motors_calls_individual_writes(self):
        """
        Test that set_motors calls set_motor for each motor individually,
        resulting in write_byte_data calls to the module's bus.
        """
        # We verify calls on the actual bus object used by the module
        bus = self.pz_module.bus

        self.adapter.set_motors(50, -50)

        # Filter calls to write_byte_data for motor registers 0 and 1
        # Address 0x22
        motor_calls = [
            call for call in bus.write_byte_data.call_args_list
            if call.args[0] == 0x22 and call.args[1] in (0, 1)
        ]

        self.assertTrue(len(motor_calls) >= 2, f"Expected 2 writes, got {len(motor_calls)}. Calls: {bus.write_byte_data.call_args_list}")

        # Check for specific motor values
        # Args: (addr, reg, value)
        found_motor0 = any(c.args[1] == 0 and c.args[2] == 50 for c in motor_calls)
        found_motor1 = any(c.args[1] == 1 and c.args[2] == -50 for c in motor_calls)

        self.assertTrue(found_motor0, "Did not find write for Motor 0 with 50")
        self.assertTrue(found_motor1, "Did not find write for Motor 1 with -50")

    def test_bus_switching(self):
        """Test that initializing with a different bus updates the global pz.bus."""
        # Initialize with bus 0
        self.PiconZeroAdapter(bus_number=0)

        # Check that smbus.SMBus(0) was called on our patch
        self.MockSMBus.assert_any_call(0)

    def test_set_retries(self):
        """Test that set_retries updates the module global RETRIES."""
        self.adapter.set_retries(5)
        self.assertEqual(self.pz_module.RETRIES, 5)

if __name__ == '__main__':
    unittest.main()
