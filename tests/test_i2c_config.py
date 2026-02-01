import os
import sys
from unittest.mock import patch, MagicMock

# Create mocks for the dependencies
mock_mpu_pkg = MagicMock()
mock_mpu_class = MagicMock()
mock_mpu_pkg.mpu6050 = mock_mpu_class
sys.modules["mpu6050"] = mock_mpu_pkg

mock_pz_pkg = MagicMock()
mock_pz_class = MagicMock()
mock_pz_pkg.PiconZero = mock_pz_class
sys.modules["balance_bot.hardware.piconzero"] = mock_pz_pkg

# Now we can safely import RobotHardware
from balance_bot.config import RobotConfig, PIDParams
from balance_bot.hardware.robot_hardware import RobotHardware

def test_config_i2c_bus_default():
    """Test that i2c_bus defaults to 1."""
    config = RobotConfig(pid=PIDParams())
    assert config.i2c_bus == 1

def test_config_i2c_bus_load():
    """Test that i2c_bus can be loaded from config."""
    config = RobotConfig(pid=PIDParams(), i2c_bus=3)
    assert config.i2c_bus == 3

def test_hardware_init_with_bus():
    """Test that RobotHardware initializes mpu6050 with the correct bus."""
    # Reset mocks
    mock_mpu_class.reset_mock()

    # Ensure not in mock mode
    with patch.dict(os.environ, {}, clear=True):
        if "MOCK_HARDWARE" in os.environ:
             del os.environ["MOCK_HARDWARE"]

        hw = RobotHardware(motor_l=0, motor_r=1, i2c_bus=3)

        # Verify mpu6050 was called with bus=3
        mock_mpu_class.assert_called_once_with(0x68, bus=3)
        assert hw.i2c_bus == 3

def test_hardware_init_default_bus():
    """Test that RobotHardware initializes mpu6050 with default bus 1."""
    # Reset mocks
    mock_mpu_class.reset_mock()

    with patch.dict(os.environ, {}, clear=True):
        if "MOCK_HARDWARE" in os.environ:
             del os.environ["MOCK_HARDWARE"]

        hw = RobotHardware(motor_l=0, motor_r=1)

        # Verify mpu6050 was called with bus=1 (default)
        mock_mpu_class.assert_called_once_with(0x68, bus=1)
        assert hw.i2c_bus == 1
