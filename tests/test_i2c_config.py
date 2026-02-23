import os
import sys
from unittest.mock import patch, MagicMock
from balance_bot.config import RobotConfig, PIDParams

def test_config_i2c_bus_default():
    """Test that i2c buses default to None (defer to hardware default)."""
    config = RobotConfig(pid=PIDParams())
    assert config.motor_i2c_bus is None
    assert config.imu_i2c_bus is None

def test_config_i2c_bus_load_separate():
    """Test that distinct buses can be loaded from config."""
    config = RobotConfig(pid=PIDParams(), motor_i2c_bus=0, imu_i2c_bus=3)
    assert config.motor_i2c_bus == 0
    assert config.imu_i2c_bus == 3

def test_hardware_init_with_bus():
    """Test that RobotHardware initializes drivers with correct buses."""
    # Setup Mocks
    mock_mpu_pkg = MagicMock()
    mock_mpu_class = MagicMock()
    mock_mpu_pkg.mpu6050 = mock_mpu_class

    mock_pz_pkg = MagicMock()
    mock_pz_class = MagicMock()
    mock_pz_pkg.PiconZero = mock_pz_class

    # Patch modules
    with patch.dict(sys.modules, {
        "mpu6050": mock_mpu_pkg,
        "balance_bot.hardware.piconzero": mock_pz_pkg
    }):
        # Force reload RobotHardware to pick up mocks
        if "balance_bot.hardware.robot_hardware" in sys.modules:
            del sys.modules["balance_bot.hardware.robot_hardware"]

        from balance_bot.hardware.robot_hardware import RobotHardware

        # Ensure not in mock mode
        with patch.dict(os.environ, {}, clear=True):
            # Fix: Create config and pass it
            config = RobotConfig(pid=PIDParams())
            config.motor_l = 0
            config.motor_r = 1
            config.motor_i2c_bus = 0
            config.imu_i2c_bus = 3

            hw = RobotHardware(config)

            # Verify mpu6050 was called with bus=3
            mock_mpu_class.assert_called_once_with(0x68, bus=3)
            # Fix: Check config
            assert hw.config.imu_i2c_bus == 3

            # Verify PiconZero was called with bus=0
            mock_pz_class.assert_called_once_with(bus_number=0)
            # Fix: Check config
            assert hw.config.motor_i2c_bus == 0

def test_hardware_init_uses_defaults_if_none():
    """Test that RobotHardware uses defaults if buses are None."""
    # Setup Mocks
    mock_mpu_pkg = MagicMock()
    mock_mpu_class = MagicMock()
    mock_mpu_pkg.mpu6050 = mock_mpu_class

    mock_pz_pkg = MagicMock()
    mock_pz_class = MagicMock()
    mock_pz_pkg.PiconZero = mock_pz_class

    with patch.dict(sys.modules, {
        "mpu6050": mock_mpu_pkg,
        "balance_bot.hardware.piconzero": mock_pz_pkg
    }):
        # Force reload RobotHardware
        if "balance_bot.hardware.robot_hardware" in sys.modules:
            del sys.modules["balance_bot.hardware.robot_hardware"]

        from balance_bot.hardware.robot_hardware import RobotHardware

        with patch.dict(os.environ, {}, clear=True):
            # Fix: Create config with None
            config = RobotConfig(pid=PIDParams())
            config.motor_l = 0
            config.motor_r = 1
            config.motor_i2c_bus = None
            config.imu_i2c_bus = None

            hw = RobotHardware(config)

            # Verify mpu6050 was called with default bus 1
            mock_mpu_class.assert_called_once_with(0x68, bus=1)
            # Fix: Check config
            assert hw.config.imu_i2c_bus == 1

            # Verify PiconZero was called with default bus 1
            mock_pz_class.assert_called_once_with(bus_number=1)
            # Fix: Check config
            assert hw.config.motor_i2c_bus == 1
