import os
import sys
from unittest.mock import patch, MagicMock
from balance_bot.configuration import HardwareConfig, LearningState

def test_config_i2c_bus_default() -> None:
    """Test that i2c buses default to None (defer to hardware default)."""
    config = HardwareConfig()
    assert config.motor_i2c_bus is None
    assert config.imu_i2c_bus is None

def test_config_i2c_bus_load_separate() -> None:
    """Test that distinct buses can be loaded from config."""
    config = HardwareConfig(motor_i2c_bus=0, imu_i2c_bus=3)
    assert config.motor_i2c_bus == 0
    assert config.imu_i2c_bus == 3

def test_hardware_init_with_bus() -> None:
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
            config = HardwareConfig(motor_l=0, motor_r=1, motor_i2c_bus=0, imu_i2c_bus=3)
            hw = RobotHardware(config, LearningState())

            # Verify mpu6050 was called with bus=3
            mock_mpu_class.assert_called_once_with(0x68, bus=3)
            assert hw.hw_config.imu_i2c_bus == 3

            # Verify PiconZero was called with bus=0
            mock_pz_class.assert_called_once_with(bus_number=0)
            assert hw.hw_config.motor_i2c_bus == 0

def test_hardware_init_skips_if_none() -> None:
    """Test that RobotHardware skips init if buses are None."""
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
            config = HardwareConfig(motor_l=0, motor_r=1)
            RobotHardware(config, LearningState())

            # Verify mpu6050 was NOT called
            mock_mpu_class.assert_not_called()

            # Verify PiconZero was NOT called
            mock_pz_class.assert_not_called()
