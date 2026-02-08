import sys
import pytest
from unittest.mock import MagicMock, patch

# Mock smbus
mock_smbus = MagicMock()
sys.modules["smbus"] = mock_smbus

# Mock RobotHardware and Config
sys.modules["balance_bot.hardware.robot_hardware"] = MagicMock()

from balance_bot.movement_check import MovementCheck  # noqa: E402

@pytest.fixture
def mc():
    with patch("balance_bot.movement_check.RobotConfig") as MockConfig, \
         patch("balance_bot.movement_check.RobotHardware") as MockHW:

        config_inst = MagicMock()
        # Mock enum values
        config_inst.accel_forward_axis.value = 'z'
        config_inst.gyro_yaw_axis.value = 'z'
        MockConfig.load.return_value = config_inst

        hw_inst = MagicMock()
        MockHW.return_value = hw_inst

        mc = MovementCheck()
        return mc

@patch("builtins.input", return_value="y")
def test_drive_move(mock_input, mc):
    # Mock HW read_imu_raw to return data
    # (accel, gyro)
    mc.hw.read_imu_raw.return_value = ({'x': 0, 'y': 0, 'z': 10}, {'x': 0, 'y': 0, 'z': 0})

    # We need to patch collect_imu_data because it uses time.monotonic loop which is hard to mock perfectly without refactoring utils test logic
    # But wait, collect_imu_data IS in utils, and we just tested it.
    # So we can let it run if we mock time.sleep and time.monotonic OR we can mock collect_imu_data.

    # Let's mock collect_imu_data to return predictable data
    with patch("balance_bot.movement_check.collect_imu_data") as mock_collect:
        # Return list of accel dicts, list of gyro dicts
        # Simulate a range of 10 in Z axis (10 to 20)
        mock_collect.return_value = (
            [{'x': 0, 'y': 0, 'z': 10}, {'x': 0, 'y': 0, 'z': 20}], # Accel
            [] # Gyro
        )

        mc.drive_move(60, 1.0, "Forward")

        # Verify it called collect_imu_data correctly
        mock_collect.assert_called_with(mc.hw, duration=1.0, motor_speeds=(60, 60))

        # Verify user input was asked
        mock_input.assert_called()
