import pytest
from unittest.mock import MagicMock, patch
from balance_bot.reflex.balance_core import BalanceCore, MotionRequest, TuningParams
from balance_bot.config import RobotConfig, PIDParams, ControlConfig

def test_turn_gain_impact():
    """Verify that changing turn_gain affects motor output."""

    # 1. Setup Config with default turn_gain
    control_config = ControlConfig(turn_gain=30.0)
    config = RobotConfig(pid=PIDParams(), control=control_config)

    # 2. Mock Hardware
    mock_hw_instance = MagicMock()

    # Mock read_imu_converted
    mock_reading = MagicMock()
    mock_reading.pitch_angle = 0.0
    mock_reading.pitch_rate = 0.0
    mock_reading.yaw_rate = 0.0
    mock_hw_instance.read_imu_converted.return_value = mock_reading

    # Patch RobotHardware class in its definition source so config.to_hardware picks up the mock
    with patch("balance_bot.hardware.robot_hardware.RobotHardware", return_value=mock_hw_instance):

        # 3. Instantiate Core
        core = BalanceCore(config)

        # 4. Define Motion and Tuning
        # Full turn request
        motion = MotionRequest(turn_rate=1.0)
        # Zero PID params to isolate turn effect from balancing
        tuning = TuningParams(kp=0.0, ki=0.0, kd=0.0, target_angle_offset=0.0)

        # 5. Test Default Gain (30.0)
        core.update(motion, tuning, loop_delta_time=0.01)

        # Verify motor calls
        # Expected: Left = +30, Right = -30
        args, _ = mock_hw_instance.set_motors.call_args
        left_motor, right_motor = args

        assert left_motor == 30.0
        assert right_motor == -30.0

        # 6. Change Gain
        config.control.turn_gain = 50.0

        # 7. Test New Gain
        core.update(motion, tuning, loop_delta_time=0.01)

        # Verify motor calls
        args, _ = mock_hw_instance.set_motors.call_args
        left_motor, right_motor = args

        assert left_motor == 50.0
        assert right_motor == -50.0
