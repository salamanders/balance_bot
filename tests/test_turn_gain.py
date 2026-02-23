import pytest
from unittest.mock import MagicMock
from balance_bot.reflex.balance_core import BalanceCore, MotionRequest, TuningParams
from balance_bot.configuration import HardwareConfig, LearningState, ControlConfig

def test_turn_gain_impact():
    """Verify that changing turn_gain affects motor output."""

    # 1. Setup Config with default turn_gain
    # Note: turn_gain default is 30.0 in class definition, but we can override.
    # Let's start with explicit 30.0
    control_config = ControlConfig(turn_gain=30.0)
    config = HardwareConfig(pid=LearningState(), control=control_config)

    # 2. Mock Hardware
    # We need to mock RobotHardware enough to instantiate BalanceCore and run update()
    mock_hw_cls = MagicMock()
    mock_hw_instance = mock_hw_cls.return_value

    # Mock read_imu_converted
    mock_reading = MagicMock()
    mock_reading.pitch_angle = 0.0
    mock_reading.pitch_rate = 0.0
    mock_reading.yaw_rate = 0.0
    mock_hw_instance.read_imu_converted.return_value = mock_reading

    # Patch RobotHardware class in balance_core.py
    with pytest.MonkeyPatch.context() as m:
        m.setattr("balance_bot.reflex.balance_core.RobotHardware", mock_hw_cls)

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
