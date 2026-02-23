import pytest
from unittest.mock import MagicMock
from balance_bot.reflex.balance_core import BalanceCore, MotionRequest, TuningParams, BalanceTelemetry
from balance_bot.configuration import HardwareConfig, LearningState

def test_balance_core_update_with_mutable_tuning_params():
    # Setup
    hw_config = HardwareConfig()
    learning_state = LearningState(kp=1.0, ki=0.0, kd=0.0)

    # Mock Hardware inside BalanceCore
    with pytest.MonkeyPatch.context() as m:
        # Mock init to avoid I2C bus checks
        m.setattr("balance_bot.hardware.robot_hardware.RobotHardware.__init__", lambda self, *args, **kwargs: None)
        m.setattr("balance_bot.hardware.robot_hardware.RobotHardware.init", lambda self: None)

        # Mock read_imu_converted to return valid reading
        dummy_reading = MagicMock()
        dummy_reading.pitch_angle = 5.0
        dummy_reading.pitch_rate = 0.0
        dummy_reading.yaw_rate = 0.0
        m.setattr("balance_bot.hardware.robot_hardware.RobotHardware.read_imu_converted", lambda self: dummy_reading)
        m.setattr("balance_bot.hardware.robot_hardware.RobotHardware.set_motor_retries", lambda self, r: None)

        # Mock motor setting
        m.setattr("balance_bot.hardware.robot_hardware.RobotHardware.set_motors", lambda self, left, right: None)
        m.setattr("balance_bot.hardware.robot_hardware.RobotHardware.stop", lambda self: None)

        # Initialize Core
        core = BalanceCore(hw_config, learning_state)

        # Test Data
        motion = MotionRequest()
        # Instantiate TuningParams (new mutable class)
        tuning = TuningParams(kp=2.0, ki=0.1, kd=0.01, target_angle_offset=1.0)

        # 1. First Update
        telemetry = core.update(motion, tuning, loop_delta_time=0.01)

        assert isinstance(telemetry, BalanceTelemetry)
        # Check if PID params were updated from tuning params
        # Use learning_state which is shared with PID
        assert core.learning_state.kp == 2.0
        assert core.learning_state.ki == 0.1
        assert core.learning_state.kd == 0.01

        # 2. Modify TuningParams in place
        tuning.kp = 3.0
        tuning.ki = 0.2
        tuning.target_angle_offset = 2.0

        telemetry = core.update(motion, tuning, loop_delta_time=0.01)

        assert core.learning_state.kp == 3.0
        assert core.learning_state.ki == 0.2

        print("Integration test passed!")

if __name__ == "__main__":
    test_balance_core_update_with_mutable_tuning_params()
