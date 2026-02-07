import pytest
from unittest.mock import MagicMock
from balance_bot.reflex.balance_core import BalanceCore, MotionRequest, TuningParams, BalanceTelemetry
from balance_bot.config import RobotConfig, PIDParams

def test_balance_core_update_with_mutable_tuning_params():
    # Setup
    config = RobotConfig(pid=PIDParams(kp=1.0, ki=0.0, kd=0.0))

    # Mock Hardware inside BalanceCore
    # We use monkeypatch to avoid hardware init issues
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

        # Mock motor setting
        m.setattr("balance_bot.hardware.robot_hardware.RobotHardware.set_motors", lambda self, left, right: None)
        m.setattr("balance_bot.hardware.robot_hardware.RobotHardware.stop", lambda self: None)

        # Initialize Core
        core = BalanceCore(config)

        # Test Data
        motion = MotionRequest()
        # Instantiate TuningParams (new mutable class)
        tuning = TuningParams(kp=2.0, ki=0.1, kd=0.01, target_angle_offset=1.0)

        # 1. First Update
        telemetry = core.update(motion, tuning, loop_delta_time=0.01)

        assert isinstance(telemetry, BalanceTelemetry)
        # Check if PID params were updated from tuning params
        assert core.pid.params.kp == 2.0
        assert core.pid.params.ki == 0.1
        assert core.pid.params.kd == 0.01

        # 2. Modify TuningParams in place (Optimization verification)
        tuning.kp = 3.0
        tuning.ki = 0.2
        tuning.target_angle_offset = 2.0

        telemetry = core.update(motion, tuning, loop_delta_time=0.01)

        assert core.pid.params.kp == 3.0
        assert core.pid.params.ki == 0.2
        # Target angle check logic in BalanceCore:
        # target_angle = config.pid.target_angle + tuning.target_angle_offset + velocity_tilt
        # We can't easily check target_angle directly as it is local variable, but we can verify it ran without error.

        print("Integration test passed!")

if __name__ == "__main__":
    test_balance_core_update_with_mutable_tuning_params()
