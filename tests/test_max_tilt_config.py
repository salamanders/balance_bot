from typing import Any
from unittest.mock import MagicMock
from balance_bot.reflex.balance_core import BalanceCore, MotionRequest, TuningParams
from balance_bot.configuration import HardwareConfig, LearningState, PIDParams, ControlConfig

def create_mocked_core(monkeypatch: Any, max_tilt_angle: Any = None) -> Any:
    # Setup Config
    control_config = ControlConfig()
    if max_tilt_angle is not None:
        control_config.max_tilt_angle = max_tilt_angle

    hw_config = HardwareConfig(motor_l=0, motor_r=1)
    learning_state = LearningState(
        pid=PIDParams(),
        control=control_config
    )

    # Mock Hardware
    monkeypatch.setattr("balance_bot.hardware.robot_hardware.RobotHardware.__init__", lambda self, *args, **kwargs: None)
    monkeypatch.setattr("balance_bot.hardware.robot_hardware.RobotHardware.init", lambda self: None)

    dummy_reading = MagicMock()
    dummy_reading.pitch_angle = 0.0
    dummy_reading.pitch_rate = 0.0
    dummy_reading.yaw_rate = 0.0
    monkeypatch.setattr("balance_bot.hardware.robot_hardware.RobotHardware.read_imu_converted", lambda self: dummy_reading)

    monkeypatch.setattr("balance_bot.hardware.robot_hardware.RobotHardware.set_motors", lambda self, left_pwm, right_pwm: None)
    monkeypatch.setattr("balance_bot.hardware.robot_hardware.RobotHardware.stop", lambda self: None)

    core = BalanceCore(hw_config, learning_state)

    # Mock Filter to ensure pitch is 0.0
    core.filter = MagicMock()
    core.filter.update.return_value = 0.0

    # Mock PID to capture error
    core.pid = MagicMock()
    core.pid.params = learning_state.pid # Maintain reference to params
    core.pid.update.return_value = 0.0

    return core

def test_max_tilt_angle_default(monkeypatch: Any) -> Any:
    """Verify that the default max tilt angle is used if not specified."""
    custom_angle = 25.0
    core = create_mocked_core(monkeypatch, max_tilt_angle=custom_angle)

    motion = MotionRequest(velocity=1.0) # Full speed forward
    tuning = TuningParams(kp=1.0, ki=0.0, kd=0.0, target_angle_offset=0.0)

    core.update(motion, tuning, loop_delta_time=0.01)

    # Get the arguments passed to pid.update
    args, _ = core.pid.update.call_args
    error = args[0]

    # This should fail initially because the code uses MAX_TILT_ANGLE = 10.0
    # Expected error = 0 - 25.0 = -25.0
    # Actual error = 0 - 10.0 = -10.0
    assert error == -custom_angle

def test_max_tilt_angle_negative_velocity(monkeypatch: Any) -> Any:
    """Verify that negative velocity tilts backwards."""
    custom_angle = 20.0
    core = create_mocked_core(monkeypatch, max_tilt_angle=custom_angle)

    motion = MotionRequest(velocity=-0.5) # Half speed backward
    tuning = TuningParams(kp=1.0, ki=0.0, kd=0.0, target_angle_offset=0.0)

    core.update(motion, tuning, loop_delta_time=0.01)

    # Expected target angle = 0 + 0 + (-0.5 * 20.0) = -10.0
    # Expected error = 0 - (-10.0) = 10.0

    args, _ = core.pid.update.call_args
    error = args[0]

    assert error == 10.0
