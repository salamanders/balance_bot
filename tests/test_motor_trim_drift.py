from unittest.mock import MagicMock, patch
from balance_bot.discovery.motor_trim import MotorTrimStep
from balance_bot.discovery.step import StepStatus
from balance_bot.configuration import HardwareConfig, LearningState
from balance_bot.hardware.robot_hardware import RobotHardware

def test_motor_trim_excessive_drift() -> None:
    hw = MagicMock(spec=RobotHardware)

    # In motor_trim.py, `avg_yaw = res.avg_yaw_rate`
    # We just need to mock `res.samples` to be non-empty and `avg_yaw_rate` to be high
    res1 = MagicMock()
    res1.samples = [MagicMock()]
    res1.avg_yaw_rate = 30.0
    res1.abs_avg_yaw_rate = 30.0

    # fwd_steps maneuver, rev_steps maneuver
    # It does this 15 times, so 30 calls. We'll just return res1 every time.
    hw.execute_maneuver.return_value = res1

    config = HardwareConfig(motor_i2c_bus=1, imu_i2c_bus=1)
    state = LearningState()
    state.min_power_visible = 10

    step = MotorTrimStep()
    hw.wait_for_stability = MagicMock()

    with patch.object(step, '_run_square_validation'):
        status, _, _ = step.run(hw, config, state)
        assert status == StepStatus.FATAL
