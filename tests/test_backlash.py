import pytest
from unittest.mock import MagicMock, patch
from src.balance_bot.config import RobotConfig, ControlConfig, PIDParams
from src.balance_bot.reflex.balance_core import BalanceCore, MotionRequest, TuningParams

@pytest.fixture
def mock_hardware():
    with patch('src.balance_bot.reflex.balance_core.RobotHardware') as MockHW:
        hw_instance = MockHW.return_value
        # Default IMU reading
        hw_instance.read_imu_converted.return_value = MagicMock(
            pitch_angle=0.0, pitch_rate=0.0, yaw_rate=0.0
        )
        yield hw_instance

@pytest.fixture
def core(mock_hardware):
    config = RobotConfig(
        pid=PIDParams(),
        control=ControlConfig(backlash_pulse_time=0.1) # 100ms pulse
    )
    # Ensure bus config so it doesn't try to auto-detect
    config.motor_i2c_bus = 1
    config.imu_i2c_bus = 1

    core = BalanceCore(config)
    return core

def set_pitch(core, mock_hardware, pitch):
    """Helper to set pitch on both hardware mock and internal filter to avoid lag."""
    mock_hardware.read_imu_converted.return_value.pitch_angle = pitch
    core.filter.angle = pitch

def test_backlash_trigger(core, mock_hardware):
    # Setup tuning
    tuning = TuningParams(kp=1.0, ki=0.0, kd=0.0, target_angle_offset=0.0)
    motion = MotionRequest(velocity=0.0, turn_rate=0.0)

    # 1. Steady state: Positive output
    set_pitch(core, mock_hardware, 10.0)
    core.update(motion, tuning, loop_delta_time=0.01)

    # Verify motor sign is positive (default is 1, so no change yet if output is pos)
    assert core.last_motor_sign == 1
    assert core.backlash_timer == 0.0

    # 2. Cross Zero: Negative output
    set_pitch(core, mock_hardware, -10.0)

    # This call should trigger the backlash pulse
    core.update(motion, tuning, loop_delta_time=0.01)

    # Verify backlash triggered
    # Timer starts at 0.1, decremented by 0.01 -> 0.09
    assert abs(core.backlash_timer - (core.config.control.backlash_pulse_time - 0.01)) < 1e-9
    assert core.last_motor_sign == -1

    # Verify Kick Power
    # Kick power is 40.0 * sign. Sign is -1.
    expected_kick = -40.0
    mock_hardware.set_motors.assert_called_with(expected_kick, expected_kick)

    # 3. Next step: Still in backlash time
    # backlash_timer was 0.09.

    # Call again
    mock_hardware.set_motors.reset_mock()
    core.update(motion, tuning, loop_delta_time=0.05)

    # Timer should be 0.09 - 0.05 = 0.04
    # Still kicking
    mock_hardware.set_motors.assert_called_with(expected_kick, expected_kick)

    # 4. Finish Pulse
    mock_hardware.set_motors.reset_mock()
    core.update(motion, tuning, loop_delta_time=0.10) # 0.04 - 0.10 = -0.06

    # This update consumes the rest of the timer, but since it started > 0, it still kicks.
    mock_hardware.set_motors.assert_called_with(expected_kick, expected_kick)

    # Timer exhausted now
    assert core.backlash_timer < 0

    # 5. Normal operation resumed
    mock_hardware.set_motors.reset_mock()
    core.update(motion, tuning, loop_delta_time=0.01)

    # Should revert to PID output
    # Error is still -10 -> PID is approx -10 * Kp(1.0) = -10

    args, _ = mock_hardware.set_motors.call_args
    left, right = args
    assert left != -40.0
    # It should be around -10 (PID output)
    assert -15.0 < left < -5.0

def test_backlash_ignores_turning(core, mock_hardware):
    tuning = TuningParams(kp=1.0, ki=0.0, kd=0.0, target_angle_offset=0.0)
    motion = MotionRequest(velocity=0.0, turn_rate=1.0) # Turning Right

    # 1. Establish positive state
    set_pitch(core, mock_hardware, 10.0)
    core.update(motion, tuning, loop_delta_time=0.01)

    # 2. Trigger Backlash (Cross to negative)
    set_pitch(core, mock_hardware, -10.0)
    core.update(motion, tuning, loop_delta_time=0.01)

    # Should be pure kick power, ignoring turn
    expected_kick = -40.0
    mock_hardware.set_motors.assert_called_with(expected_kick, expected_kick)

def test_backlash_skipped_if_small_output(core, mock_hardware):
    # If PID output is small (< 2.0), we shouldn't trigger kick
    tuning = TuningParams(kp=0.1, ki=0.0, kd=0.0, target_angle_offset=0.0) # Low gain
    motion = MotionRequest(velocity=0.0, turn_rate=0.0)

    # 1. Establish positive state
    set_pitch(core, mock_hardware, 10.0)
    core.update(motion, tuning, loop_delta_time=0.01)
    assert core.last_motor_sign == 1

    # 2. Cross to negative, but small output
    # Pitch = -10, Kp = 0.1 -> Output = -1.0
    set_pitch(core, mock_hardware, -10.0)
    core.update(motion, tuning, loop_delta_time=0.01)

    # Output is small (-1.0), so backlash logic shouldn't trigger

    assert core.backlash_timer == 0.0
    # last_motor_sign should NOT update if we treat it as "didn't cross significantly yet"
    # Actually, the logic is:
    # if current_sign != self.last_motor_sign and abs(pid_output) > 2.0:
    #     self.last_motor_sign = current_sign

    # So last_motor_sign remains 1.
    assert core.last_motor_sign == 1

    # Check motors - should be PID output (~ -1.0)
    args, _ = mock_hardware.set_motors.call_args
    left, right = args
    assert -1.5 < left < -0.5
