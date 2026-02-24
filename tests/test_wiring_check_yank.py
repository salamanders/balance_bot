import sys
import pytest
import glm
from unittest.mock import MagicMock, patch, call

# Mock smbus2 before import
if 'smbus2' not in sys.modules:
    sys.modules['smbus2'] = MagicMock()

from balance_bot.wiring_check import WiringCheck
from balance_bot.enums import Axis
from balance_bot.hardware.robot_hardware import MeasureResult, IMUReading
from balance_bot.configuration import HardwareConfig, LearningState

@pytest.fixture
def wc_fixture():
    with patch("balance_bot.utils.smbus"), \
         patch("balance_bot.wiring_check.RobotHardware") as MockHW, \
         patch("balance_bot.wiring_check.HardwareConfig") as MockHWConfig, \
         patch("balance_bot.wiring_check.LearningState") as MockLearningState:

        # Setup Configs
        hw_config = MagicMock(spec=HardwareConfig)
        hw_config.motor_l = 0
        hw_config.motor_r = 1

        # Make model_copy create a new mock but keep the update capability simulating a real Pydantic model
        def model_copy_side_effect(update=None, deep=False):
            new_config = MagicMock(spec=HardwareConfig)
            # Copy existing attributes
            for k, v in hw_config.__dict__.items():
                setattr(new_config, k, v)
            # Apply updates
            if update:
                for k, v in update.items():
                    setattr(new_config, k, v)
            return new_config

        hw_config.model_copy.side_effect = model_copy_side_effect
        MockHWConfig.load.return_value = hw_config

        learning_state = MagicMock(spec=LearningState)
        learning_state.min_power_visible = 20
        learning_state.motor_direction_verified = False
        MockLearningState.load.return_value = learning_state

        wc = WiringCheck()
        wc.hw = MockHW.return_value
        wc.hw.hw_config = hw_config
        wc.hw.learning_state = learning_state

        yield wc, hw_config, learning_state

def test_determine_motor_direction_yank(wc_fixture):
    wc, hw_config, learning_state = wc_fixture

    # 1. Setup Mock Data
    # Baseline: Robot is stable
    # Accel: Gravity acts on Y axis (assuming robot is flat-ish but maybe tilted)
    # Let's say Z is Vertical (Gravity -9.8), X is Forward, Y is Pitch Axis (Lateral)
    # But wait, we don't know axes yet. Let's pick arbitrary raw axes.
    # Raw Frame: X=Lateral(Pitch), Y=Forward, Z=Vertical

    # Baseline Accel: Gravity on Z (-9.8)
    baseline_accel = glm.vec3(0.1, 0.2, -9.8)

    # Yank Action:
    # - Drive Forward
    # - Inertial Pitch Backward (Rotate around X axis).
    #   If Right-Hand Rule applies to X, positive rotation is Y->Z.
    #   Let's say it rotates +50 deg/s around X.
    #   User requirement: "set gyro_pitch_invert so this specific rotational direction reads as a NEGATIVE pitch rate."
    #   So if raw is +50, invert must be TRUE to make it -50.

    # - Inertial Lag Backward (Accel along Y axis).
    #   If Y is Forward, lag means mass moves backward relative to sensor -> Negative Y acceleration?
    #   Wait, physics: F=ma. Sensor accelerates Forward (Pos Y). Mass lags?
    #   Actually, accelerometer measures proper acceleration (Force/mass).
    #   If robot accelerates Forward, the sensor is pushed Forward by the mounting.
    #   The internal mass resists, pushing "back" on the sensor elements?
    #   No, accelerometer reads the force applied TO the sensor to make it move.
    #   If the robot accelerates Forward (+Y), the accelerometer reads +Y.
    #   BUT the user prompt says: "The IMU mass lags backward when the base shoots forward... save this as accel_forward_axis and set accel_forward_invert so this delta reads as NEGATIVE."
    #   Okay, I will follow the user's physics model regardless of my own.
    #   User says: "delta (moving - baseline) ... set invert so this delta reads as NEGATIVE."

    #   So if the physical event (Yank) produces a Delta Vector D, we want D_mapped < 0.
    #   Let's say the raw Delta is +2.0 on Y axis. Then we must Invert Y to get -2.0.

    # Baseline Reading
    wc.hw.read_imu_raw.return_value = (baseline_accel, glm.vec3(0,0,0)) # Accel, Gyro

    # Yank Measurement Result
    # Gyro: Rotating around X (+50)
    # Accel: Y increases by +2.0 (Lag? Forward? Whatever, delta is +2.0)
    yank_samples = []
    for _ in range(10):
        # Gyro: Dominant X
        g = glm.vec3(50.0, 1.0, -1.0)
        # Accel: Baseline + Delta.
        # Delta on Y = +2.0. So Y becomes 2.2.
        a = baseline_accel + glm.vec3(0.0, 2.0, 0.0)

        yank_samples.append(IMUReading(0,0,0,0,0, accel_raw=a, gyro_raw=g))

    res_yank = MeasureResult(duration=0.3, samples=yank_samples)

    wc.hw.drive_and_measure.return_value = res_yank

    # 2. Run the method
    wc.determine_motor_direction()

    # 3. Verification
    # Check HardwareConfig updates
    # We expect:
    #   Gyro Pitch Axis = X
    #   Gyro Pitch Invert = True (because raw was +50, we want negative)
    #   Accel Forward Axis = Y (excluding pitch axis X)
    #   Accel Forward Invert = True (because delta was +2.0, we want negative)

    # Note: MockHWConfig is a mock, so we check the calls to model_copy
    assert hw_config.model_copy.called

    # We need to find the call that sets these values
    # It might be one or multiple calls.

    # Collect all updates
    all_updates = {}
    for call_args in hw_config.model_copy.call_args_list:
        if 'update' in call_args.kwargs:
            all_updates.update(call_args.kwargs['update'])

    print("Captured Updates:", all_updates)

    assert all_updates.get('gyro_pitch_axis') == Axis.X
    assert all_updates.get('gyro_pitch_invert') is True

    assert all_updates.get('accel_forward_axis') == Axis.Y
    assert all_updates.get('accel_forward_invert') is True

    # Check LearningState update
    assert learning_state.motor_direction_verified is True
    assert learning_state.save.called
