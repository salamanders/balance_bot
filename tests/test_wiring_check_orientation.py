import sys
import pytest
import glm
from unittest.mock import MagicMock, patch

# Mock smbus2 before import
if 'smbus2' not in sys.modules:
    sys.modules['smbus2'] = MagicMock()

from balance_bot.wiring_check import WiringCheck
from balance_bot.enums import Axis
from balance_bot.hardware.robot_hardware import MeasureResult, IMUReading

@pytest.fixture
def wc_fixture():
    with patch("balance_bot.utils.smbus"), \
         patch("balance_bot.wiring_check.RobotHardware") as MockHW, \
         patch("balance_bot.wiring_check.HardwareConfig") as MockHWConfig, \
         patch("balance_bot.wiring_check.LearningState") as MockLearningState:

        # Setup Configs
        hw_config = MagicMock()
        hw_config.accel_vertical_axis = None
        hw_config.motor_l = 0
        hw_config.motor_r = 1
        MockHWConfig.load.return_value = hw_config

        # When model_copy is called, return the same mock so we can track calls easier
        hw_config.model_copy.return_value = hw_config

        learning_state = MagicMock()
        learning_state.min_power_visible = 20
        MockLearningState.load.return_value = learning_state

        wc = WiringCheck()
        wc.hw = MockHW.return_value

        # Mock wait_for_stability
        wc.hw.wait_for_stability = MagicMock()

        yield wc, hw_config, learning_state

def test_calibrate_orientation(wc_fixture):
    wc, hw_config, learning_state = wc_fixture

    # 1. Back (Vertical Gravity) - Z=-9.8
    back_vec = glm.vec3(0.1, 0.2, -9.8)
    # 2. Front (Flop Forward) - Y=9.8
    front_vec = glm.vec3(0.1, 9.8, 0.2)

    # Setup MeasureResults for drive_and_measure calls

    # Result 1 (Back Measurement)
    res_back = MeasureResult(duration=1.0, samples=[
        IMUReading(0,0,0,0,0, accel_raw=back_vec, gyro_raw=glm.vec3(0,0,0))
    ] * 10)

    # Result 2 (Flop Action)
    res_flop = MeasureResult(duration=0.5, samples=[])

    # Result 3 (Front Measurement)
    res_front = MeasureResult(duration=1.0, samples=[
        IMUReading(0,0,0,0,0, accel_raw=front_vec, gyro_raw=glm.vec3(0,0,0))
    ] * 10)

    # wc.hw.drive_and_measure.side_effect = [res_back, res_flop, res_front]

    # Mock read_imu_converted for rest angle calculation
    wc.hw.read_imu_converted.return_value = IMUReading(
        pitch_angle=30.0, pitch_rate=0, yaw_rate=0, roll_angle=0, roll_rate=0,
        accel_raw=front_vec, gyro_raw=glm.vec3(0,0,0)
    )

    # Mock get_axis_value to return dummy value
    wc.hw.get_axis_value.return_value = 0.5

    # Run
    # Mock _measure_gravity_vectors to bypass physical collection
    with patch.object(wc, '_measure_gravity_vectors', return_value=(back_vec, front_vec)):
        wc.calibrate_static_orientation()

    # Assertions
    # Verify calls to model_copy with update kwargs
    assert hw_config.model_copy.called

    found_axis_update = False
    for call in hw_config.model_copy.call_args_list:
        kwargs = call.kwargs.get('update', {})
        if 'accel_vertical_axis' in kwargs:
            assert kwargs['accel_vertical_axis'] == Axis.Z
            assert kwargs['accel_vertical_invert'] == True
            assert kwargs['gyro_pitch_axis'] == Axis.X
            found_axis_update = True

    assert found_axis_update, "Did not find axis update call"
