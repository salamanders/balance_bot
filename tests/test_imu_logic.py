import math
import pytest
import glm
from balance_bot.enums import Axis
from balance_bot.hardware.robot_hardware import IMUReading

@pytest.mark.parametrize("config_params, accel_in, gyro_in, expected_pitch, expected_pitch_rate", [
    # 1. Default (Vertical Z, Forward Y, Pitch X) - Flat
    (
        {"accel_vertical_axis": Axis.Z, "accel_forward_axis": Axis.Y, "gyro_pitch_axis": Axis.X},
        glm.vec3(0.0, 0.0, 9.8), glm.vec3(0.0, 0.0, 0.0),
        0.0, 0.0
    ),
    # 2. Default - Tilted 45 Forward
    (
        {"accel_vertical_axis": Axis.Z, "accel_forward_axis": Axis.Y, "gyro_pitch_axis": Axis.X},
        glm.vec3(0.0, 9.8*0.707, 9.8*0.707), glm.vec3(10.0, 0.0, 0.0),
        45.0, 10.0
    ),
    # 3. Axis Y Configuration (Forward X, Pitch Y)
    (
        {"gyro_pitch_axis": Axis.Y, "accel_forward_axis": Axis.X, "accel_vertical_axis": Axis.Z},
        glm.vec3(9.8*0.707, 0.0, 9.8*0.707), glm.vec3(0.0, 5.0, 0.0),
        45.0, 5.0
    ),
    # 4. Invert Logic (Pitch X Inverted, Forward Y Inverted)
    (
        {"gyro_pitch_invert": True, "accel_forward_invert": True,
         "accel_vertical_axis": Axis.Z, "accel_forward_axis": Axis.Y, "gyro_pitch_axis": Axis.X},
        glm.vec3(0.0, 9.8*0.707, 9.8*0.707), glm.vec3(10.0, 0.0, 0.0),
        -45.0, -10.0
    ),
    # 5. Sideways Mounting (Vertical X, Forward Y, Pitch Z)
    (
        {"accel_vertical_axis": Axis.X, "accel_forward_axis": Axis.Y, "gyro_pitch_axis": Axis.Z,
         "gyro_yaw_axis": Axis.X, "gyro_roll_axis": Axis.Y},
        glm.vec3(9.8*0.707, 9.8*0.707, 0.0), glm.vec3(0.0, 0.0, 5.0),
        45.0, 5.0
    ),
])
def test_imu_processing(mock_hardware_factory, config_params, accel_in, gyro_in, expected_pitch, expected_pitch_rate):
    """
    Consolidated test for IMU processing logic covering various configurations and inputs.
    """
    hw = mock_hardware_factory(**config_params)

    hw.sensor.get_accel_data.return_value = accel_in
    hw.sensor.get_gyro_data.return_value = gyro_in

    reading: IMUReading = hw.read_imu_converted()

    assert math.isclose(reading.pitch_angle, expected_pitch, abs_tol=0.1)
    assert math.isclose(reading.pitch_rate, expected_pitch_rate)
