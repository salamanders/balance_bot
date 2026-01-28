import os
import logging
from typing import Protocol, runtime_checkable
from dataclasses import dataclass

from .utils import clamp, calculate_pitch, Vector3

logger = logging.getLogger(__name__)

GYRO_AXIS_X = "x"
GYRO_AXIS_Y = "y"
MOTOR_MIN_OUTPUT = -100
MOTOR_MAX_OUTPUT = 100


@dataclass(frozen=True)
class IMUReading:
    """Immutable data structure for converted IMU readings."""

    pitch_angle: float
    pitch_rate: float
    yaw_rate: float


@runtime_checkable
class MotorDriver(Protocol):
    def init(self) -> None: ...
    def cleanup(self) -> None: ...
    def stop(self) -> None: ...
    def set_motor(self, motor: int, value: int) -> None: ...


@runtime_checkable
class IMUDriver(Protocol):
    def get_accel_data(self) -> Vector3: ...
    def get_gyro_data(self) -> Vector3: ...


class PiconZeroAdapter:
    """Adapter for the vendored piconzero module."""

    def __init__(self, pz_module):
        self.pz = pz_module

    def init(self) -> None:
        self.pz.init()

    def cleanup(self) -> None:
        self.pz.cleanup()

    def stop(self) -> None:
        self.pz.stop()

    def set_motor(self, motor: int, value: int) -> None:
        self.pz.setMotor(motor, value)


class MPU6050Adapter:
    """Adapter for the mpu6050 library class."""

    def __init__(self, sensor_instance):
        self.sensor = sensor_instance

    def get_accel_data(self) -> Vector3:
        return self.sensor.get_accel_data()

    def get_gyro_data(self) -> Vector3:
        return self.sensor.get_gyro_data()


class RobotHardware:
    """Abstraction layer for robot hardware (Motors and IMU)."""

    def __init__(
        self,
        motor_l: int,
        motor_r: int,
        invert_l: bool = False,
        invert_r: bool = False,
        gyro_axis: str = GYRO_AXIS_X,
        gyro_invert: bool = False,
    ):
        """
        Initialize the robot hardware abstraction.
        :param motor_l: Left motor channel.
        :param motor_r: Right motor channel.
        :param invert_l: Whether to invert left motor.
        :param invert_r: Whether to invert right motor.
        :param gyro_axis: Axis to use for pitch ('x' or 'y').
        :param gyro_invert: Whether to invert gyro reading.
        """
        self.motor_l = motor_l
        self.motor_r = motor_r
        self.invert_l = invert_l
        self.invert_r = invert_r
        self.gyro_axis = gyro_axis
        self.gyro_invert = gyro_invert

        self.pz: MotorDriver
        self.sensor: IMUDriver

        self._init_hardware()

    def _init_hardware(self) -> None:
        """Initialize hardware components or mocks."""
        if os.environ.get("MOCK_HARDWARE"):
            self._init_mock_hardware()
            return

        try:
            from . import piconzero as pz_module
            from mpu6050 import mpu6050

            self.pz = PiconZeroAdapter(pz_module)
            self.sensor = MPU6050Adapter(mpu6050(0x68))
            logger.info("Hardware initialized.")
        except (ImportError, OSError):
            logger.warning("Hardware not found. Falling back to Mock Mode.")
            self._init_mock_hardware()

    def _init_mock_hardware(self) -> None:
        """Initialize mock hardware components."""
        logger.info("Running in Mock Mode")
        from .mocks import MockPiconZero, MockMPU6050

        # Mocks must implement the Protocols
        self.pz = MockPiconZero()
        self.sensor = MockMPU6050(0x68)

    def init(self) -> None:
        """Initialize the underlying motor driver."""
        self.pz.init()

    def read_imu_raw(self) -> tuple[Vector3, Vector3]:
        """
        Returns raw accelerometer and gyro data.
        :return: Tuple of (accel_dict, gyro_dict).
        """
        return self.sensor.get_accel_data(), self.sensor.get_gyro_data()

    def read_imu_converted(self) -> IMUReading:
        """
        Read IMU and calculate pitch/rates based on config.
        :return: IMUReading object containing pitch angle and rates.
        """
        accel, gyro = self.read_imu_raw()

        if self.gyro_axis == GYRO_AXIS_Y:
            # Pitch around Y axis
            accel_forward = accel["x"]
            accel_vertical = accel["z"]
            raw_gyro_rate = gyro["y"]
        else:
            # Default: Pitch around X axis (Y is forward)
            accel_forward = accel["y"]
            accel_vertical = accel["z"]
            raw_gyro_rate = gyro["x"]

        # Calculate Accelerometer Angle
        acc_angle = calculate_pitch(accel_forward, accel_vertical)

        gyro_rate = raw_gyro_rate
        yaw_rate = gyro["z"]

        if self.gyro_invert:
            acc_angle = -acc_angle
            gyro_rate = -gyro_rate

        return IMUReading(
            pitch_angle=acc_angle, pitch_rate=gyro_rate, yaw_rate=yaw_rate
        )

    def set_motors(self, left: float, right: float) -> None:
        """
        Set motor speeds.
        :param left: Speed -100 to 100
        :param right: Speed -100 to 100
        """
        if self.invert_l:
            left = -left
        if self.invert_r:
            right = -right

        # Use helper clamp, cast to int for driver
        left_val = int(clamp(left, MOTOR_MIN_OUTPUT, MOTOR_MAX_OUTPUT))
        right_val = int(clamp(right, MOTOR_MIN_OUTPUT, MOTOR_MAX_OUTPUT))

        self.pz.set_motor(self.motor_l, left_val)
        self.pz.set_motor(self.motor_r, right_val)

    def stop(self) -> None:
        """Stop all motors."""
        self.pz.stop()

    def cleanup(self) -> None:
        """Cleanup hardware resources."""
        self.pz.cleanup()
