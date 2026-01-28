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
    """
    Immutable data structure for converted IMU readings.
    """

    pitch_angle: float
    """Current pitch angle in degrees (calculated from Accelerometer)."""

    pitch_rate: float
    """Current rate of change of pitch in degrees/second (from Gyroscope)."""

    yaw_rate: float
    """Current rate of change of yaw in degrees/second (from Gyroscope)."""


@runtime_checkable
class MotorDriver(Protocol):
    """Protocol for a Motor Driver (e.g. PiconZero or Mock)."""

    def init(self) -> None: ...
    def cleanup(self) -> None: ...
    def stop(self) -> None: ...
    def set_motor(self, motor: int, value: int) -> None: ...


@runtime_checkable
class IMUDriver(Protocol):
    """Protocol for an IMU Driver (e.g. MPU6050 or Mock)."""

    def get_accel_data(self) -> Vector3: ...
    def get_gyro_data(self) -> Vector3: ...


class MPU6050Adapter:
    """Adapter for the third-party mpu6050 library class to match our Protocol."""

    def __init__(self, sensor_instance):
        self.sensor = sensor_instance

    def get_accel_data(self) -> Vector3:
        return self.sensor.get_accel_data()

    def get_gyro_data(self) -> Vector3:
        return self.sensor.get_gyro_data()


class RobotHardware:
    """
    Hardware Abstraction Layer (HAL).
    Responsible for:
    1. initializing hardware (or mocks).
    2. normalizing sensor data into a standard coordinate system.
    3. normalizing motor commands.
    """

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

        :param motor_l: Channel index for Left Motor.
        :param motor_r: Channel index for Right Motor.
        :param invert_l: If True, reverses Left Motor direction.
        :param invert_r: If True, reverses Right Motor direction.
        :param gyro_axis: The physical axis of the IMU aligned with the robot's pitch ('x' or 'y').
        :param gyro_invert: If True, negates the gyro readings (to match pitch direction).
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
        """
        Initialize hardware components.
        Checks for `MOCK_HARDWARE` env var or import failures to fallback to mocks.
        """
        if os.environ.get("MOCK_HARDWARE"):
            self._init_mock_hardware()
            return

        try:
            from .piconzero import PiconZero
            from mpu6050 import mpu6050

            self.pz = PiconZero()
            self.sensor = MPU6050Adapter(mpu6050(0x68))
            logger.info("Hardware initialized.")
        except (ImportError, OSError):
            logger.warning("Hardware not found. Falling back to Mock Mode.")
            self._init_mock_hardware()

    def _init_mock_hardware(self) -> None:
        """Initialize mock hardware components for testing/simulation."""
        logger.info("Running in Mock Mode")
        from .mocks import MockPiconZero, MockMPU6050

        self.pz = MockPiconZero()
        self.sensor = MockMPU6050(0x68)

    def init(self) -> None:
        """Initialize the underlying motor driver (reset registers etc)."""
        self.pz.init()

    def read_imu_raw(self) -> tuple[Vector3, Vector3]:
        """
        Returns raw accelerometer and gyro data dictionaries.
        :return: Tuple of (accel_dict, gyro_dict).
        """
        return self.sensor.get_accel_data(), self.sensor.get_gyro_data()

    def read_imu_converted(self) -> IMUReading:
        """
        Read IMU and calculate pitch angle and rates based on configuration.
        Handles axis remapping and inversion so the rest of the code only sees "Pitch".

        :return: IMUReading object containing pitch angle (deg) and rates (deg/s).
        """
        accel, gyro = self.read_imu_raw()

        # Coordinate System Mapping
        # We need to determine which axis measures "forward/backward" acceleration
        # and which axis measures "vertical" acceleration to calculate the angle.
        if self.gyro_axis == GYRO_AXIS_Y:
            # Pitch around Y axis means X is Forward, Z is Vertical
            accel_forward = accel["x"]
            accel_vertical = accel["z"]
            raw_gyro_rate = gyro["y"]
        else:
            # Default: Pitch around X axis means Y is Forward, Z is Vertical
            accel_forward = accel["y"]
            accel_vertical = accel["z"]
            raw_gyro_rate = gyro["x"]

        # Calculate Accelerometer Angle using Trigonometry
        acc_angle = calculate_pitch(accel_forward, accel_vertical)

        gyro_rate = raw_gyro_rate
        yaw_rate = gyro["z"]  # Yaw is usually Z-axis

        if self.gyro_invert:
            acc_angle = -acc_angle
            gyro_rate = -gyro_rate

        return IMUReading(
            pitch_angle=acc_angle, pitch_rate=gyro_rate, yaw_rate=yaw_rate
        )

    def set_motors(self, left: float, right: float) -> None:
        """
        Set motor speeds.

        :param left: Speed value from -100 (Full Reverse) to 100 (Full Forward).
        :param right: Speed value from -100 (Full Reverse) to 100 (Full Forward).
        """
        # Apply Hardware Inversion
        if self.invert_l:
            left = -left
        if self.invert_r:
            right = -right

        # Clamp and Cast
        # The underlying driver expects integers between -100 and 100
        left_val = int(clamp(left, MOTOR_MIN_OUTPUT, MOTOR_MAX_OUTPUT))
        right_val = int(clamp(right, MOTOR_MIN_OUTPUT, MOTOR_MAX_OUTPUT))

        self.pz.set_motor(self.motor_l, left_val)
        self.pz.set_motor(self.motor_r, right_val)

    def stop(self) -> None:
        """Stop all motors immediately."""
        self.pz.stop()

    def cleanup(self) -> None:
        """Cleanup hardware resources (e.g. close I2C bus)."""
        self.pz.cleanup()
