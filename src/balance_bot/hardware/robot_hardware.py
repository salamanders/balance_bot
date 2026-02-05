import os
import logging
from typing import Protocol, runtime_checkable, Any
from dataclasses import dataclass

from ..utils import clamp, calculate_pitch, Vector3
from ..diagnostics import get_i2c_failure_report
from ..config import (
    BALANCING_THRESHOLD,
    REST_ANGLE_MIN,
    REST_ANGLE_MAX,
)
from ..enums import Axis

logger = logging.getLogger(__name__)

MOTOR_MIN_OUTPUT = -100
MOTOR_MAX_OUTPUT = 100


@dataclass(frozen=True)
class IMUReading:
    """
    Immutable data structure for converted IMU readings.

    :param pitch_angle: Calculated pitch angle in degrees (Zero = Upright).
    :param pitch_rate: Angular velocity around pitch axis in deg/s.
    :param yaw_rate: Angular velocity around yaw (vertical) axis in deg/s.
    """
    pitch_angle: float
    pitch_rate: float
    yaw_rate: float


@runtime_checkable
class MotorDriver(Protocol):
    """
    Protocol for motor driver implementations.
    Allows swapping between real hardware (PiconZero) and Mocks.
    """

    def init(self) -> None:
        """Initialize the motor driver hardware."""
        ...

    def cleanup(self) -> None:
        """Release hardware resources."""
        ...

    def stop(self) -> None:
        """Stop all motors immediately."""
        ...

    def set_retries(self, retries: int) -> None:
        """Set the number of I2C retries."""
        ...

    def set_motor(self, motor: int, value: int) -> None:
        """
        Set speed for a specific motor.
        :param motor: Motor channel index (0 or 1).
        :param value: Speed (-100 to 100).
        """
        ...


@runtime_checkable
class IMUDriver(Protocol):
    """
    Protocol for IMU driver implementations.
    Abstracts specific sensor libraries (e.g. mpu6050).
    """

    def get_accel_data(self) -> Vector3:
        """
        Get raw accelerometer data.
        :return: Dictionary with x, y, z keys.
        """
        ...

    def get_gyro_data(self) -> Vector3:
        """
        Get raw gyroscope data.
        :return: Dictionary with x, y, z keys.
        """
        ...


class MPU6050Adapter:
    """
    Adapter for the mpu6050 library class to match IMUDriver protocol.
    """

    def __init__(self, sensor_instance: Any):
        """
        Initialize the adapter.
        :param sensor_instance: Instance of mpu6050 class.
        """
        self.sensor = sensor_instance

    def get_accel_data(self) -> Vector3:
        """Get accelerometer data."""
        return self.sensor.get_accel_data()

    def get_gyro_data(self) -> Vector3:
        """Get gyroscope data."""
        return self.sensor.get_gyro_data()


class RobotHardware:
    """
    Hardware Abstraction Layer (HAL) for the Robot.

    Responsibilities:
     - Initialize Motor and IMU drivers (Real or Mock).
     - Abstract away I2C bus management.
     - Remap sensor axes (X/Y/Z) to logical axes (Pitch/Vertical/Forward).
     - Convert raw sensor data into useful engineering units (Degrees, Deg/s).
    """

    def __init__(
        self,
        motor_l: int,
        motor_r: int,
        invert_l: bool = False,
        invert_r: bool = False,
        gyro_axis: Axis = Axis.X,
        gyro_invert: bool = False,
        accel_vertical_axis: Axis = Axis.Z,
        accel_vertical_invert: bool = False,
        accel_forward_axis: Axis = Axis.Y,
        accel_forward_invert: bool = False,
        i2c_bus: int = 1,
        crash_angle: float = 60.0,
    ):
        """
        Initialize the robot hardware abstraction.

        :param motor_l: Left motor channel index.
        :param motor_r: Right motor channel index.
        :param invert_l: Whether to invert left motor direction.
        :param invert_r: Whether to invert right motor direction.
        :param gyro_axis: Axis used for pitch rotation.
        :param gyro_invert: Whether to invert gyro reading sign.
        :param accel_vertical_axis: Axis corresponding to gravity.
        :param accel_vertical_invert: Invert vertical axis sign.
        :param accel_forward_axis: Axis corresponding to forward motion.
        :param accel_forward_invert: Invert forward axis sign.
        :param i2c_bus: I2C bus number for IMU (default 1).
        :param crash_angle: Angle to consider as CRASHED state.
        """
        self.motor_l = motor_l
        self.motor_r = motor_r
        self.invert_l = invert_l
        self.invert_r = invert_r
        self.gyro_axis = gyro_axis
        self.gyro_invert = gyro_invert
        self.accel_vertical_axis = accel_vertical_axis
        self.accel_vertical_invert = accel_vertical_invert
        self.accel_forward_axis = accel_forward_axis
        self.accel_forward_invert = accel_forward_invert
        self.i2c_bus = i2c_bus
        self.crash_angle = crash_angle

        self.pz: MotorDriver
        self.sensor: IMUDriver

        self._init_hardware()

    def _init_hardware(self) -> None:
        """
        Initialize hardware components.

        Strategy:
        1. Check MOCK_HARDWARE env var.
        2. Try importing hardware libraries.
        3. Try initializing PiconZero (Motors).
        4. Try initializing MPU6050 (IMU).

        If any step fails, log a critical error, generate a diagnostic report,
        and fall back to Mock Hardware to prevent crash loop.
        """
        if os.environ.get("MOCK_HARDWARE"):
            self._init_mock_hardware()
            return

        # 1. Attempt Imports
        try:
            from .piconzero import PiconZero
            from mpu6050 import mpu6050
        except ImportError as e:
            logger.error(f"CRITICAL: Required libraries not found: {e}")
            logger.error("Try running 'uv sync' or check your virtual environment.")
            self._init_mock_hardware()
            return

        # 2. Attempt PiconZero (Bus 1 assumed for HAT)
        try:
            # PiconZero defaults to Bus 1. We assume the HAT is on Bus 1.
            self.pz = PiconZero()
        except (OSError, PermissionError, FileNotFoundError) as e:
            logger.error(f"CRITICAL: PiconZero Init Failed: {e}")

            # Generate pessimistic report for PiconZero (0x22 on Bus 1)
            report = get_i2c_failure_report(1, 0x22, "PiconZero")
            logger.error(report)

            self._init_mock_hardware()
            return

        # 3. Attempt MPU6050
        try:
            self.sensor = MPU6050Adapter(mpu6050(0x68, bus=self.i2c_bus))
            logger.info(f"Hardware initialized. MPU6050 on bus {self.i2c_bus}.")
        except OSError as e:
            logger.error(f"CRITICAL: MPU6050 Init Failed on Bus {self.i2c_bus}: {e}")

            # Generate pessimistic report for MPU6050 (0x68 on configured bus)
            report = get_i2c_failure_report(self.i2c_bus, 0x68, "MPU6050")
            logger.error(report)

            self._init_mock_hardware()
            return

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

        Process:
        1. Read raw Accel/Gyro.
        2. Map Raw Axes -> Logical Axes (Forward, Vertical).
        3. Apply Inversions.
        4. Calculate Angle via atan2(forward, vertical).
        5. Return IMUReading.

        :return: IMUReading object containing pitch angle and rates.
        """
        accel, gyro = self.read_imu_raw()

        # Get raw values based on config
        accel_forward = accel[self.accel_forward_axis]
        accel_vertical = accel[self.accel_vertical_axis]
        gyro_rate = gyro[self.gyro_axis]

        # Apply inversions
        if self.accel_forward_invert:
            accel_forward = -accel_forward
        if self.accel_vertical_invert:
            accel_vertical = -accel_vertical

        # Calculate Accelerometer Angle
        # calculate_pitch(y, z) assumes y is forward, z is vertical.
        acc_angle = calculate_pitch(accel_forward, accel_vertical)

        # Apply Gyro Inversion
        if self.gyro_invert:
            gyro_rate = -gyro_rate

        # Yaw rate - For now, assume Yaw is rotation around vertical axis
        yaw_rate = gyro[self.accel_vertical_axis]
        if self.accel_vertical_invert:
            yaw_rate = -yaw_rate

        return IMUReading(
            pitch_angle=acc_angle, pitch_rate=gyro_rate, yaw_rate=yaw_rate
        )

    def set_motor_retries(self, retries: int) -> None:
        """Set the I2C retry count for the motor driver."""
        self.pz.set_retries(retries)

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

    def get_posture_state(self) -> str:
        """
        Determine the robot's current posture state based on pitch.
        States:
         - BALANCED: Upright within operating range.
         - RESTING: Leaning on struts/training wheels.
         - CRASHED: Fallen completely over (hard stop required).
         - FALLING: In transition (optional).
        """
        reading = self.read_imu_converted()
        pitch = abs(reading.pitch_angle)

        if pitch < BALANCING_THRESHOLD:
            return "BALANCED"
        elif REST_ANGLE_MIN < pitch < REST_ANGLE_MAX:
            return "RESTING"
        elif pitch > self.crash_angle:
            return "CRASHED"
        else:
            return "FALLING"
