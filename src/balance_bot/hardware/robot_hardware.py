import os
import time
import logging
from typing import Protocol, runtime_checkable, Any, Optional
from dataclasses import dataclass

from ..utils import clamp, calculate_pitch, Vector3, get_i2c_failure_report
from ..configuration import (
    BALANCING_THRESHOLD,
    REST_ANGLE_MIN,
    REST_ANGLE_MAX,
    HardwareConfig,
)
from ..enums import Axis

logger = logging.getLogger(__name__)

MOTOR_MIN_OUTPUT = -100
MOTOR_MAX_OUTPUT = 100


@dataclass(frozen=True)
class IMUReading:
    """
    Immutable data structure for converted IMU readings.
    """
    pitch_angle: float
    pitch_rate: float
    yaw_rate: float
    roll_angle: float
    roll_rate: float
    accel_raw: Optional[Vector3] = None
    gyro_raw: Optional[Vector3] = None


@dataclass(frozen=True)
class MeasureResult:
    """
    Result of a drive-and-measure maneuver.
    """
    duration: float
    samples: list[IMUReading]

    @property
    def avg_yaw_rate(self) -> float:
        if not self.samples:
            return 0.0
        return sum(s.yaw_rate for s in self.samples) / len(self.samples)

    @property
    def abs_avg_yaw_rate(self) -> float:
        if not self.samples:
            return 0.0
        return sum(abs(s.yaw_rate) for s in self.samples) / len(self.samples)

    @property
    def max_rate(self) -> float:
        if not self.samples:
            return 0.0
        return max(
            (abs(s.pitch_rate) + abs(s.yaw_rate) + abs(s.roll_rate))
            for s in self.samples
        )

    @property
    def final_pitch(self) -> float:
        if not self.samples:
            return 0.0
        return self.samples[-1].pitch_angle


@runtime_checkable
class MotorDriver(Protocol):
    def init(self) -> None: ...
    def cleanup(self) -> None: ...
    def stop(self) -> None: ...
    def set_retries(self, retries: int) -> None: ...
    def set_motor(self, motor: int, value: int) -> None: ...
    def set_motors(self, motor_0_val: int, motor_1_val: int) -> None: ...


@runtime_checkable
class IMUDriver(Protocol):
    def get_accel_data(self) -> Vector3: ...
    def get_gyro_data(self) -> Vector3: ...


class MPU6050Adapter:
    def __init__(self, sensor_instance: Any):
        self.sensor = sensor_instance

    def get_accel_data(self) -> Vector3:
        return Vector3.from_dict(self.sensor.get_accel_data())

    def get_gyro_data(self) -> Vector3:
        return Vector3.from_dict(self.sensor.get_gyro_data())


class RobotHardware:
    """
    Hardware Abstraction Layer (HAL) for the Robot.
    """

    def __init__(self, config: HardwareConfig):
        """
        Initialize the robot hardware abstraction.
        :param config: The shared HardwareConfig object.
        """
        self.config = config
        self._imu_consecutive_errors = 0

        # Store the "last known good" value
        self._last_accel = Vector3(0.0, 0.0, 0.0)
        self._last_gyro = Vector3(0.0, 0.0, 0.0)

        self.pz: MotorDriver | None = None
        self.sensor: IMUDriver | None = None

        # Resolve defaults locally to ensure runtime safety even with incomplete config
        self.motor_i2c_bus = self.config.motor_i2c_bus if self.config.motor_i2c_bus is not None else 1
        self.imu_i2c_bus = self.config.imu_i2c_bus if self.config.imu_i2c_bus is not None else 1

        self.gyro_pitch_axis = self.config.gyro_pitch_axis or Axis.X
        self.gyro_pitch_invert = self.config.gyro_pitch_invert

        self.gyro_yaw_axis = self.config.gyro_yaw_axis or Axis.Z
        self.gyro_yaw_invert = self.config.gyro_yaw_invert

        self.gyro_roll_axis = self.config.gyro_roll_axis or Axis.Y
        self.gyro_roll_invert = self.config.gyro_roll_invert

        self.accel_vertical_axis = self.config.accel_vertical_axis or Axis.Z
        self.accel_vertical_invert = self.config.accel_vertical_invert

        self.accel_forward_axis = self.config.accel_forward_axis or Axis.Y
        self.accel_forward_invert = self.config.accel_forward_invert

        self._init_hardware()

    @property
    def accel_roll_axis(self) -> Axis | None:
        """Deduce Accel Roll Axis (The one not used by Vertical or Forward)"""
        # Use local resolved attributes
        axes = {Axis.X, Axis.Y, Axis.Z}
        used = {self.accel_vertical_axis, self.accel_forward_axis}
        remaining = axes - used
        if remaining:
            return list(remaining)[0]
        else:
            return Axis.X  # Fallback

    def _get_axis_value(self, vector: Vector3, axis: Axis | None, invert: bool) -> float:
        """Helper to extract and optionally invert a vector component."""
        if axis is None:
            return 0.0
        val = getattr(vector, axis.value)
        return -val if invert else val

    def _init_hardware(self) -> None:
        """
        Initialize hardware components.
        """
        # If running in explicit mock mode via env var, do that first.
        if os.environ.get("ALLOW_MOCK_FALLBACK"):
            logger.warning("Hardware Init: Mock Mode Requested via Environment.")
            self._init_mock_hardware()
            return

        try:
            # 1. Attempt Imports
            try:
                # Shim smbus for mpu6050
                import sys
                import smbus2
                sys.modules['smbus'] = smbus2

                from .piconzero import PiconZero
                from mpu6050 import mpu6050
            except (ImportError, OSError) as e:
                logger.error(f"CRITICAL: Required libraries not found or failed to load: {e}")
                raise e

            # 2. Attempt PiconZero
            try:
                self.pz = PiconZero(bus_number=self.motor_i2c_bus)
            except (OSError, PermissionError, FileNotFoundError) as e:
                logger.error(f"CRITICAL: PiconZero Init Failed on Bus {self.motor_i2c_bus}: {e}")
                report = get_i2c_failure_report(self.motor_i2c_bus, 0x22, "PiconZero")
                logger.error(report)
                raise e

            # 3. Attempt MPU6050
            try:
                self.sensor = MPU6050Adapter(mpu6050(0x68, bus=self.imu_i2c_bus))
            except OSError as e:
                logger.error(f"CRITICAL: MPU6050 Init Failed on Bus {self.imu_i2c_bus}: {e}")
                report = get_i2c_failure_report(self.imu_i2c_bus, 0x68, "MPU6050")
                logger.error(report)
                raise e

            logger.info(f"Hardware initialized. PiconZero={self.motor_i2c_bus}, MPU6050={self.imu_i2c_bus}.")

        except (ImportError, OSError, PermissionError, FileNotFoundError) as e:
             # Check for Fallback (if initialization failed)
            if os.environ.get("ALLOW_MOCK_FALLBACK"):
                logger.warning("Hardware Init Failed. Falling back to MOCKS as requested.")
                self._init_mock_hardware()
                return
            raise e

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
        """
        if self.sensor is None:
            raise RuntimeError("IMU Sensor not initialized")

        try:
            # Try to read fresh data
            accel = self.sensor.get_accel_data()
            gyro = self.sensor.get_gyro_data()

            # Update cache
            self._last_accel = accel
            self._last_gyro = gyro

            # Reset error counter on success
            self._imu_consecutive_errors = 0

            return accel, gyro

        except OSError:
            self._imu_consecutive_errors += 1
            if self._imu_consecutive_errors > self.config.imu_max_retries:
                logger.error(f"IMU Failed {self._imu_consecutive_errors} times in a row. Raising Error.")
                raise

            return self._last_accel, self._last_gyro

    def read_imu_converted(self) -> IMUReading:
        """
        Read IMU and calculate pitch/rates based on config.
        """
        accel, gyro = self.read_imu_raw()

        # Get raw values based on local attributes (resolved defaults)
        accel_forward = self._get_axis_value(
            accel, self.accel_forward_axis, self.accel_forward_invert
        )
        accel_vertical = self._get_axis_value(
            accel, self.accel_vertical_axis, self.accel_vertical_invert
        )

        # Calculate Accelerometer Angle
        acc_angle = calculate_pitch(accel_forward, accel_vertical)

        # Gyro Rates
        gyro_rate = self._get_axis_value(
            gyro, self.gyro_pitch_axis, self.gyro_pitch_invert
        )
        yaw_rate = self._get_axis_value(
            gyro, self.gyro_yaw_axis, self.gyro_yaw_invert
        )
        roll_rate = self._get_axis_value(
            gyro, self.gyro_roll_axis, self.gyro_roll_invert
        )

        # Roll Angle (Approximate from Accel)
        roll_axis = self.accel_roll_axis
        if roll_axis:
            accel_roll = getattr(accel, roll_axis.value)
            roll_angle = calculate_pitch(accel_roll, accel_vertical)
        else:
            roll_angle = 0.0

        return IMUReading(
            pitch_angle=acc_angle,
            pitch_rate=gyro_rate,
            yaw_rate=yaw_rate,
            roll_angle=roll_angle,
            roll_rate=roll_rate,
            accel_raw=accel,
            gyro_raw=gyro
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
        if self.pz is None:
            raise RuntimeError("Motor Driver not initialized")

        # Apply Motor Trim
        if self.config.motor_trim > 0:
            right *= (1.0 - self.config.motor_trim)
        elif self.config.motor_trim < 0:
            left *= (1.0 - abs(self.config.motor_trim))

        if self.config.motor_l_invert:
            left = -left
        if self.config.motor_r_invert:
            right = -right

        # Use helper clamp, cast to int for driver
        left_val = int(clamp(left, MOTOR_MIN_OUTPUT, MOTOR_MAX_OUTPUT))
        right_val = int(clamp(right, MOTOR_MIN_OUTPUT, MOTOR_MAX_OUTPUT))

        # Map logical Left/Right to Physical 0/1
        val_0 = 0
        val_1 = 0

        # Assign Left Motor Value
        if self.config.motor_l is not None:
            if self.config.motor_l == 0:
                val_0 = left_val
            elif self.config.motor_l == 1:
                val_1 = left_val

        # Assign Right Motor Value
        if self.config.motor_r is not None:
            if self.config.motor_r == 0:
                val_0 = right_val
            elif self.config.motor_r == 1:
                val_1 = right_val

        self.pz.set_motors(val_0, val_1)

    def stop(self) -> None:
        """Stop all motors."""
        self.pz.stop()

    def cleanup(self) -> None:
        """Cleanup hardware resources."""
        self.pz.cleanup()

    def get_posture_state(self) -> str:
        """
        Determine the robot's current posture state based on pitch.
        """
        reading = self.read_imu_converted()
        pitch = abs(reading.pitch_angle)

        if pitch < BALANCING_THRESHOLD:
            return "BALANCED"
        elif REST_ANGLE_MIN < pitch < REST_ANGLE_MAX:
            return "RESTING"
        elif pitch > self.config.crash_angle:
            return "CRASHED"
        else:
            return "FALLING"

    def wait_for_stability(self, duration: float = 2.0, threshold: float = 2.0) -> None:
        """
        Wait for the robot to be stable (gyro rates low) for a duration.
        """
        logger.info(f"Waiting for stability (rates < {threshold} deg/s) for {duration}s...")

        start_stable_time = None
        last_log = 0.0

        while True:
            try:
                accel, gyro = self.read_imu_raw()
            except Exception as e:
                logger.warning(f"  [Error reading IMU] {e}")
                time.sleep(0.1)
                continue

            rate = abs(gyro.x) + abs(gyro.y) + abs(gyro.z)

            if rate < threshold:
                if start_stable_time is None:
                    start_stable_time = time.time()
                elif time.time() - start_stable_time >= duration:
                    logger.info("  [STABLE] Robot is still.")
                    return
            else:
                if start_stable_time is not None:
                    start_stable_time = None
                    if time.time() - last_log > 1.0:
                        logger.debug(f"  [MOVING] Rate {rate:.1f} > {threshold}. Waiting...")
                        last_log = time.time()

            time.sleep(0.05)

    def execute_maneuver(self, steps: list[tuple[float, float, float]], sample_interval: float = 0.01) -> MeasureResult:
        """
        Execute a sequence of motor commands and collect IMU readings throughout.
        """
        samples = []
        total_duration = 0.0

        try:
            for left, right, duration in steps:
                self.set_motors(left, right)
                step_start = time.time()

                while time.time() - step_start < duration:
                    samples.append(self.read_imu_converted())
                    time.sleep(sample_interval)

                total_duration += duration
        finally:
            self.stop()

        return MeasureResult(duration=total_duration, samples=samples)

    def drive_and_measure(self, left_power: float, right_power: float, duration: float, sample_interval: float = 0.01) -> MeasureResult:
        return self.execute_maneuver([(left_power, right_power, duration)], sample_interval)
