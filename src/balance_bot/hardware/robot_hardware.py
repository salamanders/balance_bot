import os
import time
import logging
from typing import Protocol, runtime_checkable, Any, Optional
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
    :param roll_angle: Calculated roll angle in degrees.
    :param roll_rate: Angular velocity around roll axis in deg/s.
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
    Encapsulates raw samples and derived statistics.
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

    def set_motors(self, motor_0_val: int, motor_1_val: int) -> None:
        """
        Set speed for both motors simultaneously using a block write.
        :param motor_0_val: Speed for Motor 0 (-100 to 100).
        :param motor_1_val: Speed for Motor 1 (-100 to 100).
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
        return Vector3.from_dict(self.sensor.get_accel_data())

    def get_gyro_data(self) -> Vector3:
        """Get gyroscope data."""
        return Vector3.from_dict(self.sensor.get_gyro_data())


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
        motor_l: int | None = None,
        motor_r: int | None = None,
        invert_l: bool = False,
        invert_r: bool = False,
        gyro_axis: Axis | None = None,
        gyro_invert: bool = False,
        gyro_yaw_axis: Axis | None = None,
        gyro_yaw_invert: bool = False,
        gyro_roll_axis: Axis | None = None,
        gyro_roll_invert: bool = False,
        accel_vertical_axis: Axis | None = None,
        accel_vertical_invert: bool = False,
        accel_forward_axis: Axis | None = None,
        accel_forward_invert: bool = False,
        motor_i2c_bus: int | None = None,
        imu_i2c_bus: int | None = None,
        crash_angle: float = 60.0,
        imu_max_retries: int = 5,
        motor_trim: float = 0.0,
    ):
        """
        Initialize the robot hardware abstraction.

        :param motor_l: Left motor channel index.
        :param motor_r: Right motor channel index.
        :param invert_l: Whether to invert left motor direction.
        :param invert_r: Whether to invert right motor direction.
        :param gyro_axis: Axis used for pitch rotation.
        :param gyro_invert: Whether to invert gyro reading sign.
        :param gyro_yaw_axis: Axis used for yaw rotation.
        :param gyro_yaw_invert: Whether to invert yaw reading sign.
        :param gyro_roll_axis: Axis used for roll rotation.
        :param gyro_roll_invert: Whether to invert roll reading sign.
        :param accel_vertical_axis: Axis corresponding to gravity.
        :param accel_vertical_invert: Invert vertical axis sign.
        :param accel_forward_axis: Axis corresponding to forward motion.
        :param accel_forward_invert: Invert forward axis sign.
        :param motor_i2c_bus: I2C bus number for Motor Driver.
        :param imu_i2c_bus: I2C bus number for IMU.
        :param crash_angle: Angle to consider as CRASHED state.
        :param imu_max_retries: Max consecutive IMU failures before raising error.
        :param motor_trim: Motor output scaling (Pos=Reduce Right, Neg=Reduce Left).
        """
        self.motor_l = motor_l
        self.motor_r = motor_r
        self.invert_l = invert_l
        self.invert_r = invert_r
        self.gyro_axis = gyro_axis
        self.gyro_invert = gyro_invert
        self.gyro_yaw_axis = gyro_yaw_axis
        self.gyro_yaw_invert = gyro_yaw_invert
        self.gyro_roll_axis = gyro_roll_axis
        self.gyro_roll_invert = gyro_roll_invert
        self.accel_vertical_axis = accel_vertical_axis
        self.accel_vertical_invert = accel_vertical_invert
        self.accel_forward_axis = accel_forward_axis
        self.accel_forward_invert = accel_forward_invert
        self.motor_i2c_bus = motor_i2c_bus
        self.imu_i2c_bus = imu_i2c_bus
        self.crash_angle = crash_angle
        self.imu_max_retries = imu_max_retries
        self.motor_trim = motor_trim
        self._imu_consecutive_errors = 0

        # Deduce Accel Roll Axis (The one not used by Vertical or Forward)
        if accel_vertical_axis and accel_forward_axis:
            axes = {Axis.X, Axis.Y, Axis.Z}
            used = {accel_vertical_axis, accel_forward_axis}
            remaining = axes - used
            if remaining:
                self.accel_roll_axis = list(remaining)[0]
            else:
                self.accel_roll_axis = Axis.X  # Fallback
        else:
            self.accel_roll_axis = None

        # Store the "last known good" value
        self._last_accel = Vector3(0.0, 0.0, 0.0)
        self._last_gyro = Vector3(0.0, 0.0, 0.0)

        self.pz: MotorDriver | None = None
        self.sensor: IMUDriver | None = None

        self._init_hardware()

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
                from .piconzero import PiconZero
                from mpu6050 import mpu6050
            except (ImportError, OSError) as e:
                logger.error(f"CRITICAL: Required libraries not found or failed to load: {e}")
                raise e

            # 2. Attempt PiconZero (if bus known)
            if self.motor_i2c_bus is not None:
                try:
                    self.pz = PiconZero(bus_number=self.motor_i2c_bus)
                except (OSError, PermissionError, FileNotFoundError) as e:
                    logger.error(f"CRITICAL: PiconZero Init Failed on Bus {self.motor_i2c_bus}: {e}")
                    report = get_i2c_failure_report(self.motor_i2c_bus, 0x22, "PiconZero")
                    logger.error(report)
                    raise e
            else:
                logger.info("Skipping PiconZero init (Bus Unknown)")

            # 3. Attempt MPU6050 (if bus known)
            if self.imu_i2c_bus is not None:
                try:
                    self.sensor = MPU6050Adapter(mpu6050(0x68, bus=self.imu_i2c_bus))
                except OSError as e:
                    logger.error(f"CRITICAL: MPU6050 Init Failed on Bus {self.imu_i2c_bus}: {e}")
                    report = get_i2c_failure_report(self.imu_i2c_bus, 0x68, "MPU6050")
                    logger.error(report)
                    raise e
            else:
                logger.info("Skipping MPU6050 init (Bus Unknown)")

            logger.info(f"Hardware initialized (Partial). PiconZero={self.motor_i2c_bus}, MPU6050={self.imu_i2c_bus}.")

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
        Includes error handling for I2C noise.
        :return: Tuple of (accel_dict, gyro_dict).
        """
        if self.sensor is None:
            raise RuntimeError("IMU Sensor not initialized (Bus Unknown?)")

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
            if self._imu_consecutive_errors > self.imu_max_retries:
                logger.error(f"IMU Failed {self._imu_consecutive_errors} times in a row. Raising Error.")
                raise

            # If I2C fails (noise), return the last known good values
            # This prevents the robot from crashing or freezing
            # logger.warning("I2C Glitch - Using stale data")
            return self._last_accel, self._last_gyro

    def read_imu_converted(self) -> IMUReading:
        """
        Read IMU and calculate pitch/rates based on config.
        """
        if (
            self.accel_forward_axis is None
            or self.accel_vertical_axis is None
            or self.gyro_axis is None
        ):
            raise RuntimeError("IMU axes not configured. Use read_imu_raw() instead.")

        accel, gyro = self.read_imu_raw()

        # Get raw values based on config
        accel_forward = getattr(accel, self.accel_forward_axis.value)
        accel_vertical = getattr(accel, self.accel_vertical_axis.value)
        gyro_rate = getattr(gyro, self.gyro_axis.value)

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

        # Yaw rate
        # Default to 0.0 if not configured
        if self.gyro_yaw_axis:
            yaw_rate = getattr(gyro, self.gyro_yaw_axis.value)
            if self.gyro_yaw_invert:
                yaw_rate = -yaw_rate
        else:
            yaw_rate = 0.0

        # Roll rate
        if self.gyro_roll_axis:
            roll_rate = getattr(gyro, self.gyro_roll_axis.value)
            if self.gyro_roll_invert:
                roll_rate = -roll_rate
        else:
            roll_rate = 0.0

        # Roll Angle (Approximate from Accel)
        if self.accel_roll_axis:
            accel_roll = getattr(accel, self.accel_roll_axis.value)
            # Calculate angle of side vector relative to vertical
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
            # If not initialized, maybe we can't drive?
            # Or should we raise?
            # The user might be calling this from "Twitch" which knows the bus...
            # But Twitch should probably use pz.set_motor directly if it's doing raw stuff.
            # But if we want to use the high level abstraction, we expect pz to be there.
            raise RuntimeError("Motor Driver not initialized (Bus Unknown?)")

        # Apply Motor Trim (Compensation for mismatched motors)
        # Trim > 0: Scale down Right Motor (Right is stronger)
        # Trim < 0: Scale down Left Motor (Left is stronger)
        if self.motor_trim > 0:
            right *= (1.0 - self.motor_trim)
        elif self.motor_trim < 0:
            left *= (1.0 - abs(self.motor_trim))

        if self.invert_l:
            left = -left
        if self.invert_r:
            right = -right

        # Use helper clamp, cast to int for driver
        left_val = int(clamp(left, MOTOR_MIN_OUTPUT, MOTOR_MAX_OUTPUT))
        right_val = int(clamp(right, MOTOR_MIN_OUTPUT, MOTOR_MAX_OUTPUT))

        # Map logical Left/Right to Physical 0/1
        val_0 = 0
        val_1 = 0

        # Assign Left Motor Value
        if self.motor_l is not None:
            if self.motor_l == 0:
                val_0 = left_val
            elif self.motor_l == 1:
                val_1 = left_val

        # Assign Right Motor Value
        if self.motor_r is not None:
            if self.motor_r == 0:
                val_0 = right_val
            elif self.motor_r == 1:
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

    def wait_for_stability(self, duration: float = 2.0, threshold: float = 2.0) -> None:
        """
        Wait for the robot to be stable (gyro rates low) for a duration.
        Blocks until the condition is met.
        """
        logger.info(f"Waiting for stability (rates < {threshold} deg/s) for {duration}s...")

        start_stable_time = None
        last_log = 0.0

        while True:
            # Read sensors
            try:
                reading = self.read_imu_converted()
            except RuntimeError:
                # Re-raise if config is missing, as we can't measure stability
                raise
            except Exception as e:
                logger.warning(f"  [Error reading IMU] {e}")
                time.sleep(0.1)
                continue

            # Calculate total rate magnitude
            rate = abs(reading.pitch_rate) + abs(reading.yaw_rate) + abs(reading.roll_rate)

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

        :param steps: List of (left_power, right_power, duration) tuples.
        :param sample_interval: Time between IMU samples.
        :return: MeasureResult containing total duration and all samples.
        """
        samples = []
        total_duration = 0.0

        try:
            for left, right, duration in steps:
                self.set_motors(left, right)
                step_start = time.time()

                # Run for the duration of this step
                while time.time() - step_start < duration:
                    samples.append(self.read_imu_converted())
                    time.sleep(sample_interval)

                total_duration += duration
        finally:
            self.stop()

        return MeasureResult(duration=total_duration, samples=samples)

    def drive_and_measure(self, left_power: float, right_power: float, duration: float, sample_interval: float = 0.01) -> MeasureResult:
        """
        Drive motors for a duration and collect IMU readings.
        Returns a MeasureResult object containing samples and stats.
        """
        return self.execute_maneuver([(left_power, right_power, duration)], sample_interval)
