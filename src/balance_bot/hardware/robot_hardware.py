"""
# System Context
This module is part of the `balance_bot` application, designed to control a self-balancing
homebrew robot. It relies on a deterministic, high-frequency control loop and pessimistic hardware interactions.

# Business Rules
- Fail-fast initialization: The system must crash loudly if physical hardware is missing or unresponsive during boot.
- Fault-tolerant control loop: Once Tier 1 is running (e.g., `BalanceCore`), transient I/O errors must not collapse the system; use continuous data quality metrics instead of fatal exceptions.
- Physical pessimism: Never hardcode physical constants; rely on zero-knowledge self-discovery to deduce configuration.

# Dependency Maps
- Relies on internal configuration (`HardwareConfig`, `LearningState`).
- Interfaces with Tier 1 (`BalanceCore`), Tier 3 (`Agent`), and physical hardware abstraction (`RobotHardware`).
"""

import icontract

import contextlib
import logging
import os
import threading
import time
from collections import deque
from typing import Protocol, runtime_checkable, Any

import glm

from .types import IMUReading, DriveCommand, MeasureResult
from ..configuration import (
    BALANCING_THRESHOLD,
    REST_ANGLE_MIN,
    REST_ANGLE_MAX,
    HardwareConfig,
    LearningState
)
from ..enums import Axis
from ..utils import clamp, calculate_pitch, get_i2c_failure_report, average_vector
from ..watchdog import SurvivalWatchdog

logger = logging.getLogger(__name__)

MOTOR_MIN_OUTPUT = -100
MOTOR_MAX_OUTPUT = 100


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

    def get_accel_data(self) -> glm.vec3:
        """
        Get raw accelerometer data.
        :return: glm.vec3.
        """
        ...

    def get_gyro_data(self) -> glm.vec3:
        """
        Get raw gyroscope data.
        :return: glm.vec3.
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
        # Open /dev/null once for the lifecycle of the adapter
        # This avoids opening/closing files 200 times a second in the 100Hz control loop
        self._devnull = open(os.devnull, 'w')

    def __del__(self) -> None:
        """Cleanup the file handle when the adapter is destroyed."""
        if hasattr(self, '_devnull') and not self._devnull.closed:
            self._devnull.close()

    def get_accel_data(self) -> glm.vec3:
        """Get accelerometer data."""
        with contextlib.redirect_stdout(self._devnull):
            d = self.sensor.get_accel_data()
        return glm.vec3(d["x"], d["y"], d["z"])

    def get_gyro_data(self) -> glm.vec3:
        """Get gyroscope data."""
        with contextlib.redirect_stdout(self._devnull):
            d = self.sensor.get_gyro_data()
        return glm.vec3(d["x"], d["y"], d["z"])


class RobotHardware:
    """
    Hardware Abstraction Layer (HAL) for the Robot.

    Responsibilities:
     - Initialize Motor and IMU drivers (Real or Mock).
     - Abstract away I2C bus management.
     - Remap sensor axes (X/Y/Z) to logical axes (Pitch/Vertical/Forward).
     - Convert raw sensor data into useful engineering units (Degrees, Deg/s).
    """

    def __init__(self, hw_config: HardwareConfig, learning_state: LearningState,
                 watchdog: SurvivalWatchdog | None = None):
        """
        Initialize the robot hardware abstraction.
        :param hw_config: The immutable HardwareConfig object.
        :param learning_state: The mutable LearningState object.
        :param watchdog: Optional SurvivalWatchdog for heartbeat pulses.
        """
        self.hw_config = hw_config
        self.learning_state = learning_state
        self.watchdog = watchdog
        self._imu_consecutive_errors = 0

        # Store the "last known good" value
        self._last_accel = glm.vec3(0.0)
        self._last_gyro = glm.vec3(0.0)

        # Threading for IMU Sensor
        self._sensor_thread: threading.Thread | None = None
        self._sensor_running = False
        self._sensor_lock = threading.Lock()

        self.pz: MotorDriver | None = None
        self.sensor: IMUDriver | None = None

        self.initialize_drivers()
        self.start_sensor_thread()

    def start_sensor_thread(self) -> None:
        if self.sensor is None:
            return

        self._sensor_running = True
        self._sensor_thread = threading.Thread(target=self._sensor_worker, daemon=True)
        self._sensor_thread.start()
        logger.info("IMU SensorThread started.")

    def _sensor_worker(self) -> None:
        if self.sensor is None:
            return
        while self._sensor_running:
            try:
                accel = self.sensor.get_accel_data()
                gyro = self.sensor.get_gyro_data()

                # Apply Bias Calibration
                bias_vec = glm.vec3(
                    self.learning_state.gyro_bias_x,
                    self.learning_state.gyro_bias_y,
                    self.learning_state.gyro_bias_z
                )
                gyro = gyro - bias_vec

                with self._sensor_lock:
                    self._last_accel = accel
                    self._last_gyro = gyro
                    self._imu_consecutive_errors = 0

            except OSError as e:
                if self._imu_consecutive_errors >= self.hw_config.imu_max_retries:
                    logger.warning(f"IMU read failed: {e}")
                with self._sensor_lock:
                    self._imu_consecutive_errors += 1
                if self._imu_consecutive_errors > self.hw_config.imu_max_retries * 10:
                    logger.error("IMU completely dead. Stopping sensor thread.")
                    break

            # Give a very tiny sleep to avoid absolute 100% core starvation
            time.sleep(0.005)

    def stop_sensor_thread(self) -> None:
        self._sensor_running = False
        if self._sensor_thread and self._sensor_thread.is_alive():
            self._sensor_thread.join(timeout=1.0)

    def apply_config(self, new_config: HardwareConfig) -> None:
        """Safely apply a new config without tearing down the object or I2C buses."""
        self.hw_config = new_config

    @property
    def accel_roll_axis(self) -> Axis | None:
        """Deduce Accel Roll Axis (The one not used by Vertical or Forward)"""
        if self.hw_config.accel_vertical_axis and self.hw_config.accel_forward_axis:
            axes = {Axis.X, Axis.Y, Axis.Z}
            used = {self.hw_config.accel_vertical_axis, self.hw_config.accel_forward_axis}
            remaining = axes - used
            if remaining:
                return list(remaining)[0]
            else:
                return Axis.X  # Fallback
        return None

    @staticmethod
    def get_axis_value(vector: glm.vec3, axis: Axis | None, invert: bool) -> float:
        """Helper to extract and optionally invert a vector component."""
        if axis is None:
            raise RuntimeError("CRITICAL: HAL attempted to read an unmapped sensor axis.")
        val = getattr(vector, axis.value)
        return -val if invert else val

    def get_mapped_value(self, vector: glm.vec3, config_name: str) -> float:
        """
        Helper to extract a value based on a config prefix.
        e.g. config_name="accel_forward" -> uses self.hw_config.accel_forward_axis
        """
        axis = getattr(self.hw_config, f"{config_name}_axis")
        invert = getattr(self.hw_config, f"{config_name}_invert")
        return RobotHardware.get_axis_value(vector, axis, bool(invert))

    def initialize_drivers(self) -> None:
        """
        Initialize hardware components.
        """
        if self.pz is not None and self.sensor is not None:
            return

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

            # 2. Attempt PiconZero (if bus known)
            if self.hw_config.motor_i2c_bus is not None:
                try:
                    self.pz = PiconZero(bus_number=self.hw_config.motor_i2c_bus)
                except (OSError, PermissionError, FileNotFoundError) as e:
                    logger.error(f"CRITICAL: PiconZero Init Failed on Bus {self.hw_config.motor_i2c_bus}: {e}")
                    report = get_i2c_failure_report(self.hw_config.motor_i2c_bus, 0x22, "PiconZero")
                    logger.error(report)
                    raise e
            else:
                logger.info("Skipping PiconZero init (Bus Unknown)")

            # 3. Attempt MPU6050 (if bus known)
            if self.hw_config.imu_i2c_bus is not None:
                try:
                    self.sensor = MPU6050Adapter(mpu6050(0x68, bus=self.hw_config.imu_i2c_bus))
                except OSError as e:
                    logger.error(f"CRITICAL: MPU6050 Init Failed on Bus {self.hw_config.imu_i2c_bus}: {e}")
                    report = get_i2c_failure_report(self.hw_config.imu_i2c_bus, 0x68, "MPU6050")
                    logger.error(report)
                    raise e
            else:
                logger.info("Skipping MPU6050 init (Bus Unknown)")

            logger.info(
                f"Hardware initialized (Partial). PiconZero={self.hw_config.motor_i2c_bus}, MPU6050={self.hw_config.imu_i2c_bus}.")

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
        if self.pz is not None:
            self.pz.init()

    def read_imu_raw(self) -> tuple[glm.vec3, glm.vec3]:
        """
        Returns raw accelerometer and gyro data instantly from the background thread buffer.
        :return: Tuple of (accel_dict, gyro_dict).
        """
        if self.sensor is None:
            raise RuntimeError("IMU Sensor not initialized (Bus Unknown?)")

        # Synchronous fallback for tests when sensor is mocked or thread isn't running
        if not self._sensor_running:
            try:
                accel = self.sensor.get_accel_data()
                gyro = self.sensor.get_gyro_data()

                bias_vec = glm.vec3(
                    self.learning_state.gyro_bias_x,
                    self.learning_state.gyro_bias_y,
                    self.learning_state.gyro_bias_z
                )
                gyro = gyro - bias_vec

                self._last_accel = accel
                self._last_gyro = gyro
                self._imu_consecutive_errors = 0
            except OSError as e:
                if self._imu_consecutive_errors >= self.hw_config.imu_max_retries:
                    logger.warning(f"IMU read failed: {e}")
                self._imu_consecutive_errors += 1

        with self._sensor_lock:
            accel = self._last_accel
            gyro = self._last_gyro
            errors = self._imu_consecutive_errors

        if errors > self.hw_config.imu_max_retries:
            logger.error(
                f"IMU Failed {errors} times in a row. Returning cached data indefinitely to avoid fatal crash.")
        elif errors > 0:
            logger.debug(f"IMU Glitch ({errors}/{self.hw_config.imu_max_retries}). Using cached data.")

        return accel, gyro

    def read_imu_converted(self) -> IMUReading:
        """
        Read IMU and calculate pitch/rates based on config.
        If axes are not configured, returns raw data with zeroed angles/rates.
        """
        accel, gyro = self.read_imu_raw()

        if (
                self.hw_config.accel_forward_axis is None
                or self.hw_config.accel_vertical_axis is None
                or self.hw_config.gyro_pitch_axis is None
        ):
            # Return raw data only (Toddler Mode)
            return IMUReading(
                pitch_angle=0.0,
                pitch_rate=0.0,
                yaw_rate=0.0,
                roll_angle=0.0,
                roll_rate=0.0,
                error_count=self._imu_consecutive_errors,
                accel_raw=accel,
                gyro_raw=gyro
            )

        # Get raw values based on config
        accel_forward = self.get_mapped_value(accel, "accel_forward")
        accel_vertical = self.get_mapped_value(accel, "accel_vertical")

        # Calculate Accelerometer Angle
        # calculate_pitch(y, z) assumes y is forward, z is vertical.
        acc_angle = calculate_pitch(accel_forward, accel_vertical)

        # Gyro Rates
        gyro_rate = self.get_mapped_value(gyro, "gyro_pitch")

        # Explicitly handle optional axes instead of relying on silent failures
        yaw_rate = self.get_mapped_value(gyro, "gyro_yaw") if self.hw_config.gyro_yaw_axis else 0.0
        roll_rate = self.get_mapped_value(gyro, "gyro_roll") if self.hw_config.gyro_roll_axis else 0.0

        # Roll Angle (Approximate from Accel)
        roll_axis = self.accel_roll_axis
        if roll_axis:
            accel_roll = getattr(accel, roll_axis.value)
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
            error_count=self._imu_consecutive_errors,
            accel_raw=accel,
            gyro_raw=gyro
        )

    def set_motor_retries(self, retries: int) -> None:
        """Set the I2C retry count for the motor driver."""
        if self.pz is not None:
            self.pz.set_retries(retries)

    @icontract.require(lambda left: -100 <= left <= 100, "Left motor speed must be between -100 and 100")
    @icontract.require(lambda right: -100 <= right <= 100, "Right motor speed must be between -100 and 100")
    def set_motors(self, left: float, right: float, trim_override: float | None = None) -> None:
        """
        Set motor speeds.
        """
        if self.pz is None:
            raise RuntimeError("CRITICAL: Motor Driver not initialized (Bus Unknown?)")

        # FAIL LOUD: Do not silently ignore unmapped motors
        if self.hw_config.motor_l is None or self.hw_config.motor_r is None:
            raise RuntimeError(
                f"CRITICAL: Attempted to actuate motors, but channels are unmapped. L:{self.hw_config.motor_l} R:{self.hw_config.motor_r}")

        trim = trim_override if trim_override is not None else self.learning_state.motor_trim

        if trim > 0:
            right *= (1.0 - trim)
        elif trim < 0:
            left *= (1.0 - abs(trim))

        if self.hw_config.motor_l_invert:
            left = -left
        if self.hw_config.motor_r_invert:
            right = -right

        left_val = int(clamp(left, MOTOR_MIN_OUTPUT, MOTOR_MAX_OUTPUT))
        right_val = int(clamp(right, MOTOR_MIN_OUTPUT, MOTOR_MAX_OUTPUT))

        vals = [0, 0]
        vals[self.hw_config.motor_l] = left_val
        vals[self.hw_config.motor_r] = right_val

        self.pz.set_motors(vals[0], vals[1])

    def stop(self) -> None:
        """Stop all motors."""
        if self.pz is not None:
            self.pz.stop()

    def cleanup(self) -> None:
        """Cleanup hardware resources."""
        if self.pz is not None:
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
        elif pitch > self.learning_state.crash_angle:
            return "CRASHED"
        else:
            return "FALLING"

    def _calibrate_bias_if_static(self, history: deque[glm.vec3], rate: float, threshold: float) -> bool:
        """
        Check if the robot is static but reading a biased rate, and auto-calibrate if so.
        Returns True if calibration was applied.
        """
        if len(history) != history.maxlen:
            return False

        # Calculate range (max - min) for each axis
        xs = [v.x for v in history]
        ys = [v.y for v in history]
        zs = [v.z for v in history]

        range_x = max(xs) - min(xs)
        range_y = max(ys) - min(ys)
        range_z = max(zs) - min(zs)

        # If variance is low (< 0.5 deg/s noise), we are likely still.
        # But rate > threshold means we are biased.
        if max(range_x, range_y, range_z) < 0.5:
            logger.warning(f"  [DRIFT] Rate {rate:.1f} > {threshold} but Variance is low. Auto-Calibrating...")

            # Calculate observed bias (Average of current readings)
            avg_x = sum(xs) / len(xs)
            avg_y = sum(ys) / len(ys)
            avg_z = sum(zs) / len(zs)

            if max(abs(avg_x), abs(avg_y), abs(avg_z)) > 50.0:
                logger.error(f"  [FATAL] Excessive gyro drift detected: ({avg_x:.2f}, {avg_y:.2f}, {avg_z:.2f})")
                raise RuntimeError("Excessive gyro drift")

            self.learning_state.gyro_bias_x += avg_x
            self.learning_state.gyro_bias_y += avg_y
            self.learning_state.gyro_bias_z += avg_z

            self.learning_state.save()
            logger.info(
                f"  [CALIBRATED] Bias Updated. New Offsets: ({self.learning_state.gyro_bias_x:.2f}, {self.learning_state.gyro_bias_y:.2f}, {self.learning_state.gyro_bias_z:.2f})")
            return True

        return False

    def wait_for_stability(self, duration: float = 2.0, threshold: float = 2.0) -> None:
        """
        Wait for the robot to be stable (gyro rates low) for a duration.
        Blocks until the condition is met.

        Baby Brain Adaptation:
        If the robot detects it is "stable" (low variance) but "biased" (high rate),
        it will auto-calibrate the gyro bias to zero the current reading.
        """
        logger.info(f"Waiting for stability (rates < {threshold} deg/s) for {duration}s...")

        start_stable_time = None
        last_log = 0.0

        # History for Variance Check (Last 1 second @ 20Hz)
        history: deque[glm.vec3] = deque(maxlen=20)

        while True:
            # Pulse the heartbeat to show we are still alive
            if self.watchdog:
                self.watchdog.heartbeat()

            # Read sensors
            try:
                # Use RAW data (which now includes bias correction)
                accel, gyro = self.read_imu_raw()
            except Exception as e:
                logger.warning(f"  [Error reading IMU] {e}")
                time.sleep(0.1)
                continue

            # Maintain History
            history.append(gyro)

            # Calculate total rate magnitude
            rate = abs(gyro.x) + abs(gyro.y) + abs(gyro.z)

            if rate < threshold:
                if start_stable_time is None:
                    start_stable_time = time.time()
                elif time.time() - start_stable_time >= duration:
                    logger.info("  [STABLE] Robot is still.")
                    return
            else:
                # Reset stability timer
                start_stable_time = None

                # Check for "Static but Biased" condition
                if self._calibrate_bias_if_static(history, rate, threshold):
                    # Clear history and retry immediately
                    history.clear()
                    continue

                if time.time() - last_log > 1.0:
                    logger.debug(f"  [MOVING] Rate {rate:.1f} > {threshold}. Waiting...")
                    last_log = time.time()

            time.sleep(0.05)

    def execute_maneuver(self, steps: list[tuple[float, float, float]], sample_interval: float = 0.01,
                         trim_override: float | None = None) -> MeasureResult:
        """
        Execute a sequence of motor commands and collect IMU readings throughout.

        :param steps: list of (left_power, right_power, duration) tuples.
        :param sample_interval: Time between IMU samples.
        :param trim_override: Optional override for motor trim (e.g. for calibration).
        :return: MeasureResult containing total duration and all samples.
        """
        samples = []
        total_duration = 0.0

        try:
            for left, right, duration in steps:
                self.set_motors(left, right, trim_override=trim_override)
                step_end = time.time() + duration

                # Run for the duration of this step
                while time.time() < step_end:
                    if self.watchdog:
                        self.watchdog.heartbeat()
                    samples.append(self.read_imu_converted())
                    time.sleep(sample_interval)

                total_duration += duration
        finally:
            self.stop()

        return MeasureResult(duration=total_duration, samples=samples)

    @icontract.require(lambda command: command.duration > 0, "Duration must be positive")
    @icontract.ensure(lambda result: result.status != 'unknown', "Measure result status must be known")
    def drive_and_measure(self, command: DriveCommand) -> MeasureResult:
        """
        Drive motors for a duration and collect IMU readings based on a DriveCommand.
        Returns a MeasureResult object containing samples and stats.
        """
        if command.wait_for_stability:
            self.wait_for_stability()
        return self.execute_maneuver(
            [(command.left_power, command.right_power, command.duration)],
            command.sample_interval,
            trim_override=command.trim_override
        )

    def measure_gravity(self, duration: float = 1.0) -> glm.vec3:
        """
        Measure average gravity vector by staying still (0 power).
        """
        res = self.drive_and_measure(DriveCommand(0.0, 0.0, duration))
        return average_vector([s.accel_raw for s in res.samples if s.accel_raw])
