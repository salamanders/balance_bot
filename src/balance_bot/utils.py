import sys
import time
import math
import logging
from pathlib import Path
from typing import TypedDict

logger = logging.getLogger(__name__)

FORCE_CALIB_FILE = Path("force_calibration.txt")
"""File existence check for forcing calibration on startup."""


class Vector3(TypedDict):
    """
    Type definition for a 3D vector.
    Used for IMU data (Accel/Gyro).
    """

    x: float
    y: float
    z: float


class ComplementaryFilter:
    """
    A simple Complementary Filter for sensor fusion.

    Purpose:
    Combines a noisy but fast signal (Gyroscope Integration) with a
    stable but slow/noisy signal (Accelerometer Angle).

    Theory:
    Angle = (Alpha * (Angle + Gyro * dt)) + ((1 - Alpha) * Accel)

    - High Pass Filter on Gyro: Integrates rate but drifts over time.
    - Low Pass Filter on Accel: Absolute angle but sensitive to vibration.
    """

    def __init__(self, alpha: float):
        """
        Initialize the filter.

        :param alpha: Filter coefficient (0.0 to 1.0).
                      Higher values (e.g., 0.98) trust the Gyroscope more.
                      Lower values trust the Accelerometer more.
        """
        self.alpha = alpha
        self.angle = 0.0
        """Current estimated angle."""

    def update(self, new_angle: float, rate: float, loop_delta_time: float) -> float:
        """
        Update the filter state with new sensor data.

        :param new_angle: The new absolute angle measurement (Accelerometer).
                          UOM: Degrees.
        :param rate: The rate of change (Gyroscope).
                     UOM: Degrees/Second.
        :param loop_delta_time: Time elapsed since last update.
                                UOM: Seconds.
        :return: The filtered angle.
                 UOM: Degrees.
        """
        # Integrate Gyro: Old Angle + (Rate * Time)
        gyro_part = self.angle + (rate * loop_delta_time)

        # Fuse with Accelerometer
        self.angle = (self.alpha * gyro_part) + ((1.0 - self.alpha) * new_angle)
        return self.angle


class RateLimiter:
    """
    Helper to maintain a consistent loop frequency.
    Ensures the loop does not run faster than the target speed.
    """

    def __init__(self, frequency: float):
        """
        Initialize the rate limiter.

        :param frequency: Target frequency in Hz.
        """
        self.period = 1.0 / frequency
        self.next_time = time.monotonic()

    def sleep(self) -> None:
        """
        Sleep for the remainder of the period.

        Calculates how much time is left in the current time slice and sleeps.
        Updates the next target time.
        """
        self.next_time += self.period
        sleep_time = self.next_time - time.monotonic()
        if sleep_time > 0:
            time.sleep(sleep_time)

    def reset(self) -> None:
        """
        Reset the internal timer to current time.
        Useful when starting a new phase to avoid a huge initial sleep.
        """
        self.next_time = time.monotonic()


class LogThrottler:
    """
    Helper to throttle log messages.
    Prevents console spamming for events that happen frequently (e.g., Low Battery).
    """

    def __init__(self, interval_sec: float):
        """
        Initialize the log throttler.

        :param interval_sec: Minimum interval between logs in seconds.
        """
        self.interval = interval_sec
        self.last_log_time = 0.0

    def should_log(self) -> bool:
        """
        Check if it's time to log.

        :return: True if enough time has passed since the last log.
                 Also updates the internal timer if returning True.
        """
        now = time.monotonic()
        if now - self.last_log_time > self.interval:
            self.last_log_time = now
            return True
        return False


def clamp(value: float, min_val: float, max_val: float) -> float:
    """
    Clamp a value between a minimum and maximum.

    :param value: Input value.
    :param min_val: Minimum allowed value.
    :param max_val: Maximum allowed value.
    :return: Clamped value.
    """
    return max(min(value, max_val), min_val)


def calculate_pitch(accel_y: float, accel_z: float) -> float:
    """
    Calculate pitch angle in degrees from accelerometer data.

    Assumes the sensor is mounted such that:
    - Y-axis points Forward/Backward.
    - Z-axis points Up/Down.

    :param accel_y: Y-axis acceleration (Forward).
    :param accel_z: Z-axis acceleration (Vertical).
    :return: Pitch angle in degrees.
             Positive = Leaning Forward (usually).
    """
    return math.degrees(math.atan2(accel_y, accel_z))


def setup_logging(level: int = logging.INFO) -> None:
    """
    Configure logging for the application.
    Sets up stdout logging with timestamp and level.

    :param level: Logging level (default INFO).
    """
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%H:%M:%S",
    )


def check_force_calibration_flag() -> bool:
    """
    Check if force calibration is requested.

    Triggers if:
    1. 'force_calibration.txt' exists in the working directory.
    2. '--force-calibration' argument is passed to the script.

    :return: True if force calibration is requested.
    """
    if FORCE_CALIB_FILE.exists():
        logger.info(f"Force calibration file found: {FORCE_CALIB_FILE}")
        return True
    if "--force-calibration" in sys.argv:
        logger.info("Force calibration flag found")
        return True
    return False
