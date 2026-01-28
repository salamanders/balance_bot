import sys
import time
import math
import logging
from pathlib import Path
from typing import TypedDict

logger = logging.getLogger(__name__)

FORCE_CALIB_FILE = Path("force_calibration.txt")


class Vector3(TypedDict):
    """Type definition for a 3D vector."""

    x: float
    y: float
    z: float


class ComplementaryFilter:
    """
    A simple Complementary Filter for sensor fusion.
    Combines a noisy but fast signal (Gyro) with a stable but slow signal (Accel).
    """

    def __init__(self, alpha: float):
        """
        Initialize the filter.
        :param alpha: Filter coefficient (0.0 to 1.0). Higher means trust gyro more.
        """
        self.alpha = alpha
        self.angle = 0.0

    def update(self, new_angle: float, rate: float, loop_delta_time: float) -> float:
        """
        Update the filter state.
        :param new_angle: The new absolute angle measurement (e.g., from Accelerometer).
        :param rate: The rate of change (e.g., from Gyroscope).
        :param loop_delta_time: Time delta in seconds.
        :return: The filtered angle.
        """
        self.angle = (self.alpha * (self.angle + rate * loop_delta_time)) + (
            (1.0 - self.alpha) * new_angle
        )
        return self.angle


class RateLimiter:
    """Helper to maintain a consistent loop frequency."""

    def __init__(self, frequency: float):
        """
        Initialize the rate limiter.
        :param frequency: Target frequency in Hz.
        """
        self.period = 1.0 / frequency
        self.next_time = time.monotonic()

    def sleep(self) -> None:
        """Sleep for the remainder of the period."""
        self.next_time += self.period
        sleep_time = self.next_time - time.monotonic()
        if sleep_time > 0:
            time.sleep(sleep_time)

    def reset(self) -> None:
        """Reset the internal timer to current time."""
        self.next_time = time.monotonic()


class LogThrottler:
    """Helper to throttle log messages."""

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
    Assumes Y is forward/backward axis and Z is vertical.
    Returns value in degrees.
    :param accel_y: Y-axis acceleration.
    :param accel_z: Z-axis acceleration.
    :return: Pitch angle in degrees.
    """
    return math.degrees(math.atan2(accel_y, accel_z))


def setup_logging(level: int = logging.INFO) -> None:
    """
    Configure logging for the application.
    :param level: Logging level.
    """
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%H:%M:%S",
    )


def check_force_calibration_flag() -> bool:
    """
    Check if force calibration is requested via file or command line argument.
    :return: True if force calibration is requested.
    """
    if FORCE_CALIB_FILE.exists():
        logger.info(f"Force calibration file found: {FORCE_CALIB_FILE}")
        return True
    if "--force-calibration" in sys.argv:
        logger.info("Force calibration flag found")
        return True
    return False
