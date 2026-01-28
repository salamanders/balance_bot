import sys
import time
import math
import logging
from pathlib import Path
from typing import TypedDict

logger = logging.getLogger(__name__)

FORCE_CALIB_FILE = Path("force_calibration.txt")


class Vector3(TypedDict):
    """Type definition for a 3D vector (x, y, z)."""

    x: float
    y: float
    z: float


class ComplementaryFilter:
    """
    A Sensor Fusion algorithm that combines Gyroscope and Accelerometer data.

    Why?
    - Accelerometer is good for absolute angle (gravity vector) but is noisy.
    - Gyroscope is good for rate of change (smooth) but drifts over time.

    The Complementary Filter combines them:
    Angle = (alpha * (Angle + GyroRate * dt)) + ((1 - alpha) * AccelAngle)
    """

    def __init__(self, alpha: float):
        """
        :param alpha: Filter coefficient (0.0 to 1.0).
                      Values closer to 1.0 trust the Gyro more (smoother).
                      Values closer to 0.0 trust the Accel more (less drift).
                      Typical value: 0.98.
        """
        self.alpha = alpha
        self.angle = 0.0

    def update(self, new_angle: float, rate: float, loop_delta_time: float) -> float:
        """
        Update the filter state.
        :param new_angle: The new absolute angle measurement (e.g., from Accelerometer). Unit: Degrees.
        :param rate: The rate of change (e.g., from Gyroscope). Unit: Degrees/Second.
        :param loop_delta_time: Time delta since last update. Unit: Seconds.
        :return: The filtered pitch angle. Unit: Degrees.
        """
        # Integrate Gyro Rate
        gyro_estimate = self.angle + (rate * loop_delta_time)

        # Fuse with Accelerometer Angle
        self.angle = (self.alpha * gyro_estimate) + ((1.0 - self.alpha) * new_angle)
        return self.angle


class RateLimiter:
    """Helper to maintain a consistent control loop frequency."""

    def __init__(self, frequency: float):
        """
        :param frequency: Target frequency in Hz (e.g., 100 for 100 loops/second).
        """
        self.period = 1.0 / frequency
        self.next_time = time.monotonic()

    def sleep(self) -> None:
        """
        Sleep for the remainder of the period to keep timing consistent.
        Should be called at the end of the loop.
        """
        self.next_time += self.period
        sleep_time = self.next_time - time.monotonic()
        if sleep_time > 0:
            time.sleep(sleep_time)

    def reset(self) -> None:
        """Reset the internal timer to current time. Call before starting a loop."""
        self.next_time = time.monotonic()


class LogThrottler:
    """Helper to prevent log flooding by throttling messages."""

    def __init__(self, interval_sec: float):
        """
        :param interval_sec: Minimum time (seconds) between allowed logs.
        """
        self.interval = interval_sec
        self.last_log_time = 0.0

    def should_log(self) -> bool:
        """Returns True if enough time has passed since the last log."""
        now = time.monotonic()
        if now - self.last_log_time > self.interval:
            self.last_log_time = now
            return True
        return False


def clamp(value: float, min_val: float, max_val: float) -> float:
    """Clamp a value between a minimum and maximum limit."""
    return max(min(value, max_val), min_val)


def calculate_pitch(accel_y: float, accel_z: float) -> float:
    """
    Calculate pitch angle in degrees from accelerometer data using atan2.

    Concept:
    Gravity acts as a reference vector pointing down (Z).
    When the robot tilts, the gravity vector projects onto the forward axis (Y).
    atan2(y, z) calculates the angle of this vector.

    :param accel_y: Acceleration along the forward/backward axis.
    :param accel_z: Acceleration along the vertical axis.
    :return: Pitch angle in degrees.
    """
    return math.degrees(math.atan2(accel_y, accel_z))


def setup_logging(level: int = logging.INFO) -> None:
    """Configure logging for the application with standard format."""
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%H:%M:%S",
    )


def check_force_calibration_flag() -> bool:
    """
    Check if force calibration is requested via file or command line argument.
    Useful for overriding saved configuration without deleting files manually.
    """
    if FORCE_CALIB_FILE.exists():
        logger.info(f"Force calibration file found: {FORCE_CALIB_FILE}")
        return True
    if "--force-calibration" in sys.argv:
        logger.info("Force calibration flag found")
        return True
    return False
