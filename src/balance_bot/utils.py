import sys
import time
import math
import logging
from pathlib import Path
from collections import deque
from typing import NamedTuple, Union

logger = logging.getLogger(__name__)

FORCE_CALIB_FILE = Path("force_calibration.txt")
_CAPTURE_HANDLER = None


class LogCaptureHandler(logging.Handler):
    """Handler that stores the last N log records in memory."""

    def __init__(self, capacity: int = 50):
        super().__init__()
        self.buffer = deque(maxlen=capacity)
        self.setFormatter(
            logging.Formatter(
                "%(asctime)s [%(levelname)s] %(message)s", datefmt="%H:%M:%S"
            )
        )

    def emit(self, record):
        try:
            msg = self.format(record)
            self.buffer.append(msg)
        except Exception:
            self.handleError(record)


class Vector3(NamedTuple):
    """Type definition for a 3D vector (x, y, z)."""

    x: float
    y: float
    z: float

    def __getitem__(self, key: Union[str, int]) -> float:
        if isinstance(key, str):
            if key == "x":
                return self.x
            if key == "y":
                return self.y
            if key == "z":
                return self.z
            raise KeyError(key)
        return tuple.__getitem__(self, key)

    def items(self):
        return self._asdict().items()

    @staticmethod
    def from_dict(d: dict[str, float]) -> "Vector3":
        return Vector3(d["x"], d["y"], d["z"])

    def __add__(self, other: "Vector3") -> "Vector3":
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Vector3") -> "Vector3":
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> "Vector3":
        return Vector3(self.x * scalar, self.y * scalar, self.z * scalar)

    def __rmul__(self, scalar: float) -> "Vector3":
        return self.__mul__(scalar)

    def __truediv__(self, scalar: float) -> "Vector3":
        if scalar == 0:
            raise ZeroDivisionError("Vector division by zero")
        return Vector3(self.x / scalar, self.y / scalar, self.z / scalar)

    def __neg__(self) -> "Vector3":
        return Vector3(-self.x, -self.y, -self.z)

    @property
    def magnitude(self) -> float:
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def dot(self, other: "Vector3") -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self, other: "Vector3") -> "Vector3":
        return Vector3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )


class ComplementaryFilter:
    """
    Sensor Fusion Algorithm.
    Combines Gyro (Fast, Drifts) and Accelerometer (Slow, Noisy, Stable) data.

    Formula:
        Angle = alpha * (Angle + GyroRate * dt) + (1 - alpha) * AccelAngle
    """

    def __init__(self, alpha: float):
        """
        Initialize the filter.
        :param alpha: Trust factor for Gyro (0.0 to 1.0).
                      Example: 0.98 means 98% Gyro, 2% Accel.
        """
        self.alpha = alpha
        self.angle = 0.0

    def update(self, new_angle: float, rate: float, loop_delta_time: float) -> float:
        """
        Update the filter state.
        :param new_angle: The absolute angle from Accelerometer.
        :param rate: The angular rate from Gyroscope.
        :param loop_delta_time: Time delta in seconds.
        :return: The filtered angle.
        """
        self.angle = (self.alpha * (self.angle + rate * loop_delta_time)) + (
            (1.0 - self.alpha) * new_angle
        )
        return self.angle


class RateLimiter:
    """
    Loop Frequency Regulator.
    Ensures the control loop runs at a consistent predictable speed (e.g. 100Hz).
    Uses a hybrid sleep/busy-wait strategy for sub-millisecond precision.
    """

    # Threshold for switching from time.sleep to busy-wait (seconds)
    BUSY_WAIT_THRESHOLD = 0.0015  # 1.5ms

    def __init__(self, frequency: float):
        """
        Initialize the rate limiter.
        :param frequency: Target frequency in Hz.
        """
        self.period = 1.0 / frequency
        self.next_time = time.perf_counter()
        self.last_wake = self.next_time

    def sleep(self) -> float:
        """
        Sleep for the remainder of the current period.
        :return: The actual time elapsed since the last sleep ended (delta time).
        """
        self.next_time += self.period
        now = time.perf_counter()

        # If we are lagging, reset next_time to avoid catch-up bursts.
        if now > self.next_time:
            self.next_time = now

        # Hybrid sleep: use time.sleep for the bulk of the time
        sleep_time = self.next_time - time.perf_counter() - self.BUSY_WAIT_THRESHOLD
        if sleep_time > 0:
            time.sleep(sleep_time)

        # Busy-wait for high precision
        while time.perf_counter() < self.next_time:
            pass

        # Calculate actual dt
        final_now = time.perf_counter()
        dt = final_now - self.last_wake
        self.last_wake = final_now

        return dt

    def reset(self) -> None:
        """Reset the internal timer to current time (e.g., after a pause)."""
        self.next_time = time.perf_counter()
        self.last_wake = self.next_time


class LogThrottler:
    """
    Prevents log flooding by enforcing a minimum interval between logs.
    """

    def __init__(self, interval_sec: float):
        """
        :param interval_sec: Minimum seconds between logs.
        """
        self.interval = interval_sec
        self.last_log_time = 0.0

    def should_log(self) -> bool:
        """
        Check if we are allowed to log now.
        :return: True if enough time has passed.
        """
        now = time.monotonic()
        if now - self.last_log_time > self.interval:
            self.last_log_time = now
            return True
        return False


def clamp(value: float, min_val: float, max_val: float) -> float:
    """
    Restrict a value to be within a specific range.
    :param value: Input value.
    :param min_val: Floor.
    :param max_val: Ceiling.
    :return: Clamped value.
    """
    return max(min(value, max_val), min_val)


def to_signed(h: int, low: int) -> int:
    """
    Convert high and low bytes to a signed 16-bit integer.

    :param h: High byte (0-255).
    :param low: Low byte (0-255).
    :return: Signed 16-bit integer (-32768 to 32767).
    """
    val = (h << 8) | low
    if val >= 32768:
        val -= 65536
    return val


def calculate_pitch(accel_y: float, accel_z: float) -> float:
    """
    Calculate pitch angle from accelerometer vectors using atan2.

    :param accel_y: Acceleration along the forward axis.
    :param accel_z: Acceleration along the vertical axis.
    :return: Angle in degrees.
    """
    return math.degrees(math.atan2(accel_y, accel_z))


def setup_logging(level: int = logging.INFO) -> None:
    """
    Configure standard logging format.
    :param level: Logging verbosity (default INFO).
    """
    global _CAPTURE_HANDLER
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%H:%M:%S",
    )

    # Attach capture handler to root logger
    root = logging.getLogger()
    if _CAPTURE_HANDLER is None:
        _CAPTURE_HANDLER = LogCaptureHandler()
        root.addHandler(_CAPTURE_HANDLER)


def get_captured_logs() -> str:
    """Retrieve recent logs from the capture buffer."""
    if _CAPTURE_HANDLER:
        return "\n".join(_CAPTURE_HANDLER.buffer)
    return "No logs captured."


def check_force_calibration_flag() -> bool:
    """
    Check for external triggers to force a calibration run.
    Triggers:
     1. Existence of 'force_calibration.txt'.
     2. '--force-calibration' command line arg.

    :return: True if calibration is requested.
    """
    if FORCE_CALIB_FILE.exists():
        logger.info(f"Force calibration file found: {FORCE_CALIB_FILE}")
        return True
    if "--force-calibration" in sys.argv:
        logger.info("Force calibration flag found")
        return True
    return False


def cross_product(a: Vector3, b: Vector3) -> Vector3:
    """
    Calculate the cross product of two 3D vectors (a x b).

    :param a: First vector (Vector3 or object with x,y,z attributes).
    :param b: Second vector (Vector3 or object with x,y,z attributes).
    :return: Cross product vector (Vector3).
    """
    # Prefer method if available (it is now)
    if hasattr(a, "cross") and callable(a.cross):
        return a.cross(b)

    # Fallback for duck-typed objects (if any still exist)
    return Vector3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    )


def analyze_dominance(
    data: Union[dict[str, float], Vector3],
    label: str,
    expected_axis: str = None,
    threshold: float = 1.5,
) -> tuple[str, float, bool]:
    """
    Analyzes a dictionary of axis values to find the dominant signal.

    :param data: Dictionary of axis values (e.g. {'x': 100, 'y': 10}) or Vector3.
    :param label: Name of the test for logging.
    :param expected_axis: (Optional) The axis expected to be dominant.
    :param threshold: Minimum ratio between winner and runner-up.
    :return: Tuple (winner_axis, ratio, is_success)
    """
    if isinstance(data, Vector3):
        data = data._asdict()

    sorted_items = sorted(data.items(), key=lambda x: abs(x[1]), reverse=True)
    winner, winner_val = sorted_items[0]
    runner, runner_val = sorted_items[1]

    ratio = abs(winner_val) / (abs(runner_val) + 1e-9)

    print(
        f"   [Analysis] {label}: Winner={winner.upper()} ({abs(winner_val):.2f}) vs Runner={runner.upper()} ({abs(runner_val):.2f}) -> Ratio: {ratio:.1f}"
    )

    success = True

    if expected_axis and winner != expected_axis:
        print(
            f"   [FAILURE] Expected {expected_axis.upper()} to be dominant, but {winner.upper()} won!"
        )
        success = False

    if ratio < threshold:
        print(f"   [WARNING] Ambiguous Result! Ratio {ratio:.1f} < {threshold}")
        success = False
        if not expected_axis:
            print("   The detected axis is not significantly stronger than others.")

    if success:
        print(f"   [PASS] Strong signal for {label}.")

    return winner, ratio, success
