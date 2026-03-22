import sys
import time
import math
import logging
import subprocess
import os
from pathlib import Path
from collections import deque
from typing import Union, Callable, Any, Optional, Sequence

import glm

try:
    import smbus2 as smbus # type: ignore
except ImportError:
    smbus = None # type: ignore

logger = logging.getLogger(__name__)

FORCE_CALIB_FILE = Path("force_calibration.txt")
_CAPTURE_HANDLER = None


class LogCaptureHandler(logging.Handler):
    """Handler that stores the last N log records in memory."""

    def __init__(self, capacity: int = 50):
        super().__init__()
        self.buffer: deque[str] = deque(maxlen=capacity)
        self.setFormatter(
            logging.Formatter(
                "%(asctime)s [%(levelname)s] %(message)s", datefmt="%H:%M:%S"
            )
        )

    def emit(self, record):
        try:
            msg = self.format(record)
            self.buffer.append(msg)
        except (OSError, IOError, Exception):
            self.handleError(record)


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


class StdOutToLog(object):
    """File-like object that redirects writes to a logger."""
    def __init__(self, target_logger, level):
        self._logger = target_logger
        self.level = level
        self.buffer = ""

    def write(self, message):
        if message != '\n':
            self._logger.log(self.level, message.rstrip())

    def flush(self):
        pass


def setup_logging(level: int = logging.INFO, capture_stdout: bool = False) -> None:
    """
    Configure standard logging format.
    :param level: Logging verbosity (default INFO).
    :param capture_stdout: If True, redirect stdout and stderr to logging.
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

    if capture_stdout:
        sys.stdout = StdOutToLog(logging.getLogger("STDOUT"), logging.INFO)
        sys.stderr = StdOutToLog(logging.getLogger("STDERR"), logging.ERROR)


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


def analyze_dominance(
    data: Union[dict[str, float], glm.vec3],
    label: str,
    expected_axis: Optional[str] = None,
    threshold: float = 1.5,
) -> tuple[str, float, bool]:
    """
    Analyzes a dictionary of axis values to find the dominant signal.

    :param data: Dictionary of axis values (e.g. {'x': 100, 'y': 10}) or glm.vec3.
    :param label: Name of the test for logging.
    :param expected_axis: (Optional) The axis expected to be dominant.
    :param threshold: Minimum ratio between winner and runner-up.
    :return: Tuple (winner_axis, ratio, is_success)
    """
    if isinstance(data, glm.vec3):
        data = {'x': data.x, 'y': data.y, 'z': data.z}

    sorted_items = sorted(data.items(), key=lambda x: abs(x[1]), reverse=True)
    winner, winner_val = sorted_items[0]
    runner, runner_val = sorted_items[1]

    ratio = abs(winner_val) / (abs(runner_val) + 1e-9)

    logger.info(
        f"   [Analysis] {label}: Winner={winner.upper()} ({abs(winner_val):.2f}) vs Runner={runner.upper()} ({abs(runner_val):.2f}) -> Ratio: {ratio:.1f}"
    )

    success = True

    if expected_axis and winner != expected_axis:
        logger.error(
            f"   [FAILURE] Expected {expected_axis.upper()} to be dominant, but {winner.upper()} won!"
        )
        success = False

    if ratio < threshold:
        logger.warning(f"   [WARNING] Ambiguous Result! Ratio {ratio:.1f} < {threshold}")
        success = False
        if not expected_axis:
            logger.info("   The detected axis is not significantly stronger than others.")

    if success:
        logger.info(f"   [PASS] Strong signal for {label}.")

    return winner, ratio, success


# --- Diagnostic Functions ---

def get_i2c_failure_report(bus_id: int, address: int, device_name: str) -> str:
    """
    Generate a detailed diagnostic report for a failed I2C device.
    Checks:
     1. Bus existence (/dev/i2c-*).
     2. File permissions (User access).
     3. Connectivity (i2cdetect).

    :param bus_id: I2C Bus ID.
    :param address: I2C Device Address (7-bit).
    :param device_name: Human readable name.
    :return: Diagnostic string.
    """
    # Check if bus exists
    path = Path(f"/dev/i2c-{bus_id}")
    if not path.exists():
        return f"CRITICAL FAILURE: I2C Bus {bus_id} ({path}) does not exist. The kernel driver is not loaded. Enable I2C in raspi-config or /boot/config.txt."

    # Check permissions
    if not os.access(path, os.R_OK | os.W_OK):
        return f"CRITICAL FAILURE: Permission denied accessing {path}. User '{os.environ.get('USER')}' cannot read/write. Run: sudo usermod -aG i2c $USER"

    # Check connectivity with i2cdetect
    try:
        result = subprocess.run(["i2cdetect", "-y", str(bus_id)], capture_output=True, text=True)
        hex_addr = f"{address:02x}"
        if hex_addr in result.stdout:
            return f"CONFUSION: Device {device_name} (0x{hex_addr}) IS detected on Bus {bus_id} by i2cdetect, but Python driver failed. Check for library version mismatch or intermittent wiring connection."
        else:
             return "Hardware not found or failed.  You may be using a different bus, try 'Check I2C Bus'.  Also check wire connections."
    except FileNotFoundError:
        return "DEPENDENCY FAILURE: 'i2cdetect' is missing. Install i2c-tools."
    except Exception as e:
        return f"UNKNOWN FAILURE: Error running diagnostics: {e}"


# --- Generic Helper Functions (Extracted from legacy WiringCheck) ---

def make_i2c_check_fn(address: int, register: int = 0, expected_value: Optional[int] = None) -> Callable[[Any], bool]:
    """
    Creates a check function for scan_i2c.
    :param address: I2C Device Address (7-bit).
    :param register: Register to read (default 0).
    :param expected_value: Value to expect. If None, any successful read is True.
    :return: Callable that returns True if device is found.
    """
    def check(bus):
        try:
            val = bus.read_byte_data(address, register)
            if expected_value is not None:
                return val == expected_value
            return True
        except (OSError, IOError, Exception):
            return False
    return check


def scan_i2c_candidates(name: str, check_fn: Callable[[Any], bool]) -> int | None:
    """
    Scans I2C buses for a device using a callback.
    Returns the bus ID if found, else None.
    """
    if smbus is None:
        return None

    candidates = [1, 3, 0, 2]
    for bus_id in candidates:
        try:
            # smbus2 handles bus opening gracefully
            bus = smbus.SMBus(bus_id)
            try:
                if check_fn(bus):
                    logger.info(f"  [FOUND] {name} on Bus {bus_id}")
                    return bus_id
            except OSError:
                pass
            finally:
                try:
                    bus.close()
                except (OSError, IOError, Exception):
                    pass
        except (OSError, IOError, Exception):
            pass
    return None

def scan_i2c(name: str, check_fn: Callable[[Any], bool]) -> Optional[int]:
    """
    Scans and returns bus ID if found.
    If not found, prints diagnostics and returns None.
    Replaces scan_i2c_or_die.
    """
    bus = scan_i2c_candidates(name, check_fn)
    if bus is None:
        logger.error(f"  [FAILURE] Could not find {name} on any bus.")

        # Use improved diagnostics for the likely bus (1 or 3)
        # Default to checking both or giving a hint
        logger.info("  [DIAGNOSTIC] Analyzing potential causes...")
        # We don't know the address here easily unless passed, but we can guess based on name
        addr = 0x22 if "PiconZero" in name else 0x68

        # Check likely buses
        for b in [1, 3]:
            logger.info(f"  --- Bus {b} Report ---")
            logger.info(get_i2c_failure_report(b, addr, name))

        return None
    return bus

def verify_with_retries(name: str, test_fn: Callable[[int], Any],
                         check_fn: Callable[[Any], bool | str],
                         max_attempts: int = 3) -> bool:
    """
    Generic verification loop with retries.
    check_fn should return True (Pass), False (Fail/Retry),
    or a string "FAIL_FATAL" / "FAIL_RETRY".
    Returns True if verified, False if failed.
    """
    logger.info(f">>> Verifying {name} <<<")
    for i in range(max_attempts):
        logger.info(f"  [Attempt {i+1}] Checking {name}...")
        result = test_fn(i)

        outcome = check_fn(result)
        if outcome is True or outcome == "PASS":
            logger.info(f"  [SUCCESS] {name} Verified.")
            return True

        if outcome == "FAIL_FATAL":
            break

        # If outcome is False or "FAIL_RETRY", loop continues
        logger.info(f"  [RETRY] {name} check failed/ambiguous.")

    logger.error(f"  [FAILURE] Could not verify {name} after {max_attempts} attempts.")
    return False

def find_threshold(name: str, start: float, step: float, limit: float,
                    action_fn: Callable[[float], Any],
                    check_fn: Callable[[Any], bool],
                    fail_action: Optional[Callable[[Any], bool]] = None,
                    heartbeat_fn: Optional[Callable[[], None]] = None) -> Optional[float]:
    """
    Find a threshold value by incrementing.
    fail_action: Optional callback on failure. Return True to retry SAME level.
    heartbeat_fn: Optional callback to keep watchdog alive.
    Returns the found value, or None if failed.
    """
    logger.info(f">>> Finding Threshold: {name} <<<")
    val = start
    while val <= limit:
        if heartbeat_fn:
            heartbeat_fn()

        logger.info(f"  Testing {val}...")
        result = action_fn(val)

        if check_fn(result):
            logger.info(f"  [FOUND] {name} at {val}.")
            return val

        retry_same = False
        if fail_action:
            retry_same = fail_action(result)

        if not retry_same:
            val += step

    logger.error(f"  [FAILURE] Could not find {name} within limit {limit}.")
    return None


def average_vector(vectors: Sequence[glm.vec3]) -> glm.vec3:
    """
    Calculate the average vector from a sequence of vectors.
    """
    if not vectors:
        return glm.vec3(0.0)

    # We use a loop rather than `sum(vectors, start=glm.vec3(0.0))`
    # to maintain high-frequency performance and avoid creating
    # unnecessary temporary intermediate objects, which is
    # crucial for tight loops per memory constraints.
    sum_x = sum_y = sum_z = 0.0
    for v in vectors:
        sum_x += v.x
        sum_y += v.y
        sum_z += v.z

    count = len(vectors)
    return glm.vec3(sum_x / count, sum_y / count, sum_z / count)

def circular_difference(target: float, current: float) -> float:
    """
    Calculate the shortest-path angle difference between a target angle and a current angle.
    Handles the 180/-180 degree boundary safely.
    :param target: Target angle in degrees.
    :param current: Current angle in degrees.
    :return: The difference in degrees (-180 to 180).
    """
    diff = (target - current + 180) % 360 - 180
    return diff
