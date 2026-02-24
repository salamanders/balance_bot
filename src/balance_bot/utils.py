import sys
import time
import math
import logging
import subprocess
import os
import importlib
from pathlib import Path
from collections import deque
from typing import NamedTuple, Union, Callable, Any

try:
    import smbus2 as smbus
except ImportError:
    smbus = None

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

def check_system_i2c_config():
    """Verify /boot/config.txt for Software I2C overlays."""
    print("\nChecking System I2C Config...")
    config_paths = [Path("/boot/firmware/config.txt"), Path("/boot/config.txt")]
    found_overlay = False

    for path in config_paths:
        if path.exists():
            try:
                content = path.read_text()
                active_overlay = False
                for line in content.splitlines():
                    if line.strip().startswith("dtoverlay=i2c-gpio"):
                        active_overlay = True
                        break

                if active_overlay:
                    print(f"SUCCESS: Software I2C overlay found in {path}")
                    found_overlay = True
                else:
                    print(f"INFO: No Software I2C overlay in {path}")
            except Exception as e:
                print(f"WARNING: Could not read {path}: {e}")

    if not found_overlay:
        print("INFO: Software I2C (Bus 3) not configured. This is OK if using standard hardware.")
        print("      If PiconZero blocks the I2C pins, you may need to enable Software I2C.")

def check_i2c_tools():
    """Scan I2C buses using i2cdetect."""
    print("\nChecking I2C Devices (i2cdetect)...")

    for bus_id in [1, 3]:
        print(f"--- Scanning Bus {bus_id} ---")
        try:
            result = subprocess.run(["i2cdetect", "-y", str(bus_id)], capture_output=True, text=True)
            print(result.stdout)

            # Bus 1 usually has PiconZero (0x22)
            if bus_id == 1:
                if "22" in result.stdout:
                    print("SUCCESS: PiconZero (0x22) detected on Bus 1.")
                else:
                    print("FAILURE: PiconZero (0x22) NOT detected on Bus 1.")

            # MPU6050 (0x68) can be on either
            if "68" in result.stdout:
                print(f"SUCCESS: MPU6050 (0x68) detected on Bus {bus_id}.")
            else:
                print(f"INFO: MPU6050 (0x68) NOT detected on Bus {bus_id}.")

        except FileNotFoundError:
            print("WARNING: 'i2cdetect' command not found. Please install i2c-tools.")
            break
        except Exception as e:
            print(f"ERROR running i2cdetect on Bus {bus_id}: {e}")

def check_i2c_library():
    """Check if Python smbus2 library can open the bus."""
    print("\nChecking smbus2 Access...")
    try:
        import smbus2 as smbus
        for bus_id in [1, 3]:
            try:
                # Try to open the bus
                bus = smbus.SMBus(bus_id)
                try:
                    bus.close()
                except AttributeError:
                    pass
                print(f"SUCCESS: smbus2.SMBus({bus_id}) opened successfully.")
            except Exception as e:
                print(f"INFO: Could not open smbus2.SMBus({bus_id}): {e}")
                if bus_id == 1:
                     print("Hint: Check permissions (e.g. 'sudo usermod -aG i2c $USER') and reboot.")
    except ImportError:
         print("FAILURE: 'smbus2' package not installed.")

def check_imports():
    """Check if required Python packages are installed."""
    print("\nChecking Python Imports...")

    # Shim smbus for mpu6050 check
    try:
        import sys
        import smbus2
        sys.modules['smbus'] = smbus2
    except ImportError:
        pass

    modules = ["smbus2", "mpu6050"]

    for mod in modules:
        try:
            importlib.import_module(mod)
            print(f"SUCCESS: Import '{mod}' working.")
        except ImportError as e:
             print(f"FAILURE: Could not import '{mod}': {e}")

    # Check internal piconzero
    try:
        from .hardware import piconzero  # noqa: F401
        print("SUCCESS: Internal 'piconzero' module import working.")
    except Exception as e:
         print(f"FAILURE: Could not import internal 'piconzero': {e}")

def run_diagnostics():
    """
    Main entry point for diagnostics.
    Runs a suite of checks to identify hardware/software issues.
    """
    print("=== Hardware Diagnostics ===")
    check_imports()
    check_system_i2c_config()
    check_i2c_library()
    check_i2c_tools()
    print("\nDiagnostics Complete.")
    print("========================")

# --- Generic Helper Functions (Extracted from WiringCheck) ---

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
                    print(f"  [FOUND] {name} on Bus {bus_id}")
                    return bus_id
            except OSError:
                pass
            finally:
                try:
                    bus.close()
                except Exception:
                    pass
        except Exception:
            pass
    return None

def scan_i2c_or_die(name: str, check_fn: Callable[[Any], bool]) -> int:
    """
    Scans and exits if not found (with diagnostics).
    """
    bus = scan_i2c_candidates(name, check_fn)
    if bus is None:
        print(f"  [FAILURE] Could not find {name} on any bus.")

        # Use improved diagnostics for the likely bus (1 or 3)
        # Default to checking both or giving a hint
        print("  [DIAGNOSTIC] Analyzing potential causes...")
        # We don't know the address here easily unless passed, but we can guess based on name
        addr = 0x22 if "PiconZero" in name else 0x68

        # Check likely buses
        for b in [1, 3]:
            print(f"  --- Bus {b} Report ---")
            print(get_i2c_failure_report(b, addr, name))

        sys.exit(1)
    return bus

def verify_with_retries(name: str, test_fn: Callable[[int], Any],
                         check_fn: Callable[[Any], bool | str],
                         max_attempts: int = 3, fail_fatal: bool = True) -> bool:
    """
    Generic verification loop with retries.
    check_fn should return True (Pass), False (Fail/Retry),
    or a string "FAIL_FATAL" / "FAIL_RETRY".
    """
    print(f">>> Verifying {name} <<<")
    for i in range(max_attempts):
        print(f"  [Attempt {i+1}] Checking {name}...")
        result = test_fn(i)

        outcome = check_fn(result)
        if outcome is True or outcome == "PASS":
            print(f"  [SUCCESS] {name} Verified.")
            return True

        if outcome == "FAIL_FATAL":
            break

        # If outcome is False or "FAIL_RETRY", loop continues
        print(f"  [RETRY] {name} check failed/ambiguous.")

    print(f"  [FAILURE] Could not verify {name} after {max_attempts} attempts.")
    if fail_fatal:
        sys.exit(1)
    return False

def find_threshold(name: str, start: float, step: float, limit: float,
                    action_fn: Callable[[float], Any],
                    check_fn: Callable[[Any], bool],
                    fail_action: Callable[[Any], bool] = None,
                    heartbeat_fn: Callable[[], None] = None) -> float:
    """
    Find a threshold value by incrementing.
    fail_action: Optional callback on failure. Return True to retry SAME level.
    heartbeat_fn: Optional callback to keep watchdog alive.
    """
    print(f">>> Finding Threshold: {name} <<<")
    val = start
    while val <= limit:
        if heartbeat_fn:
            heartbeat_fn()

        print(f"  Testing {val}...")
        result = action_fn(val)

        if check_fn(result):
            print(f"  [FOUND] {name} at {val}.")
            return val

        retry_same = False
        if fail_action:
            retry_same = fail_action(result)

        if not retry_same:
            val += step

    print(f"  [FAILURE] Could not find {name} within limit {limit}.")
    sys.exit(1)


def vector_angle(v1: Vector3, v2: Vector3) -> float:
    """Calculate the angle in degrees between two vectors."""
    dot = v1.dot(v2)
    mag = v1.magnitude * v2.magnitude
    if mag == 0:
        return 0.0
    val = max(-1.0, min(1.0, dot / mag))
    return math.degrees(math.acos(val))


def sort_resting_vectors(vectors: list[Vector3]) -> tuple[Vector3, Vector3]:
    """
    Sort resting vectors into two distinct groups based on angle separation.
    Returns the average vector for each group.
    """
    if not vectors:
        raise ValueError("No vectors collected.")

    pivot = vectors[0]
    bucket_a = []
    bucket_b = []

    for v in vectors:
        angle = vector_angle(pivot, v)
        if angle < 20.0:
            bucket_a.append(v)
        else:
            bucket_b.append(v)

    if not bucket_b:
        raise ValueError("Failed to find two distinct resting positions (all vectors clustered near pivot).")

    # Average buckets
    def avg_bucket(bucket: list[Vector3]) -> Vector3:
        if not bucket:
            return Vector3(0.0, 0.0, 0.0)
        sum_x = sum(v.x for v in bucket)
        sum_y = sum(v.y for v in bucket)
        sum_z = sum(v.z for v in bucket)
        count = len(bucket)
        return Vector3(sum_x / count, sum_y / count, sum_z / count)

    avg_a = avg_bucket(bucket_a)
    avg_b = avg_bucket(bucket_b)

    return avg_a, avg_b
