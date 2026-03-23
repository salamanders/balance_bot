from typing import Any
import subprocess
from balance_bot.utils import get_i2c_failure_report
import math
import logging
from unittest.mock import patch
import glm
from balance_bot.utils import clamp, RateLimiter, ComplementaryFilter, calculate_pitch, LogThrottler, LogCaptureHandler, check_force_calibration_flag

def test_clamp() -> None:
    assert clamp(10, 0, 5) == 5.0
    assert clamp(-10, 0, 5) == 0.0
    assert clamp(3, 0, 5) == 3.0
    assert clamp(0, 0, 5) == 0.0
    assert clamp(5, 0, 5) == 5.0

def test_rate_limiter() -> None:
    freq = 50
    # Period = 0.02

    with patch("time.perf_counter") as mock_perf, patch("time.sleep") as mock_sleep:
        # Initial time
        mock_perf.return_value = 100.0

        limiter = RateLimiter(freq) # calls perf_counter

        # Burn first cycle reset
        mock_perf.return_value = 100.0
        limiter.reset() # calls perf_counter

        # Now simulate cycle 1
        # Work took 0.001s
        mock_perf.side_effect = [
            100.001, # first call inside sleep() `now = time.perf_counter()`
            100.001, # `sleep_time = self.next_time - time.perf_counter() ...`
            100.020, # `while time.perf_counter() < self.next_time` loop exits
            100.020, # `final_now = time.perf_counter()`
        ]

        dt = limiter.sleep()

        assert abs(dt - 0.020) < 1e-6
        mock_sleep.assert_called_once()
        # Sleep should be called with 0.02 - 0.001 - BUSY_WAIT(0.002) = 0.017
        assert abs(mock_sleep.call_args[0][0] - 0.017) < 1e-6

def test_rate_limiter_lagging() -> None:
    freq = 50
    with patch("time.perf_counter") as mock_perf, patch("time.sleep") as mock_sleep:
        limiter = RateLimiter(freq)

        mock_perf.return_value = 100.0
        limiter.reset()

        # Simulate work taking longer than period (e.g. 0.030s)
        # expected self.next_time was 100.020, but now is 100.030
        mock_perf.side_effect = [
            100.030, # inside sleep() `now = time.perf_counter()`
            100.030, # `sleep_time = self.next_time - time.perf_counter() ...`
            100.030, # `while time.perf_counter() < self.next_time` loop exits
            100.030, # `final_now = time.perf_counter()`
        ]

        dt = limiter.sleep()
        assert abs(dt - 0.030) < 1e-6
        mock_sleep.assert_not_called()

def test_complementary_filter() -> None:
    alpha = 0.98
    cf = ComplementaryFilter(alpha)

    assert cf.angle == 0.0

    # Update: rate=0, angle=0 => result=0
    res = cf.update(0.0, 0.0, 0.1)
    assert res == 0.0

    # One step update
    # new_angle = 10, rate = 10, dt = 0.1
    # gyro_part = 0 + 10 * 0.1 = 1.0
    # accel_part = 10.0
    # result = 0.98 * 1.0 + 0.02 * 10.0 = 0.98 + 0.2 = 1.18
    res = cf.update(10.0, 10.0, 0.1)
    assert abs(res - 1.18) < 1e-9

    # Internal state should update
    assert cf.angle == res

def test_calculate_pitch() -> None:
    # Vertical (Z=1, Y=0)
    assert math.isclose(calculate_pitch(0.0, 1.0), 0.0)

    # 45 degrees forward (Y=1, Z=1)
    assert math.isclose(calculate_pitch(1.0, 1.0), 45.0)

    # 90 degrees forward (Y=1, Z=0)
    assert math.isclose(calculate_pitch(1.0, 0.0), 90.0)

    # -45 degrees backward (Y=-1, Z=1)
    assert math.isclose(calculate_pitch(-1.0, 1.0), -45.0)

from balance_bot.utils import analyze_dominance  # noqa: E402

def test_analyze_dominance() -> None:
    # Clear winner
    data = {'x': 100.0, 'y': 10.0, 'z': 5.0}
    winner, ratio, success = analyze_dominance(data, "Test1")
    assert winner == 'x'
    assert abs(ratio - 10.0) < 1e-5
    assert success is True

    # Ambiguous (ratio 100/90 = 1.11 < 1.5)
    data = {'x': 100.0, 'y': 90.0, 'z': 5.0}
    winner, ratio, success = analyze_dominance(data, "Test2")
    assert winner == 'x'
    assert success is False

    # Expected winner matches
    data = {'x': 100.0, 'y': 10.0}
    winner, ratio, success = analyze_dominance(data, "Test3", expected_axis='x')
    assert success is True

    # Expected winner mismatch
    data = {'x': 10.0, 'y': 100.0}
    winner, ratio, success = analyze_dominance(data, "Test4", expected_axis='x')
    assert winner == 'y'
    assert success is False

def test_analyze_dominance_with_glm() -> None:
    # Clear winner
    data = glm.vec3(100.0, 10.0, 5.0)
    winner, ratio, success = analyze_dominance(data, "TestGLM")
    assert winner == 'x'
    assert abs(ratio - 10.0) < 1e-5
    assert success is True

def test_log_throttler() -> None:
    with patch("time.monotonic") as mock_time:
        # Start at time 100.0
        mock_time.return_value = 100.0

        # Interval of 1.0 second
        throttler = LogThrottler(1.0)

        # First call should succeed (100.0 - 0.0 > 1.0)
        assert throttler.should_log() is True

        # Immediate subsequent call should fail
        assert throttler.should_log() is False

        # Advance time by 0.5s (100.5) - still shouldn't log
        mock_time.return_value = 100.5
        assert throttler.should_log() is False

        # Advance time to 101.1s (1.1s elapsed since last log) - should log
        mock_time.return_value = 101.1
        assert throttler.should_log() is True

        # Immediate subsequent call should fail again
        assert throttler.should_log() is False

def test_log_capture_handler_emit() -> None:
    """Test that the handler correctly formats and stores log records."""
    handler = LogCaptureHandler(capacity=10)
    record = logging.makeLogRecord({"msg": "Test message"})

    # Mock format so we don't depend on actual formatter
    with patch.object(handler, 'format', return_value="FORMATTED MSG"):
        handler.emit(record)

    assert len(handler.buffer) == 1
    assert handler.buffer[0] == "FORMATTED MSG"

def test_log_capture_handler_error() -> None:
    """Test that handleError is called when format raises an exception."""
    handler = LogCaptureHandler()
    record = logging.makeLogRecord({"msg": "Test message"})

    # Mock format to raise exception
    with patch.object(handler, 'format', side_effect=Exception("Boom")):
        # Mock handleError to verify it's called
        with patch.object(handler, 'handleError') as mock_handle_error:
            handler.emit(record)

            mock_handle_error.assert_called_once_with(record)
            assert len(handler.buffer) == 0



@patch("balance_bot.utils.Path.exists")
@patch("sys.argv", ["script_name.py"])
def test_check_force_calibration_flag_neither(mock_exists: Any) -> None:
    """Test when neither the file nor the flag are present."""
    mock_exists.return_value = False
    assert check_force_calibration_flag() is False

@patch("balance_bot.utils.Path.exists")
@patch("sys.argv", ["script_name.py"])
def test_check_force_calibration_flag_file_only(mock_exists: Any) -> None:
    """Test when only the force calibration file is present."""
    mock_exists.return_value = True
    assert check_force_calibration_flag() is True

@patch("balance_bot.utils.Path.exists")
@patch("sys.argv", ["script_name.py", "--force-calibration"])
def test_check_force_calibration_flag_flag_only(mock_exists: Any) -> None:
    """Test when only the force calibration flag is present."""
    mock_exists.return_value = False
    assert check_force_calibration_flag() is True

@patch("balance_bot.utils.Path.exists")
@patch("sys.argv", ["script_name.py", "--force-calibration"])
def test_check_force_calibration_flag_both(mock_exists: Any) -> None:
    """Test when both the file and the flag are present."""
    mock_exists.return_value = True
    assert check_force_calibration_flag() is True


@patch("balance_bot.utils.Path.exists")
def test_get_i2c_failure_report_no_bus(mock_exists: Any) -> None:
    mock_exists.return_value = False
    result = get_i2c_failure_report(1, 0x68, "MPU6050")
    assert result == "CRITICAL FAILURE: I2C Bus 1 (/dev/i2c-1) does not exist. The kernel driver is not loaded. Enable I2C in raspi-config or /boot/config.txt."

@patch("balance_bot.utils.Path.exists")
@patch("os.access")
@patch("os.environ.get")
def test_get_i2c_failure_report_no_permission(mock_env_get: Any, mock_access: Any, mock_exists: Any) -> None:
    mock_exists.return_value = True
    mock_access.return_value = False
    mock_env_get.return_value = "testuser"
    result = get_i2c_failure_report(1, 0x68, "MPU6050")
    assert result == "CRITICAL FAILURE: Permission denied accessing /dev/i2c-1. User 'testuser' cannot read/write. Run: sudo usermod -aG i2c $USER"

@patch("balance_bot.utils.Path.exists")
@patch("os.access")
@patch("subprocess.run")
def test_get_i2c_failure_report_confusion(mock_run: Any, mock_access: Any, mock_exists: Any) -> None:
    mock_exists.return_value = True
    mock_access.return_value = True
    mock_process = subprocess.CompletedProcess(args=[], returncode=0, stdout=" 68 ")
    mock_run.return_value = mock_process
    result = get_i2c_failure_report(1, 0x68, "MPU6050")
    assert result == "CONFUSION: Device MPU6050 (0x68) IS detected on Bus 1 by i2cdetect, but Python driver failed. Check for library version mismatch or intermittent wiring connection."

@patch("balance_bot.utils.Path.exists")
@patch("os.access")
@patch("subprocess.run")
def test_get_i2c_failure_report_not_found(mock_run: Any, mock_access: Any, mock_exists: Any) -> None:
    mock_exists.return_value = True
    mock_access.return_value = True
    mock_process = subprocess.CompletedProcess(args=[], returncode=0, stdout=" -- ")
    mock_run.return_value = mock_process
    result = get_i2c_failure_report(1, 0x68, "MPU6050")
    assert result == "Hardware not found or failed.  You may be using a different bus, try 'Check I2C Bus'.  Also check wire connections."

@patch("balance_bot.utils.Path.exists")
@patch("os.access")
@patch("subprocess.run")
def test_get_i2c_failure_report_missing_i2cdetect(mock_run: Any, mock_access: Any, mock_exists: Any) -> None:
    mock_exists.return_value = True
    mock_access.return_value = True
    mock_run.side_effect = FileNotFoundError()
    result = get_i2c_failure_report(1, 0x68, "MPU6050")
    assert result == "DEPENDENCY FAILURE: 'i2cdetect' is missing. Install i2c-tools."

@patch("balance_bot.utils.Path.exists")
@patch("os.access")
@patch("subprocess.run")
def test_get_i2c_failure_report_unknown_error(mock_run: Any, mock_access: Any, mock_exists: Any) -> None:
    mock_exists.return_value = True
    mock_access.return_value = True
    mock_run.side_effect = Exception("Mocked Error")
    result = get_i2c_failure_report(1, 0x68, "MPU6050")
    assert result == "UNKNOWN FAILURE: Error running diagnostics: Mocked Error"
