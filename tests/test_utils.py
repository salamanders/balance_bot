import time
import math
import logging
from unittest.mock import patch
import glm
from balance_bot.utils import clamp, RateLimiter, ComplementaryFilter, calculate_pitch, to_signed, LogThrottler, LogCaptureHandler

def test_clamp():
    assert clamp(10, 0, 5) == 5.0
    assert clamp(-10, 0, 5) == 0.0
    assert clamp(3, 0, 5) == 3.0
    assert clamp(0, 0, 5) == 0.0
    assert clamp(5, 0, 5) == 5.0

def test_to_signed():
    # Zero
    assert to_signed(0, 0) == 0

    # Max positive (32767) -> 0x7F, 0xFF
    assert to_signed(0x7F, 0xFF) == 32767

    # Min negative (-32768) -> 0x80, 0x00
    assert to_signed(0x80, 0x00) == -32768

    # -1 -> 0xFF, 0xFF
    assert to_signed(0xFF, 0xFF) == -1

    # Arbitrary positive: 1 -> 0x00, 0x01
    assert to_signed(0x00, 0x01) == 1

    # Arbitrary negative: -256 -> 0xFF, 0x00
    assert to_signed(0xFF, 0x00) == -256

def test_rate_limiter():
    freq = 50
    limiter = RateLimiter(freq)

    start = time.monotonic()
    # Burn first cycle reset
    limiter.reset()

    # Simulate a fast loop
    for _ in range(5):
        time.sleep(0.001) # Work takes 1ms
        limiter.sleep()   # Should sleep rest of 20ms

    end = time.monotonic()
    elapsed = end - start

    # 5 periods of 20ms = 0.1s
    assert elapsed >= 0.1
    # Should not be too slow either (allow 20% overhead)
    assert elapsed < 0.12

def test_complementary_filter():
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

def test_calculate_pitch():
    # Vertical (Z=1, Y=0)
    assert math.isclose(calculate_pitch(0.0, 1.0), 0.0)

    # 45 degrees forward (Y=1, Z=1)
    assert math.isclose(calculate_pitch(1.0, 1.0), 45.0)

    # 90 degrees forward (Y=1, Z=0)
    assert math.isclose(calculate_pitch(1.0, 0.0), 90.0)

    # -45 degrees backward (Y=-1, Z=1)
    assert math.isclose(calculate_pitch(-1.0, 1.0), -45.0)

from balance_bot.utils import analyze_dominance  # noqa: E402

def test_analyze_dominance():
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

def test_analyze_dominance_with_glm():
    # Clear winner
    data = glm.vec3(100.0, 10.0, 5.0)
    winner, ratio, success = analyze_dominance(data, "TestGLM")
    assert winner == 'x'
    assert abs(ratio - 10.0) < 1e-5
    assert success is True

def test_log_throttler():
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

def test_log_capture_handler_emit():
    """Test that the handler correctly formats and stores log records."""
    handler = LogCaptureHandler(capacity=10)
    record = logging.makeLogRecord({"msg": "Test message"})

    # Mock format so we don't depend on actual formatter
    with patch.object(handler, 'format', return_value="FORMATTED MSG"):
        handler.emit(record)

    assert len(handler.buffer) == 1
    assert handler.buffer[0] == "FORMATTED MSG"

def test_log_capture_handler_error():
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

from balance_bot.utils import check_force_calibration_flag

@patch("balance_bot.utils.Path.exists")
@patch("sys.argv", ["script_name.py"])
def test_check_force_calibration_flag_neither(mock_exists):
    """Test when neither the file nor the flag are present."""
    mock_exists.return_value = False
    assert check_force_calibration_flag() is False

@patch("balance_bot.utils.Path.exists")
@patch("sys.argv", ["script_name.py"])
def test_check_force_calibration_flag_file_only(mock_exists):
    """Test when only the force calibration file is present."""
    mock_exists.return_value = True
    assert check_force_calibration_flag() is True

@patch("balance_bot.utils.Path.exists")
@patch("sys.argv", ["script_name.py", "--force-calibration"])
def test_check_force_calibration_flag_flag_only(mock_exists):
    """Test when only the force calibration flag is present."""
    mock_exists.return_value = False
    assert check_force_calibration_flag() is True

@patch("balance_bot.utils.Path.exists")
@patch("sys.argv", ["script_name.py", "--force-calibration"])
def test_check_force_calibration_flag_both(mock_exists):
    """Test when both the file and the flag are present."""
    mock_exists.return_value = True
    assert check_force_calibration_flag() is True
