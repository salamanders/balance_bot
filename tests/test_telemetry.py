from unittest.mock import patch
from balance_bot.telemetry import TelemetryBlackbox

def test_log_tick_happy_path(tmp_path):
    """Test log_tick successfully enqueues a correctly formatted tuple."""
    # Use a temporary file to avoid cluttering the filesystem during test
    csv_file = tmp_path / "test_telemetry.csv"
    blackbox = TelemetryBlackbox(filename=str(csv_file))
    blackbox.running = True  # Simulate running state

    with patch("time.time", return_value=12345.678):
        blackbox.log_tick("DRIVING", 1.234, 5.678, 9.012, 0.5, -0.5, 0.0)

    assert not blackbox.queue.empty()
    item = blackbox.queue.get_nowait()

    # Expected: (timestamp, state, pitch, pitch_rate, yaw_rate, left_pwm, right_pwm, pid_target)
    # with floats rounded to 2 decimal places
    expected = (12345.678, "DRIVING", 1.23, 5.68, 9.01, 0.5, -0.5, 0.0)
    assert item == expected

def test_log_tick_not_running(tmp_path):
    """Test log_tick does nothing if running is False."""
    csv_file = tmp_path / "test_telemetry.csv"
    blackbox = TelemetryBlackbox(filename=str(csv_file))
    blackbox.running = False

    blackbox.log_tick("DRIVING", 1.234, 5.678, 9.012, 0.5, -0.5, 0.0)

    assert blackbox.queue.empty()

def test_log_tick_exception_handling(tmp_path):
    """Test log_tick catches and ignores exceptions during enqueueing (e.g., Queue Full)."""
    csv_file = tmp_path / "test_telemetry.csv"
    blackbox = TelemetryBlackbox(filename=str(csv_file))
    blackbox.running = True

    # Force the queue to raise an exception when put_nowait is called
    with patch.object(blackbox.queue, "put_nowait", side_effect=Exception("Queue is full")):
        # This should not raise an exception, it should be caught and ignored
        blackbox.log_tick("DRIVING", 1.234, 5.678, 9.012, 0.5, -0.5, 0.0)

    # The queue should remain empty because put_nowait failed
    assert blackbox.queue.empty()
