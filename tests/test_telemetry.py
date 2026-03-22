from unittest.mock import patch

from balance_bot.telemetry import TelemetryBlackbox

def test_log_tick_not_running(tmp_path):
    # Setup
    csv_file = tmp_path / "flight_data.csv"
    telemetry = TelemetryBlackbox(filename=str(csv_file))

    # Pre-condition check
    assert telemetry.running is False

    # Action
    telemetry.log_tick("TEST", 1.111, 2.222, 3.333, 4.0, 5.0, 6.666)

    # Verification
    assert telemetry.queue.empty() is True

@patch("balance_bot.telemetry.time.time")
def test_log_tick_running(mock_time, tmp_path):
    # Setup
    mock_time.return_value = 12345.678
    csv_file = tmp_path / "flight_data.csv"
    telemetry = TelemetryBlackbox(filename=str(csv_file))
    telemetry.running = True

    # Action
    telemetry.log_tick("ACTIVE", 1.234, 2.345, 3.456, 10.0, 20.0, 4.567)

    # Verification
    assert telemetry.queue.qsize() == 1
    item = telemetry.queue.get_nowait()

    # Unpack and verify
    assert item == (
        12345.678,  # time
        "ACTIVE",   # state_name
        1.23,       # pitch (rounded)
        2.35,       # pitch_rate (rounded)
        3.46,       # yaw_rate (rounded)
        10.0,       # left_pwm
        20.0,       # right_pwm
        4.57        # pid_target (rounded)
    )

def test_log_tick_queue_full_drops_frame(tmp_path):
    # Setup
    csv_file = tmp_path / "flight_data.csv"
    telemetry = TelemetryBlackbox(filename=str(csv_file))
    telemetry.running = True

    # Fill the queue to its maxsize
    maxsize = telemetry.queue.maxsize
    for i in range(maxsize):
        telemetry.queue.put_nowait((
            float(i), "FILL", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        ))

    assert telemetry.queue.full() is True

    # Action - should not raise an exception, frame should be dropped
    # The try/except block in log_tick should catch queue.Full (which inherits from Exception)
    telemetry.log_tick("DROP", 1.0, 2.0, 3.0, 4.0, 5.0, 6.0)

    # Verification
    assert telemetry.queue.full() is True
    assert telemetry.queue.qsize() == maxsize
