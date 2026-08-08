import csv
from typing import Any
from unittest.mock import patch

from balance_bot.telemetry import TelemetryBlackbox


def test_log_tick_happy_path(tmp_path: Any) -> None:
    """Test log_tick successfully enqueues a correctly formatted tuple."""
    csv_file = tmp_path / "test_telemetry.csv"
    blackbox = TelemetryBlackbox(filename=str(csv_file))
    blackbox.running = True

    with patch("time.time", return_value=12345.678):
        blackbox.log_tick("DRIVING", 1.234, 5.678, 9.012, 0.5, -0.5, 0.0)

    assert not blackbox.queue.empty()
    item = blackbox.queue.get_nowait()
    expected = (12345.678, "DRIVING", 1.23, 5.68, 9.01, 0.5, -0.5, 0.0)
    assert item == expected


def test_log_tick_not_running(tmp_path: Any) -> None:
    """Test log_tick does nothing if running is False."""
    csv_file = tmp_path / "test_telemetry.csv"
    blackbox = TelemetryBlackbox(filename=str(csv_file))
    blackbox.running = False

    blackbox.log_tick("DRIVING", 1.234, 5.678, 9.012, 0.5, -0.5, 0.0)
    assert blackbox.queue.empty()


def test_log_tick_exception_handling(tmp_path: Any) -> None:
    """Test log_tick catches and ignores exceptions during enqueueing."""
    csv_file = tmp_path / "test_telemetry.csv"
    blackbox = TelemetryBlackbox(filename=str(csv_file))
    blackbox.running = True

    with patch.object(blackbox.queue, "put_nowait", side_effect=Exception("Queue is full")):
        blackbox.log_tick("DRIVING", 1.234, 5.678, 9.012, 0.5, -0.5, 0.0)

    assert blackbox.queue.empty()


def test_writer_thread_logic(tmp_path: Any) -> None:
    """Test that _writer_thread correctly drains the queue and writes to CSV."""
    csv_file = tmp_path / "test_writer.csv"
    blackbox = TelemetryBlackbox(filename=str(csv_file))

    # Pre-populate the queue with more than one batch of 50
    for i in range(60):
        blackbox.queue.put((float(i), "STATE", 0.1, 0.2, 0.3, 0.4, 0.5, 0.6))

    # Ensure it's not "running" so the loop terminates after draining the queue
    blackbox.running = False

    # Manually invoke the writer thread logic
    blackbox._writer_thread()

    assert blackbox.queue.empty()

    # Verify file content
    with open(csv_file) as f:
        reader = list(csv.reader(f))

    # Header + 60 rows
    assert len(reader) == 61
    assert reader[0] == [
        "timestamp",
        "state",
        "pitch_angle",
        "pitch_rate",
        "yaw_rate",
        "left_pwm",
        "right_pwm",
        "pid_target",
    ]
    assert reader[1] == ["0.0", "STATE", "0.1", "0.2", "0.3", "0.4", "0.5", "0.6"]
    assert reader[60] == ["59.0", "STATE", "0.1", "0.2", "0.3", "0.4", "0.5", "0.6"]
