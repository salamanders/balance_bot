import time
import logging
from typing import TypeVar

T = TypeVar("T", int, float)


class RateLimiter:
    """Helper to maintain a consistent loop frequency."""

    def __init__(self, frequency: float):
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
        self.interval = interval_sec
        self.last_log_time = 0.0

    def should_log(self) -> bool:
        """Returns True if enough time has passed since the last log."""
        now = time.monotonic()
        if now - self.last_log_time > self.interval:
            self.last_log_time = now
            return True
        return False


def clamp(value: T, min_val: T, max_val: T) -> T:
    """Clamp a value between a minimum and maximum."""
    return max(min(value, max_val), min_val)


def setup_logging(level: int = logging.INFO) -> None:
    """Configure logging for the application."""
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%H:%M:%S",
    )
