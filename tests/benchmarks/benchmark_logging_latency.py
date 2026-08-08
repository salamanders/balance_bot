import contextlib
import logging
import os
import sys
import time
from pathlib import Path

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../src"))

from balance_bot.utils import LogThrottler

# Setup logging
LOG_FILE = Path("benchmark_log.txt")
logging.basicConfig(
    filename=LOG_FILE,
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    force=True,
)
logger = logging.getLogger(__name__)


class MockConfig:
    def __init__(self) -> None:
        self.pid = type("PIDParams", (), {"kp": 25.0, "ki": 0.0, "kd": 0.5})()


def run_benchmark() -> None:
    config = MockConfig()
    iterations = 1000

    # 1. Baseline: Blocking Logging
    start_time = time.perf_counter()
    for _ in range(iterations):
        # Simulate PID update causing log
        logger.info(f"-> Tuned: P={config.pid.kp:.2f} I={config.pid.ki:.3f} D={config.pid.kd:.2f}")
    end_time = time.perf_counter()
    baseline_time = (end_time - start_time) * 1000  # ms
    print(
        f"Baseline (Blocking): {baseline_time:.2f} ms for {iterations} calls ({baseline_time / iterations:.4f} ms/call)"
    )

    # 2. Optimized: Throttled Logging (1.0s interval)
    throttler = LogThrottler(1.0)

    start_time = time.perf_counter()
    log_count = 0
    for _ in range(iterations):
        if throttler.should_log():
            logger.info(
                f"-> Tuned: P={config.pid.kp:.2f} I={config.pid.ki:.3f} D={config.pid.kd:.2f}"
            )
            log_count += 1

    end_time = time.perf_counter()
    optimized_time = (end_time - start_time) * 1000  # ms
    print(
        f"Optimized (Throttled): {optimized_time:.2f} ms for {iterations} calls ({optimized_time / iterations:.4f} ms/call)"
    )
    print(f"Actual logs written in Optimized run: {log_count}")

    improvement = baseline_time / optimized_time
    print(f"Speedup: {improvement:.2f}x")

    # Cleanup
    with contextlib.suppress(OSError):
        os.remove(LOG_FILE)


if __name__ == "__main__":
    run_benchmark()
