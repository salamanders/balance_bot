
import time
import random
import statistics
import sys
import os

# Ensure src is in path for standalone execution
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../src')))

try:
    from balance_bot.adaptation.tuner import ContinuousTuner, TunerConfig
except ImportError:
    from src.balance_bot.adaptation.tuner import ContinuousTuner, TunerConfig

def benchmark_tuner():
    # Setup
    config = TunerConfig()
    tuner = ContinuousTuner(config, buffer_size=100)

    # Fill buffer first to ensure we are hitting the analysis code
    for _ in range(100):
        tuner.update(random.uniform(-5.0, 5.0))

    # Reset cooldown just in case
    tuner.cooldown_timer = 0

    start_time = time.perf_counter()
    iterations = 10000

    for _ in range(iterations):
        # Generate random errors that might trigger analysis but hopefully not too many "tuned" events that set cooldown
        # We want to stress the analysis part.
        # If we trigger 'tuned', cooldown is set to 50, which skips analysis.
        # So we should force cooldown to 0 every time to measure the worst case (analysis every step).

        tuner.update(random.uniform(-0.5, 0.5))
        tuner.cooldown_timer = 0 # Force analysis on every step for benchmarking

    end_time = time.perf_counter()

    duration = end_time - start_time
    print(f"Time for {iterations} iterations: {duration:.4f} seconds")
    print(f"Average time per call: {duration/iterations*1000000:.2f} microseconds")

if __name__ == "__main__":
    benchmark_tuner()
