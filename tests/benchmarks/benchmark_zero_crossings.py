import sys
import os
import time
import random

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../src')))

from balance_bot.adaptation.tuner import ContinuousTuner
from balance_bot.configuration import TunerConfig

def benchmark_zero_crossings():
    # Configure Tuner
    config = TunerConfig(analysis_interval=1)
    # Using larger buffer size to exaggerate the pairwise O(N) cost
    tuner = ContinuousTuner(config, buffer_size=1000)

    # Pre-fill buffer
    for _ in range(1000):
        tuner.update(random.uniform(-5.0, 5.0))

    iterations = 10000
    start_time = time.perf_counter()
    for _ in range(iterations):
        tuner.update(random.uniform(-0.5, 0.5))
        tuner.cooldown_timer = 0  # Force analysis
    end_time = time.perf_counter()

    duration = end_time - start_time
    print(f"Time for {iterations} iterations: {duration:.4f} seconds")

if __name__ == "__main__":
    benchmark_zero_crossings()
