import timeit
import sys
import os

# Add src to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../src')))

from balance_bot.adaptation.tuner import ContinuousTuner
from balance_bot.config import TunerConfig

def benchmark_tuner_update():
    buffer_size = 50000
    iterations = 5000

    # Configure Tuner
    # Use analysis_interval=1000000 to avoid heavy statistics calculation affecting the measurement of append/pop
    config = TunerConfig(analysis_interval=1000000)
    tuner = ContinuousTuner(config, buffer_size=buffer_size)

    # Pre-fill buffer with small values
    print(f"Pre-filling buffer with {buffer_size} elements...")
    for i in range(buffer_size):
        tuner.update(1.0)

    # Ensure buffer is full
    if len(tuner.errors) != buffer_size:
        print(f"Error: Buffer size mismatch: {len(tuner.errors)} != {buffer_size}")
        # Debugging
        tuner.update(1.0)
        print(f"After one more update: {len(tuner.errors)}")
        exit(1)

    print(f"Running {iterations} iterations of update()...")

    def run_update():
        for i in range(iterations):
            tuner.update(1.0)

    # Measure time
    execution_time = timeit.timeit(run_update, number=1)
    print(f"Time taken: {execution_time:.6f} seconds")
    return execution_time

if __name__ == "__main__":
    benchmark_tuner_update()
