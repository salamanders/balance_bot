import time
from dataclasses import dataclass

# Baseline: Frozen Dataclass (Simulates current implementation)
@dataclass(frozen=True)
class MotionRequestBaseline:
    velocity: float = 0.0
    turn_rate: float = 0.0
    enable_control: bool = True

# Optimization 1: Mutable Dataclass
@dataclass
class MotionRequestMutable:
    velocity: float = 0.0
    turn_rate: float = 0.0
    enable_control: bool = True

# Optimization 2: Mutable Dataclass with slots
# Note: slots=True is available in Python 3.10+
@dataclass(slots=True)
class MotionRequestSlots:
    velocity: float = 0.0
    turn_rate: float = 0.0
    enable_control: bool = True

def benchmark():
    iterations = 1_000_000

    print(f"Benchmarking {iterations} iterations...")

    # Baseline: Instantiation
    start = time.perf_counter()
    for _ in range(iterations):
        _ = MotionRequestBaseline(velocity=0.0, turn_rate=0.0, enable_control=True)
    end = time.perf_counter()
    baseline_time = end - start
    print(f"Baseline (Frozen Dataclass instantiation): {baseline_time:.4f} s")

    # Optimization 1: Reuse Mutable Dataclass
    m1 = MotionRequestMutable()
    start = time.perf_counter()
    for _ in range(iterations):
        m1.velocity = 0.0
        m1.turn_rate = 0.0
        m1.enable_control = True
    end = time.perf_counter()
    mutable_time = end - start
    print(f"Optimization 1 (Mutable Dataclass reuse): {mutable_time:.4f} s")
    print(f"Improvement: {baseline_time / mutable_time:.2f}x faster")

    # Optimization 2: Reuse Mutable Dataclass with slots
    m2 = MotionRequestSlots()
    start = time.perf_counter()
    for _ in range(iterations):
        m2.velocity = 0.0
        m2.turn_rate = 0.0
        m2.enable_control = True
    end = time.perf_counter()
    slots_time = end - start
    print(f"Optimization 2 (Mutable Dataclass w/ slots reuse): {slots_time:.4f} s")
    print(f"Improvement: {baseline_time / slots_time:.2f}x faster")

if __name__ == "__main__":
    benchmark()
