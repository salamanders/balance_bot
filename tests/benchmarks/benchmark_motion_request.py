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

# Optimization 2: Mutable Class with explicit __slots__
class MotionRequestSlots:
    __slots__ = ['velocity', 'turn_rate', 'enable_control']

    def __init__(self, velocity: float = 0.0, turn_rate: float = 0.0, enable_control: bool = True):
        self.velocity = velocity
        self.turn_rate = turn_rate
        self.enable_control = enable_control

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
    _m1 = MotionRequestMutable()
    start = time.perf_counter()
    for _ in range(iterations):
        _m1.velocity = 0.0
        _m1.turn_rate = 0.0
        _m1.enable_control = True
    end = time.perf_counter()
    mutable_time = end - start
    print(f"Optimization 1 (Mutable Dataclass reuse): {mutable_time:.4f} s")
    print(f"Improvement: {baseline_time / mutable_time:.2f}x faster")

    # Optimization 2: Reuse Mutable Dataclass with slots
    _m2 = MotionRequestSlots()
    start = time.perf_counter()
    for _ in range(iterations):
        _m2.velocity = 0.0
        _m2.turn_rate = 0.0
        _m2.enable_control = True
    end = time.perf_counter()
    slots_time = end - start
    print(f"Optimization 2 (Mutable Dataclass w/ slots reuse): {slots_time:.4f} s")
    print(f"Improvement: {baseline_time / slots_time:.2f}x faster")

if __name__ == "__main__":
    benchmark()
