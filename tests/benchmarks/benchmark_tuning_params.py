import time
from typing import NamedTuple
from dataclasses import dataclass

class TuningParamsNT(NamedTuple):
    kp: float
    ki: float
    kd: float
    target_angle_offset: float

@dataclass
class TuningParamsDC:
    kp: float
    ki: float
    kd: float
    target_angle_offset: float

class TuningParamsSlots:
    __slots__ = ['kp', 'ki', 'kd', 'target_angle_offset']
    def __init__(self, kp, ki, kd, target_angle_offset):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target_angle_offset = target_angle_offset

def benchmark():
    iterations = 1_000_000

    print(f"Benchmarking {iterations} iterations...")

    # Baseline: NamedTuple instantiation
    start = time.perf_counter()
    for _ in range(iterations):
        t = TuningParamsNT(1.0, 0.1, 0.01, 0.0)
    end = time.perf_counter()
    nt_time = end - start
    print(f"Baseline (NamedTuple instantiation): {nt_time:.4f} s")

    # Option 1: Mutable object reuse (slots)
    t = TuningParamsSlots(1.0, 0.1, 0.01, 0.0)
    start = time.perf_counter()
    for _ in range(iterations):
        t.kp = 1.0
        t.ki = 0.1
        t.kd = 0.01
        t.target_angle_offset = 0.0
    end = time.perf_counter()
    slots_time = end - start
    print(f"Optimization (Mutable object reuse w/ slots): {slots_time:.4f} s")
    print(f"Improvement: {nt_time / slots_time:.2f}x faster")

    # Option 2: Dataclass reuse
    t = TuningParamsDC(1.0, 0.1, 0.01, 0.0)
    start = time.perf_counter()
    for _ in range(iterations):
        t.kp = 1.0
        t.ki = 0.1
        t.kd = 0.01
        t.target_angle_offset = 0.0
    end = time.perf_counter()
    dc_time = end - start
    print(f"Dataclass reuse: {dc_time:.4f} s")
    print(f"Improvement (Dataclass): {nt_time / dc_time:.2f}x faster")

if __name__ == "__main__":
    benchmark()
