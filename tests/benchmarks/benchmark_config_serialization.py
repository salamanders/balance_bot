from typing import Any
import time
import sys
import json
from pathlib import Path

# Add src to sys.path to import balance_bot
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "src"))

from balance_bot.configuration import LearningState, PIDParams, TunerConfig

def benchmark_config_serialization() -> Any:
    config = LearningState(
        pid=PIDParams(kp=10.0, ki=0.5, kd=0.1),
        tuner=TunerConfig()
    )

    # Warmup
    for _ in range(100):
        _ = json.dumps(config.model_dump())

    iterations = 1000

    # Measure model_dump
    start_time = time.perf_counter()
    for _ in range(iterations):
        _ = config.model_dump()
    asdict_time = (time.perf_counter() - start_time) / iterations

    # Measure json.dumps
    config_dict = config.model_dump()
    start_time = time.perf_counter()
    for _ in range(iterations):
        _ = json.dumps(config_dict, indent=4)
    json_time = (time.perf_counter() - start_time) / iterations

    print(f"Average model_dump() time: {asdict_time*1000:.4f} ms")
    print(f"Average json.dumps() time: {json_time*1000:.4f} ms")
    print(f"Total serialization time: {(asdict_time + json_time)*1000:.4f} ms")

if __name__ == "__main__":
    benchmark_config_serialization()
