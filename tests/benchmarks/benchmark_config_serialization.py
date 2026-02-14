import time
import sys
import json
from dataclasses import asdict
from pathlib import Path

# Add src to sys.path to import balance_bot
sys.path.append(str(Path(__file__).parent.parent.parent / "src"))

from balance_bot.config import RobotConfig, PIDParams, TunerConfig

def benchmark_config_serialization():
    config = RobotConfig(
        pid=PIDParams(kp=10.0, ki=0.5, kd=0.1),
        tuner=TunerConfig()
    )

    # Warmup
    for _ in range(100):
        _ = json.dumps(asdict(config))

    iterations = 1000

    # Measure asdict
    start_time = time.perf_counter()
    for _ in range(iterations):
        _ = asdict(config)
    asdict_time = (time.perf_counter() - start_time) / iterations

    # Measure json.dumps
    config_dict = asdict(config)
    start_time = time.perf_counter()
    for _ in range(iterations):
        _ = json.dumps(config_dict, indent=4)
    json_time = (time.perf_counter() - start_time) / iterations

    print(f"Average asdict() time: {asdict_time*1000:.4f} ms")
    print(f"Average json.dumps() time: {json_time*1000:.4f} ms")
    print(f"Total serialization time: {(asdict_time + json_time)*1000:.4f} ms")

if __name__ == "__main__":
    benchmark_config_serialization()
