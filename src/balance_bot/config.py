import json
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any

CONFIG_FILE = Path("pid_config.json")


@dataclass
class PIDParams:
    kp: float = 5.0
    ki: float = 0.0
    kd: float = 0.0
    target_angle: float = 0.0
    integral_limit: float = 20.0


@dataclass
class RobotConfig:
    pid: PIDParams
    motor_l: int = 0
    motor_r: int = 1
    motor_l_invert: bool = False
    motor_r_invert: bool = False
    gyro_pitch_axis: str = "x"
    gyro_pitch_invert: bool = False
    loop_time: float = 0.01  # 10ms

    # Operational Parameters
    fall_angle_limit: float = 45.0
    complementary_alpha: float = 0.98
    vibration_threshold: int = 10

    @classmethod
    def load(cls) -> "RobotConfig":
        # Start with defaults
        pid_params = PIDParams()
        config_kwargs = {}

        if CONFIG_FILE.exists():
            try:
                text = CONFIG_FILE.read_text()
                data = json.loads(text)
                data = cls._migrate_legacy_data(data)

                # 1. Handle PID
                if "pid" in data and isinstance(data["pid"], dict):
                    pid_data = data["pid"]
                    # Filter keys valid for PIDParams
                    valid_pid = {k: v for k, v in pid_data.items() if k in PIDParams.__annotations__}
                    pid_params = PIDParams(**valid_pid)

                # 2. Handle Root Config
                # Filter keys valid for RobotConfig
                valid_config = {k: v for k, v in data.items() if k in RobotConfig.__annotations__ and k != "pid"}
                config_kwargs = valid_config

                print(
                    f"-> Loaded Config: Kp={pid_params.kp:.2f} Ki={pid_params.ki:.2f} "
                    f"Kd={pid_params.kd:.2f} Target={pid_params.target_angle:.2f}"
                )

            except (json.JSONDecodeError, OSError) as e:
                print(f"-> Error loading config: {e}")

        return cls(pid=pid_params, **config_kwargs)

    @staticmethod
    def _migrate_legacy_data(data: dict[str, Any]) -> dict[str, Any]:
        """Convert flat legacy format to nested format if needed."""
        if "pid" in data:
            return data

        # Legacy Flat Format -> Nested
        new_data = data.copy()
        pid_data = {}

        # Map legacy keys
        legacy_map = {
            "pid_kp": "kp",
            "pid_ki": "ki",
            "pid_kd": "kd",
            "target_angle": "target_angle"
        }

        for old_key, new_key in legacy_map.items():
            if old_key in data:
                pid_data[new_key] = data[old_key]
                del new_data[old_key]  # Clean up root

        new_data["pid"] = pid_data
        return new_data

    def save(self) -> None:
        try:
            CONFIG_FILE.write_text(json.dumps(asdict(self), indent=4))
            print("-> Config saved.")
        except OSError as e:
            print(f"-> Error saving config: {e}")
