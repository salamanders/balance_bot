import json
import sys
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any

CONFIG_FILE = Path("pid_config.json")
FORCE_CALIB_FILE = Path("force_calibration.txt")


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

    @classmethod
    def load(cls) -> "RobotConfig":
        pid = PIDParams()
        cfg = cls(pid=pid)

        if cls._check_force_calibration():
            return cfg

        if CONFIG_FILE.exists():
            try:
                text = CONFIG_FILE.read_text()
                data = json.loads(text)
                data = cls._migrate_legacy_data(data)

                # Apply data to config
                # Handle PID params
                if "pid" in data:
                    pid_data = data["pid"]
                    # Update existing pid object
                    for k, v in pid_data.items():
                        if hasattr(pid, k):
                            setattr(pid, k, v)

                # Handle root params
                for k, v in data.items():
                    if k != "pid" and hasattr(cfg, k):
                        setattr(cfg, k, v)

                print(
                    f"-> Loaded Config: Kp={pid.kp:.2f} Ki={pid.ki:.2f} "
                    f"Kd={pid.kd:.2f} Target={pid.target_angle:.2f}"
                )

            except (json.JSONDecodeError, OSError) as e:
                print(f"-> Error loading config: {e}")

        return cfg

    @staticmethod
    def _check_force_calibration() -> bool:
        force = False
        if FORCE_CALIB_FILE.exists():
            print(f"-> Force calibration file found: {FORCE_CALIB_FILE}")
            force = True
        if "--force-calibration" in sys.argv:
            print("-> Force calibration flag found")
            force = True
        return force

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
                del new_data[old_key] # Clean up root

        new_data["pid"] = pid_data
        return new_data

    def save(self) -> None:
        try:
            CONFIG_FILE.write_text(json.dumps(asdict(self), indent=4))
            print("-> Config saved.")
        except OSError as e:
            print(f"-> Error saving config: {e}")
