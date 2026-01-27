import json
import os
import sys
from dataclasses import dataclass, asdict

CONFIG_FILE = "pid_config.json"
FORCE_CALIB_FILE = "force_calibration.txt"


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
        force = False

        if os.path.exists(FORCE_CALIB_FILE):
            print(f"-> Force calibration file found: {FORCE_CALIB_FILE}")
            force = True
        if "--force-calibration" in sys.argv:
            print("-> Force calibration flag found")
            force = True

        if not force and os.path.exists(CONFIG_FILE):
            try:
                with open(CONFIG_FILE, "r") as f:
                    data = json.load(f)

                    # Check for nested structure (New format)
                    if "pid" in data:
                        pid_data = data["pid"]
                        pid.kp = pid_data.get("kp", pid.kp)
                        pid.ki = pid_data.get("ki", pid.ki)
                        pid.kd = pid_data.get("kd", pid.kd)
                        pid.target_angle = pid_data.get("target_angle", pid.target_angle)
                        pid.integral_limit = pid_data.get("integral_limit", pid.integral_limit)
                    else:
                        # Legacy Flat Format
                        pid.kp = data.get("pid_kp", pid.kp)
                        pid.ki = data.get("pid_ki", pid.ki)
                        pid.kd = data.get("pid_kd", pid.kd)
                        pid.target_angle = data.get("target_angle", pid.target_angle)

                    cfg.motor_l = data.get("motor_l", cfg.motor_l)
                    cfg.motor_r = data.get("motor_r", cfg.motor_r)
                    cfg.motor_l_invert = data.get("motor_l_invert", cfg.motor_l_invert)
                    cfg.motor_r_invert = data.get("motor_r_invert", cfg.motor_r_invert)
                    cfg.gyro_pitch_axis = data.get("gyro_pitch_axis", cfg.gyro_pitch_axis)
                    cfg.gyro_pitch_invert = data.get("gyro_pitch_invert", cfg.gyro_pitch_invert)
                    cfg.loop_time = data.get("loop_time", cfg.loop_time)

                    print(
                        f"-> Loaded Config: Kp={pid.kp:.2f} Ki={pid.ki:.2f} "
                        f"Kd={pid.kd:.2f} Target={pid.target_angle:.2f}"
                    )
            except (json.JSONDecodeError, OSError) as e:
                print(f"-> Error loading config: {e}")

        return cfg

    def save(self) -> None:
        try:
            with open(CONFIG_FILE, "w") as f:
                json.dump(asdict(self), f, indent=4)
            print("-> Config saved.")
        except OSError as e:
            print(f"-> Error saving config: {e}")
