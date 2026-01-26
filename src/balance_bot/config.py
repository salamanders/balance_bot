import json
import os
import sys
from dataclasses import dataclass

CONFIG_FILE = "pid_config.json"
FORCE_CALIB_FILE = "force_calibration.txt"


@dataclass
class PIDParams:
    kp: float = 5.0
    ki: float = 0.0
    kd: float = 0.0
    target_angle: float = 0.0


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

                    print(
                        f"-> Loaded Config: Kp={pid.kp:.2f} Ki={pid.ki:.2f} "
                        f"Kd={pid.kd:.2f} Target={pid.target_angle:.2f}"
                    )
            except (json.JSONDecodeError, OSError) as e:
                print(f"-> Error loading config: {e}")

        return cfg

    def save(self) -> None:
        data = {
            "pid_kp": self.pid.kp,
            "pid_ki": self.pid.ki,
            "pid_kd": self.pid.kd,
            "target_angle": self.pid.target_angle,
            "motor_l": self.motor_l,
            "motor_r": self.motor_r,
            "motor_l_invert": self.motor_l_invert,
            "motor_r_invert": self.motor_r_invert,
            "gyro_pitch_axis": self.gyro_pitch_axis,
            "gyro_pitch_invert": self.gyro_pitch_invert,
        }
        try:
            with open(CONFIG_FILE, "w") as f:
                json.dump(data, f)
            print("-> Config saved.")
        except OSError as e:
            print(f"-> Error saving config: {e}")
