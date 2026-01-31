import json
import logging
from contextlib import contextmanager
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)
CONFIG_FILE = Path("pid_config.json")


@dataclass
class PIDParams:
    kp: float = 25.0
    ki: float = 0.0
    kd: float = 0.5
    target_angle: float = 0.0
    integral_limit: float = 20.0


@dataclass(frozen=True)
class BatteryConfig:
    ema_alpha: float = 0.05
    factor_smoothing: float = 0.01
    min_compensation: float = 0.5
    max_compensation: float = 1.2
    min_pwm: float = 20.0
    baseline_samples: int = 100


@dataclass(frozen=True)
class TunerConfig:
    cooldown_reset: int = 50
    oscillation_threshold: float = 0.15
    kp_oscillation_penalty: float = -0.1
    kd_oscillation_boost: float = 0.05
    stability_std_dev: float = 1.0
    stability_mean_err: float = 1.0
    kp_stability_boost: float = 0.02
    steady_error_threshold: float = 3.0
    ki_boost: float = 0.005


@dataclass(frozen=True)
class LedConfig:
    setup_blink_interval: float = 0.05
    tuning_blink_interval: float = 0.25
    countdown_blink_count_3: int = 3
    countdown_blink_count_2: int = 2
    countdown_blink_count_1: int = 1
    countdown_blink_on_time: float = 0.2
    countdown_blink_off_time: float = 0.2
    countdown_pause_time: float = 0.5


@dataclass(frozen=True)
class ControlConfig:
    yaw_correction_factor: float = 0.5
    upright_threshold: float = 5.0
    low_battery_log_threshold: float = 0.95


@dataclass(frozen=True)
class SystemTiming:
    setup_wait: float = 2.0
    calibration_pause: float = 1.0
    save_interval: float = 30.0
    battery_log_interval: float = 5.0


@dataclass(frozen=True)
class TuningHeuristics:
    kp_increment: float = 0.05
    kp_reduction: float = 0.6
    kd_ratio: float = 0.05
    ki_ratio: float = 0.005


SYSTEM_TIMING = SystemTiming()
TUNING_HEURISTICS = TuningHeuristics()


@contextmanager
def temp_pid_overrides(pid_params: PIDParams, **overrides):
    """
    Context manager to temporarily override PID parameters.
    Restores original values on exit.
    """
    original_values = {}
    # Save originals
    for k, v in overrides.items():
        if hasattr(pid_params, k):
            original_values[k] = getattr(pid_params, k)
            setattr(pid_params, k, v)
        else:
            logger.warning(f"PIDParams has no attribute '{k}', ignoring override.")

    try:
        yield
    finally:
        # Restore originals
        for k, v in original_values.items():
            setattr(pid_params, k, v)


@dataclass
class RobotConfig:
    pid: PIDParams
    battery: BatteryConfig = BatteryConfig()
    tuner: TunerConfig = TunerConfig()
    led: LedConfig = LedConfig()
    control: ControlConfig = ControlConfig()
    motor_l: int = 0
    motor_r: int = 1
    motor_l_invert: bool = False
    motor_r_invert: bool = False
    gyro_pitch_axis: str = "x"
    gyro_pitch_invert: bool = False
    i2c_bus: int = 1
    loop_time: float = 0.01  # 10ms

    # Operational Parameters
    fall_angle_limit: float = 45.0
    complementary_alpha: float = 0.98
    vibration_threshold: int = 10

    @staticmethod
    def _filter_keys(dataclass_type, data: dict[str, Any]) -> dict[str, Any]:
        """Helper to filter dictionary keys based on dataclass annotations."""
        return {k: v for k, v in data.items() if k in dataclass_type.__annotations__}

    @classmethod
    def load(cls) -> "RobotConfig":
        # Start with defaults
        pid_params = PIDParams()
        battery_config = BatteryConfig()
        tuner_config = TunerConfig()
        led_config = LedConfig()
        control_config = ControlConfig()
        config_kwargs = {}

        if CONFIG_FILE.exists():
            try:
                text = CONFIG_FILE.read_text()
                data = json.loads(text)
                data = cls._migrate_legacy_data(data)

                # 1. Handle PID
                pid_data = data.pop("pid", {})
                if isinstance(pid_data, dict):
                    pid_params = PIDParams(**cls._filter_keys(PIDParams, pid_data))

                # 2. Handle Battery
                bat_data = data.pop("battery", {})
                if isinstance(bat_data, dict):
                    battery_config = BatteryConfig(
                        **cls._filter_keys(BatteryConfig, bat_data)
                    )

                # 3. Handle Tuner
                tune_data = data.pop("tuner", {})
                if isinstance(tune_data, dict):
                    tuner_config = TunerConfig(
                        **cls._filter_keys(TunerConfig, tune_data)
                    )

                # 4. Handle Led
                led_data = data.pop("led", {})
                if isinstance(led_data, dict):
                    led_config = LedConfig(**cls._filter_keys(LedConfig, led_data))

                # 5. Handle Control
                ctrl_data = data.pop("control", {})
                if isinstance(ctrl_data, dict):
                    control_config = ControlConfig(
                        **cls._filter_keys(ControlConfig, ctrl_data)
                    )

                # 6. Handle Root Config
                config_kwargs = cls._filter_keys(RobotConfig, data)

                logger.info(
                    f"Loaded Config: Kp={pid_params.kp:.2f} Ki={pid_params.ki:.2f} "
                    f"Kd={pid_params.kd:.2f} Target={pid_params.target_angle:.2f}"
                )

            except (json.JSONDecodeError, OSError) as e:
                logger.error(f"Error loading config: {e}")

        return cls(
            pid=pid_params,
            battery=battery_config,
            tuner=tuner_config,
            led=led_config,
            control=control_config,
            **config_kwargs,
        )

    @staticmethod
    def _migrate_legacy_data(data: dict[str, Any]) -> dict[str, Any]:
        """Convert flat legacy format to nested format if needed."""
        if "pid" in data:
            return data

        # Legacy Flat Format -> Nested
        new_data = data.copy()

        legacy_map = {
            "pid_kp": "kp",
            "pid_ki": "ki",
            "pid_kd": "kd",
            "target_angle": "target_angle",
        }

        pid_data = {
            new_key: new_data.pop(old_key)
            for old_key, new_key in legacy_map.items()
            if old_key in new_data
        }

        if pid_data:
            new_data["pid"] = pid_data

        return new_data

    def save(self) -> None:
        try:
            CONFIG_FILE.write_text(json.dumps(asdict(self), indent=4))
            logger.info("Config saved.")
        except OSError as e:
            logger.error(f"Error saving config: {e}")
