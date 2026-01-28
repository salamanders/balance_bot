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
    """
    Configuration for the PID controller.
    PID stands for Proportional, Integral, Derivative.
    These coefficients determine how the robot reacts to errors.
    """

    kp: float = 5.0
    """Proportional Gain. Reacts to the current error. Higher = stiffer but more oscillation."""

    ki: float = 0.0
    """Integral Gain. Reacts to accumulated error over time. Helps correct steady-state leaning."""

    kd: float = 0.0
    """Derivative Gain. Reacts to the rate of change of error. Dampens oscillations."""

    target_angle: float = 0.0
    """The desired pitch angle (degrees) where the robot is balanced (vertical)."""

    integral_limit: float = 20.0
    """Max limit for the integral term to prevent 'integral windup' (accumulation of huge error)."""


@dataclass(frozen=True)
class BatteryConfig:
    """
    Configuration for the Battery Estimator.
    This system estimates voltage drop by comparing motor command vs. actual acceleration.
    """

    ema_alpha: float = 0.05
    """Smoothing factor for the responsiveness estimator (0.0 to 1.0). Lower = slower updates."""

    factor_smoothing: float = 0.01
    """
    Smoothing factor for the final compensation multiplier.
    This needs to be very slow (low value) to prevent positive feedback loops.
    """

    min_compensation: float = 0.5
    """Minimum allowed compensation factor. Prevents the system from boosting power too dangerously high."""

    max_compensation: float = 1.2
    """Maximum allowed compensation factor. If battery is 'fresh', we might reduce power slightly."""

    min_pwm: float = 20.0
    """Minimum PWM command (0-100) required to consider a sample valid for estimation."""

    baseline_samples: int = 100
    """Number of initial samples to collect to establish a 'full battery' baseline."""


@dataclass(frozen=True)
class TunerConfig:
    """
    Configuration for the Continuous Tuner.
    Heuristic parameters for detecting oscillations, stability, and drift.
    """

    cooldown_reset: int = 50
    """Number of loops to wait after a tuning adjustment before analyzing again."""

    oscillation_threshold: float = 0.15
    """
    Fraction of samples (0.0 to 1.0) that are zero-crossings.
    If > 15% of samples are zero crossings, we assume oscillation.
    """

    kp_oscillation_penalty: float = -0.1
    """Amount to reduce Kp when oscillation is detected."""

    kd_oscillation_boost: float = 0.05
    """Amount to increase Kd when oscillation is detected (damping)."""

    stability_std_dev: float = 1.0
    """Standard deviation of error (degrees) below which we consider the robot 'stable'."""

    stability_mean_err: float = 1.0
    """Mean error (degrees) below which we consider the robot 'upright'."""

    kp_stability_boost: float = 0.02
    """Amount to increase Kp when the robot is stable (tightening control)."""

    steady_error_threshold: float = 3.0
    """Mean error (degrees) above which we assume the robot is leaning (needs I term)."""

    ki_boost: float = 0.005
    """Amount to increase Ki when steady-state error is detected."""


@dataclass(frozen=True)
class SystemTiming:
    """
    Global timing constants for the application (in Seconds).
    """

    setup_wait: float = 2.0
    """Time to wait during startup to let sensors stabilize."""

    calibration_pause: float = 1.0
    """Time to pause after calibration before starting tuning/balancing."""

    save_interval: float = 30.0
    """Interval (seconds) between auto-saving configuration to disk."""

    battery_log_interval: float = 5.0
    """Interval (seconds) between low-battery log warnings."""


@dataclass(frozen=True)
class TuningHeuristics:
    """
    Constants for the initial 'Run Tune' phase (Ziegler-Nichols inspired).
    """

    kp_increment: float = 0.05
    """Amount to increase Kp per loop while searching for oscillation."""

    kp_reduction: float = 0.6
    """Factor to reduce Kp by once oscillation is found (Classic Z-N uses 0.6)."""

    kd_ratio: float = 0.05
    """Ratio to set Kd relative to the critical Kp value."""

    ki_ratio: float = 0.005
    """Ratio to set Ki relative to the critical Kp value."""


SYSTEM_TIMING = SystemTiming()
TUNING_HEURISTICS = TuningHeuristics()


@contextmanager
def temp_pid_overrides(pid_params: PIDParams, **overrides):
    """
    Context manager to temporarily override PID parameters.
    Restores original values on exit.

    Usage:
        with temp_pid_overrides(params, ki=0, kd=0):
            # do something with simplified PID
        # params are restored here
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
    """
    Master Configuration class aggregating all sub-configs.
    Also handles hardware mapping and persistence.
    """

    pid: PIDParams
    battery: BatteryConfig = BatteryConfig()
    tuner: TunerConfig = TunerConfig()

    # --- Hardware Mapping ---
    motor_l: int = 0
    """Channel index for Left Motor."""

    motor_r: int = 1
    """Channel index for Right Motor."""

    motor_l_invert: bool = False
    """True if Left Motor is wired backwards."""

    motor_r_invert: bool = False
    """True if Right Motor is wired backwards."""

    gyro_pitch_axis: str = "x"
    """Which IMU axis corresponds to pitch ('x' or 'y')."""

    gyro_pitch_invert: bool = False
    """True if the gyro axis needs to be negated to match pitch direction."""

    loop_time: float = 0.01
    """Main control loop duration in Seconds (0.01s = 100Hz)."""

    # --- Operational Parameters ---
    fall_angle_limit: float = 45.0
    """Angle (degrees) at which the robot gives up and cuts power."""

    complementary_alpha: float = 0.98
    """
    Complementary Filter Coefficient (0.0 to 1.0).
    Higher value = Trust Gyro (smooth but drifts) more.
    Lower value = Trust Accelerometer (noisy but stable) more.
    0.98 is standard for this application.
    """

    vibration_threshold: int = 10
    """Number of zero-crossings required to trigger 'Oscillation Detected' state during initial tuning."""

    @staticmethod
    def _filter_keys(dataclass_type, data: dict[str, Any]) -> dict[str, Any]:
        """Helper to filter dictionary keys based on dataclass annotations."""
        return {k: v for k, v in data.items() if k in dataclass_type.__annotations__}

    @classmethod
    def load(cls) -> "RobotConfig":
        """
        Load configuration from `pid_config.json`.
        Handles migration from legacy formats.
        Returns default config if file is missing or invalid.
        """
        # Start with defaults
        pid_params = PIDParams()
        battery_config = BatteryConfig()
        tuner_config = TunerConfig()
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

                # 4. Handle Root Config
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
        """Save the current configuration to `pid_config.json`."""
        try:
            CONFIG_FILE.write_text(json.dumps(asdict(self), indent=4))
            logger.info("Config saved.")
        except OSError as e:
            logger.error(f"Error saving config: {e}")
