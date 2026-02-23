import json
import logging
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Optional

from pydantic import BaseModel, Field

from .enums import Axis

logger = logging.getLogger(__name__)
HARDWARE_CONFIG_FILE = Path("hardware_config.json")
LEARNING_STATE_FILE = Path("learning_state.json")

# Angle Thresholds (Degrees)
BALANCING_THRESHOLD = 15.0      # Normal operating range (+/-)
REST_ANGLE_MIN = 15.0           # Minimum angle to be considered "resting" on strut (covers ~20 deg)
REST_ANGLE_MAX = 55.0           # Maximum angle for resting (strut height)

# Startup / Recovery
STARTUP_RAMP_SPEED = 0.5        # Degrees per loop cycle to adjust setpoint during startup


class PIDParams(BaseModel):
    """
    PID Controller Parameters.
    """
    kp: float = 25.0
    ki: float = 0.0
    kd: float = 0.5
    target_angle: float = 0.0
    integral_limit: float = 20.0


class BatteryConfig(BaseModel):
    """
    Configuration for Battery Voltage Estimation and Compensation.
    """
    ema_alpha: float = 0.05
    factor_smoothing: float = 0.01
    min_compensation: float = 0.5
    max_compensation: float = 1.2
    min_pwm: float = 20.0
    baseline_samples: int = 100


class TunerConfig(BaseModel):
    """
    Configuration for Continuous Auto-Tuner.
    """
    cooldown_reset: int = 50
    oscillation_threshold: float = 0.15
    kp_oscillation_penalty: float = -0.1
    kd_oscillation_boost: float = 0.05
    stability_std_dev: float = 1.0
    stability_mean_err: float = 1.0
    kp_stability_boost: float = 0.02
    steady_error_threshold: float = 3.0
    ki_boost: float = 0.005
    crash_angle: float = 60.0
    analysis_interval: int = 10

    # Tuning Aggression Decay
    start_aggression_first_run: float = 5.0
    start_aggression_normal: float = 1.0
    aggression_decay: float = 0.9995
    min_aggression: float = 0.1

    # Balance Point Finder
    balance_check_interval: int = 500
    balance_learning_rate: float = 0.05
    balance_max_deviation: float = 10.0
    balance_motor_threshold: float = 5.0
    balance_pitch_rate_threshold: float = 10.0


class LedConfig(BaseModel):
    """
    Configuration for LED timings and patterns.
    """
    setup_blink_interval: float = 0.05
    tuning_blink_interval: float = 0.25
    countdown_blink_count_3: int = 3
    countdown_blink_count_2: int = 2
    countdown_blink_count_1: int = 1
    countdown_blink_on_time: float = 0.2
    countdown_blink_off_time: float = 0.2
    countdown_pause_time: float = 0.5


class ControlConfig(BaseModel):
    """
    General Control Logic Parameters (Immutable constants).
    """
    yaw_correction_factor: float = 0.5
    upright_threshold: float = 5.0
    low_battery_log_threshold: float = 0.95
    max_tilt_angle: float = 10.0
    turn_gain: float = 30.0
    soft_recovery_kp_threshold: float = 1.0


class LearnedControlParams(BaseModel):
    """
    Mutable control parameters learned via calibration.
    """
    kickup_power_forward: float = 0.0
    kickup_power_backward: float = 0.0
    backlash_pulse_time: float = 0.0


class SystemTiming(BaseModel):
    """
    System-wide Timing Constants.
    """
    setup_wait: float = 2.0
    calibration_pause: float = 1.0
    save_interval: float = 30.0
    battery_log_interval: float = 5.0
    tuning_log_interval: float = 1.0


@contextmanager
def temp_pid_overrides(pid_params: PIDParams, **overrides):
    """
    Context manager to temporarily override PID parameters.
    Restores original values on exit.
    """
    original_values = {}
    for k, v in overrides.items():
        if hasattr(pid_params, k):
            original_values[k] = getattr(pid_params, k)
            setattr(pid_params, k, v)
        else:
            logger.warning(f"PIDParams has no attribute '{k}', ignoring override.")

    try:
        yield
    finally:
        for k, v in original_values.items():
            setattr(pid_params, k, v)


class HardwareConfig(BaseModel):
    """
    Immutable Hardware Configuration.
    Contains physical mappings, constants, and system settings.
    This file is rarely changed after initial setup/wiring check.
    """
    # Mapping
    motor_l: Optional[int] = None
    motor_r: Optional[int] = None
    motor_l_invert: bool = False
    motor_r_invert: bool = False

    gyro_pitch_axis: Optional[Axis] = None
    gyro_pitch_invert: bool = False
    gyro_yaw_axis: Optional[Axis] = None
    gyro_yaw_invert: bool = False
    gyro_roll_axis: Optional[Axis] = None
    gyro_roll_invert: bool = False

    accel_vertical_axis: Optional[Axis] = None
    accel_vertical_invert: bool = False
    accel_forward_axis: Optional[Axis] = None
    accel_forward_invert: bool = False

    motor_i2c_bus: Optional[int] = None
    imu_i2c_bus: Optional[int] = None

    # System Constants
    loop_time: float = 0.01
    crash_angle: float = 50.0
    complementary_alpha: float = 0.98
    vibration_threshold: int = 10
    imu_max_retries: int = 5
    min_power_visible: int = 0  # Discovered during WiringCheck, but physically tied to motor type

    # Sub-Configs (Immutable)
    battery: BatteryConfig = Field(default_factory=BatteryConfig)
    tuner: TunerConfig = Field(default_factory=TunerConfig)
    led: LedConfig = Field(default_factory=LedConfig)
    control: ControlConfig = Field(default_factory=ControlConfig)
    timing: SystemTiming = Field(default_factory=SystemTiming)

    # Verification Flags (Tied to Hardware Setup)
    # If hardware changes, these become invalid, so they belong here.
    motor_phasing_verified: bool = False
    motor_direction_verified: bool = False
    motor_channels_verified: bool = False
    motor_trim_verified: bool = False
    backlash_verified: bool = False

    class Config:
        frozen = True  # Enforce Immutability

    @classmethod
    def load(cls) -> "HardwareConfig":
        """Load hardware configuration from disk."""
        if HARDWARE_CONFIG_FILE.exists():
            try:
                content = HARDWARE_CONFIG_FILE.read_text()
                if not content.strip():
                    return cls()
                return cls.model_validate_json(content)
            except Exception as e:
                logger.error(f"Error loading HardwareConfig: {e}. Using defaults.")
        return cls()

    def save(self) -> None:
        """Serialize and save to disk."""
        try:
            HARDWARE_CONFIG_FILE.write_text(self.model_dump_json(indent=4))
            logger.info("HardwareConfig saved.")
        except OSError as e:
            logger.error(f"Error saving HardwareConfig: {e}")


class LearningState(BaseModel):
    """
    Mutable Learning State.
    Contains parameters that evolve over time: PID, Calibration, Tuning.
    This file is updated frequently.
    """
    pid: PIDParams = Field(default_factory=PIDParams)
    control: LearnedControlParams = Field(default_factory=LearnedControlParams)

    # Calibration Data
    rest_angle_forward: Optional[float] = None
    rest_angle_backward: Optional[float] = None
    gyro_bias_x: float = 0.0
    gyro_bias_y: float = 0.0
    gyro_bias_z: float = 0.0
    motor_trim: float = 0.0

    # State Flags
    balance_verified: bool = False

    @classmethod
    def load(cls) -> "LearningState":
        """Load learning state from disk."""
        if LEARNING_STATE_FILE.exists():
            try:
                content = LEARNING_STATE_FILE.read_text()
                if not content.strip():
                    return cls()
                return cls.model_validate_json(content)
            except Exception as e:
                logger.error(f"Error loading LearningState: {e}. Using defaults.")
        return cls()

    def save(self) -> None:
        """Serialize and save to disk."""
        try:
            LEARNING_STATE_FILE.write_text(self.model_dump_json(indent=4))
            # logger.info("LearningState saved.") # Reduce spam for frequent saves
        except OSError as e:
            logger.error(f"Error saving LearningState: {e}")
