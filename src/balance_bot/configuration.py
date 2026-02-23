import json
import logging
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Optional

from .enums import Axis

logger = logging.getLogger(__name__)

HARDWARE_CONFIG_FILE = Path("hardware_config.json")
LEARNING_STATE_FILE = Path("learning_state.json")

# Angle Thresholds (Degrees) - Kept as constants for now if not configurable
BALANCING_THRESHOLD = 15.0
REST_ANGLE_MIN = 15.0
REST_ANGLE_MAX = 55.0

# Startup / Recovery
STARTUP_RAMP_SPEED = 0.5

@dataclass(frozen=True, kw_only=True)
class BatteryConfig:
    """Configuration for Battery Voltage Estimation and Compensation."""
    ema_alpha: float = 0.05
    factor_smoothing: float = 0.01
    min_compensation: float = 0.5
    max_compensation: float = 1.2
    min_pwm: float = 20.0
    baseline_samples: int = 100

@dataclass(frozen=True, kw_only=True)
class TunerConfig:
    """Configuration for Continuous Auto-Tuner."""
    cooldown_reset: int = 50
    oscillation_threshold: float = 0.15
    kp_oscillation_penalty: float = -0.1
    kd_oscillation_boost: float = 0.05
    stability_std_dev: float = 1.0
    stability_mean_err: float = 1.0
    kp_stability_boost: float = 0.02
    steady_error_threshold: float = 3.0
    ki_boost: float = 0.005
    crash_angle: float = 60.0 # Note: crash_angle is also in RobotConfig as operational param?
                              # RobotConfig had crash_angle=50.0, TunerConfig=60.0.
                              # Agent uses self.config.crash_angle (50.0).
                              # TunerConfig is for Tuner logic. I'll keep both for now to match structure.
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

@dataclass(frozen=True, kw_only=True)
class LedConfig:
    """Configuration for LED timings and patterns."""
    setup_blink_interval: float = 0.05
    tuning_blink_interval: float = 0.25
    countdown_blink_count_3: int = 3
    countdown_blink_count_2: int = 2
    countdown_blink_count_1: int = 1
    countdown_blink_on_time: float = 0.2
    countdown_blink_off_time: float = 0.2
    countdown_pause_time: float = 0.5

@dataclass(frozen=True, kw_only=True)
class ControlConfig:
    """General Control Logic Parameters."""
    yaw_correction_factor: float = 0.5
    upright_threshold: float = 5.0
    low_battery_log_threshold: float = 0.95
    kickup_power_forward: float = 0.0
    kickup_power_backward: float = 0.0
    max_tilt_angle: float = 10.0
    turn_gain: float = 30.0
    soft_recovery_kp_threshold: float = 1.0
    backlash_pulse_time: float = 0.0
    integral_limit: float = 20.0  # Moved from PIDParams

@dataclass(frozen=True, kw_only=True)
class SystemTiming:
    """System-wide Timing Constants."""
    setup_wait: float = 2.0
    calibration_pause: float = 1.0
    save_interval: float = 30.0
    battery_log_interval: float = 5.0
    tuning_log_interval: float = 1.0

@dataclass(slots=True, kw_only=True)
class LearningState:
    """
    Mutable Runtime State.
    Contains only values that change during operation (PID gains, target angle).
    """
    kp: float = 25.0
    ki: float = 0.0
    kd: float = 0.5
    target_angle: float = 0.0
    balance_verified: bool = False

    @classmethod
    def load(cls) -> "LearningState":
        """Load state from disk."""
        if LEARNING_STATE_FILE.exists():
            try:
                content = LEARNING_STATE_FILE.read_text()
                if content.strip():
                    data = json.loads(content)
                    return cls(**data)
            except Exception as e:
                logger.error(f"Error loading LearningState: {e}. Using defaults.")
        return cls()

    def save(self) -> None:
        """Save state to disk."""
        try:
            # Use asdict for serialization
            data = asdict(self)
            LEARNING_STATE_FILE.write_text(json.dumps(data, indent=4))
        except OSError as e:
            logger.error(f"Error saving LearningState: {e}")


@dataclass(frozen=True, kw_only=True)
class HardwareConfig:
    """
    Immutable Hardware Configuration.
    Loaded once at startup. Contains hardware mapping and calibration.
    """
    # Sub-Configs
    battery: BatteryConfig = field(default_factory=BatteryConfig)
    tuner: TunerConfig = field(default_factory=TunerConfig)
    led: LedConfig = field(default_factory=LedConfig)
    control: ControlConfig = field(default_factory=ControlConfig)
    timing: SystemTiming = field(default_factory=SystemTiming)

    # Hardware Mapping
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

    loop_time: float = 0.01

    # Operational Parameters (Hardware Limits)
    crash_angle: float = 50.0
    complementary_alpha: float = 0.98
    vibration_threshold: int = 10
    imu_max_retries: int = 5
    min_power_visible: int = 0
    motor_trim: float = 0.0

    # Calibrated Rest Angles
    rest_angle_forward: Optional[float] = None
    rest_angle_backward: Optional[float] = None

    # Discovery Flags (Once verified, these shouldn't change unless hardware changes)
    motor_phasing_verified: bool = False
    motor_direction_verified: bool = False
    motor_channels_verified: bool = False
    motor_trim_verified: bool = False
    backlash_verified: bool = False

    @classmethod
    def load(cls) -> "HardwareConfig":
        """Load configuration from disk."""
        if HARDWARE_CONFIG_FILE.exists():
            try:
                content = HARDWARE_CONFIG_FILE.read_text()
                if content.strip():
                    data = json.loads(content)

                    # Helper to convert nested dicts to dataclasses
                    # We need to handle nested dataclasses explicitly if json.loads returns dicts
                    # But since we use default_factory, we can just pass the dict if the keys match?
                    # No, we need to instantiate the sub-objects.

                    # Handle Axis enums
                    for key in ['gyro_pitch_axis', 'gyro_yaw_axis', 'gyro_roll_axis',
                               'accel_vertical_axis', 'accel_forward_axis']:
                        if key in data and data[key] is not None:
                             try:
                                 data[key] = Axis(data[key])
                             except ValueError:
                                 logger.warning(f"Invalid Axis value for {key}: {data[key]}")
                                 data[key] = None

                    # Handle Sub-Configs
                    if 'battery' in data: data['battery'] = BatteryConfig(**data['battery'])
                    if 'tuner' in data: data['tuner'] = TunerConfig(**data['tuner'])
                    if 'led' in data: data['led'] = LedConfig(**data['led'])
                    if 'control' in data: data['control'] = ControlConfig(**data['control'])
                    if 'timing' in data: data['timing'] = SystemTiming(**data['timing'])

                    return cls(**data)
            except Exception as e:
                logger.error(f"Error loading HardwareConfig: {e}. Using defaults.")

        return cls()

    def save(self) -> None:
        """Serialize and save configuration to disk."""
        try:
            # We need a custom encoder for Enums and nested dataclasses if asdict doesn't handle them perfectly for JSON
            # asdict converts nested dataclasses to dicts, which is good.
            # Enums need to be converted to values.

            data = asdict(self)

            # recursive function to handle Enum serialization if json.dumps doesn't (it usually doesn't for Enum members)
            def convert(o):
                if isinstance(o, dict):
                    return {k: convert(v) for k, v in o.items()}
                if isinstance(o, list):
                    return [convert(v) for v in o]
                if hasattr(o, 'value'): # Enum
                    return o.value
                return o

            clean_data = convert(data)

            HARDWARE_CONFIG_FILE.write_text(json.dumps(clean_data, indent=4))
            logger.info("HardwareConfig saved.")
        except OSError as e:
            logger.error(f"Error saving HardwareConfig: {e}")
