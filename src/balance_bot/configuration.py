"""
# System Context
This module is part of the `balance_bot` application, designed to control a self-balancing
homebrew robot. It relies on a deterministic, high-frequency control loop and pessimistic hardware interactions.

# Business Rules
- Fail-fast initialization: The system must crash loudly if physical hardware is missing or unresponsive during boot.
- Fault-tolerant control loop: Once Tier 1 is running (e.g., `BalanceCore`), transient I/O errors must not collapse the system; use continuous data quality metrics instead of fatal exceptions.
- Physical pessimism: Never hardcode physical constants; rely on zero-knowledge self-discovery to deduce configuration.

# Dependency Maps
- Relies on internal configuration (`HardwareConfig`, `LearningState`).
- Interfaces with Tier 1 (`BalanceCore`), Tier 3 (`Agent`), and physical hardware abstraction (`RobotHardware`).
"""

import logging
from pathlib import Path

from pydantic import BaseModel, ConfigDict, Field

from .enums import Axis

logger = logging.getLogger(__name__)

# Angle Thresholds (Degrees)
BALANCING_THRESHOLD = 15.0  # Normal operating range (+/-)
REST_ANGLE_MIN = 15.0  # Minimum angle to be considered "resting" on strut (covers ~20 deg)
REST_ANGLE_MAX = 55.0  # Maximum angle for resting (strut height)

# Startup / Recovery
STARTUP_RAMP_SPEED = 0.5  # Degrees per loop cycle to adjust setpoint during startup


class PIDParams(BaseModel):
    """
    PID Controller Parameters.
    """

    kp: float = 10.0
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
    countdown_blink_count_3: int = 3
    countdown_blink_count_2: int = 2
    countdown_blink_count_1: int = 1
    countdown_blink_on_time: float = 0.2
    countdown_blink_off_time: float = 0.2
    countdown_pause_time: float = 0.5


class ControlConfig(BaseModel):
    """
    General Control Logic Parameters.
    """

    yaw_correction_factor: float = 0.5
    upright_threshold: float = 5.0
    low_battery_log_threshold: float = 0.95
    kickup_power_forward: float = 0.0
    kickup_power_backward: float = 0.0
    max_tilt_angle: float = 10.0
    turn_gain: float = 30.0
    soft_recovery_kp_threshold: float = 1.0
    backlash_pulse_time: float = 0.0
    rock_amplitude_step: float = 2.0
    rock_pulse_max_duration: float = 0.40
    rock_max_pulses: int = 6
    crossover_zone_deg: float = 15.0
    min_carryover_rate: float = 40.0
    rest_settle_rate: float = 5.0


class SystemTiming(BaseModel):
    """
    System-wide Timing Constants.
    """

    setup_wait: float = 2.0
    calibration_pause: float = 1.0
    save_interval: float = 30.0
    battery_log_interval: float = 5.0
    tuning_log_interval: float = 1.0


class HardwareConfig(BaseModel):
    """
    Immutable Hardware Configuration.
    Defines fixed physical properties: pin assignments, bus IDs, axis mapping.
    Modifying this requires re-instantiating the object (e.g., via copy/replace).
    """

    model_config = ConfigDict(frozen=True)

    # I2C Bus Assignments
    motor_i2c_bus: int | None = None
    imu_i2c_bus: int | None = None

    # Motor Mapping
    motor_l: int | None = None
    motor_r: int | None = None
    motor_l_invert: bool = False
    motor_r_invert: bool = False

    # IMU Axis Mapping
    gyro_pitch_axis: Axis | None = None
    gyro_pitch_invert: bool = False
    gyro_yaw_axis: Axis | None = None
    gyro_yaw_invert: bool = False
    gyro_roll_axis: Axis | None = None
    gyro_roll_invert: bool = False
    accel_vertical_axis: Axis | None = None
    accel_vertical_invert: bool = False
    accel_forward_axis: Axis | None = None
    accel_forward_invert: bool = False

    # Loop Parameters (Fixed by Hardware Capability)
    loop_time: float = 0.01
    complementary_alpha: float = 0.98
    gyro_rate_distrust_limit: float = 60.0
    gyro_only_alpha: float = 0.999
    imu_max_retries: int = 5

    @classmethod
    def load(cls, file_path: Path = Path("hardware_config.json")) -> "HardwareConfig":
        """Load hardware config from disk."""
        if file_path.exists():
            content = file_path.read_text()
            if not content.strip():
                return cls()
            return cls.model_validate_json(content)
        return cls()

    def save(self, file_path: Path = Path("hardware_config.json")) -> None:
        """Serialize and save to disk."""
        file_path.write_text(self.model_dump_json(indent=4))
        logger.info("Hardware config saved.")


class LearningState(BaseModel):
    """
    Mutable Learned State.
    Defines tunable parameters and calibration data.
    """

    # Sub-Configs
    pid: PIDParams = Field(default_factory=PIDParams)
    battery: BatteryConfig = Field(default_factory=BatteryConfig)
    tuner: TunerConfig = Field(default_factory=TunerConfig)
    led: LedConfig = Field(default_factory=LedConfig)
    control: ControlConfig = Field(default_factory=ControlConfig)
    timing: SystemTiming = Field(default_factory=SystemTiming)

    # Operational Parameters
    crash_angle: float = 50.0
    vibration_threshold: int = 10
    min_power_visible: int = 0
    motor_trim: float = 0.0

    # Calibrated Rest Angles
    rest_angle_forward: float | None = None
    rest_angle_backward: float | None = None

    # Gyro Bias Calibration
    gyro_bias_x: float = 0.0
    gyro_bias_y: float = 0.0
    gyro_bias_z: float = 0.0

    # Discovery Flags
    i2c_buses_verified: bool = False
    hardware_init_verified: bool = False
    manual_lean_verified: bool = False
    broken_wire_verified: bool = False
    friction_threshold_verified: bool = False
    spatial_orientation_verified: bool = False
    motor_direction_verified: bool = False
    motor_phasing_verified: bool = False
    motor_channels_verified: bool = False
    motor_trim_verified: bool = False
    backlash_verified: bool = False
    kickup_dynamics_verified: bool = False
    balance_verified: bool = False

    @classmethod
    def load(cls, file_path: Path = Path("learning_state.json")) -> "LearningState":
        """Load learning state from disk."""
        if file_path.exists():
            content = file_path.read_text()
            if not content.strip():
                return cls()
            return cls.model_validate_json(content)
        return cls()

    def save(self, file_path: Path = Path("learning_state.json")) -> None:
        """Serialize and save to disk."""
        file_path.write_text(self.model_dump_json(indent=4))
        # logger.info("Learning state saved.") # Reduce log spam on frequent saves

    def reset(self) -> None:
        """Reset learned state to defaults."""
        # Create a new default instance and copy values?
        # Since we are modifying 'self', we need to reset fields.
        # Easier to just replace the object in the caller, but here we can reset fields.
        defaults = self.__class__()
        for field in self.__class__.model_fields:
            setattr(self, field, getattr(defaults, field))
