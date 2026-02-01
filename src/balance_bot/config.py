import json
import logging
from contextlib import contextmanager
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)
CONFIG_FILE = Path("pid_config.json")

# Angle Thresholds (Degrees)
BALANCING_THRESHOLD = 20.0      # Normal operating range (+/-)
REST_ANGLE_MIN = 30.0           # Minimum angle to be considered "resting" on strut
REST_ANGLE_MAX = 50.0           # Maximum angle for resting (strut height)
CRASH_ANGLE = 60.0              # If we pass this, something is wrong. HARD STOP.

# Startup / Recovery
STARTUP_RAMP_SPEED = 0.5        # Degrees per loop cycle to adjust setpoint during startup


@dataclass
class PIDParams:
    """
    PID Controller Parameters.

    :param kp: Proportional Gain.
        * UOM: Dimensionless (Motor Output per Degree Error)
        * Purpose: Main driving force to correct error.
        * Limits: 0.0 to 100.0 (Practical).
        * Impact: Higher = stiffer, more responsive; Lower = softer, sluggish.
    :param ki: Integral Gain.
        * UOM: Dimensionless (Motor Output per Degree*Second Error)
        * Purpose: Corrects steady-state error (lean).
        * Limits: 0.0 to 50.0 (Practical).
        * Impact: Higher = removes lean faster but adds instability/oscillation.
    :param kd: Derivative Gain.
        * UOM: Dimensionless (Motor Output per Degree/Second Error)
        * Purpose: Dampens oscillation and anticipates future error.
        * Limits: 0.0 to 5.0 (Practical).
        * Impact: Higher = less overshoot/vibration; Too High = high frequency noise/jitter.
    :param target_angle: Target Pitch Angle (Setpoint).
        * UOM: Degrees
        * Purpose: The angle at which the robot is mechanically balanced.
        * Limits: -10.0 to 10.0 (Practical).
        * Impact: Offset determines neutral balance point.
    :param integral_limit: Anti-Windup Limit.
        * UOM: Motor Output Units (implied)
        * Purpose: Caps the maximum contribution of the I-term.
        * Limits: 0.0 to 100.0.
        * Impact: Prevents runaway integral term during long errors (e.g., holding robot).
    """
    kp: float = 25.0
    ki: float = 0.0
    kd: float = 0.5
    target_angle: float = 0.0
    integral_limit: float = 20.0


@dataclass(frozen=True)
class BatteryConfig:
    """
    Configuration for Battery Voltage Estimation and Compensation.

    :param ema_alpha: Exponential Moving Average Alpha for responsiveness.
        * UOM: Factor (0.0 to 1.0)
        * Purpose: Smoothing factor for raw responsiveness measurements.
        * Limits: 0.01 to 0.2.
        * Impact: Higher = faster reaction to voltage drop but noisier.
    :param factor_smoothing: Smoothing factor for the final compensation multiplier.
        * UOM: Factor (0.0 to 1.0)
        * Purpose: Prevents rapid jumps in motor power scaling.
        * Limits: 0.001 to 0.1.
        * Impact: Higher = faster adaptation; Lower = stable power scaling.
    :param min_compensation: Minimum allowed compensation factor.
        * UOM: Factor
        * Purpose: Safety floor for voltage drop estimation.
        * Limits: 0.1 to 1.0.
        * Impact: Prevents over-boosting motors if estimation fails.
    :param max_compensation: Maximum allowed compensation factor.
        * UOM: Factor
        * Purpose: Safety ceiling for voltage drop estimation.
        * Limits: 1.0 to 2.0.
        * Impact: Limits how much extra power can be requested.
    :param min_pwm: Minimum PWM threshold for estimation.
        * UOM: Motor Output Units (0-100)
        * Purpose: Ignore low-speed data where friction dominates.
        * Limits: 10.0 to 50.0.
        * Impact: Determines when data is valid for battery estimation.
    :param baseline_samples: Number of samples to establish baseline battery health.
        * UOM: Count
        * Purpose: Initial calibration period size.
        * Limits: 50 to 500.
        * Impact: Higher = more accurate baseline; Lower = faster startup.
    """
    ema_alpha: float = 0.05
    factor_smoothing: float = 0.01
    min_compensation: float = 0.5
    max_compensation: float = 1.2
    min_pwm: float = 20.0
    baseline_samples: int = 100


@dataclass(frozen=True)
class TunerConfig:
    """
    Configuration for Continuous Auto-Tuner.

    :param cooldown_reset: Cooldown ticks after a tuning event.
        * UOM: Ticks (Loop Cycles)
        * Purpose: Prevent back-to-back adjustments.
        * Limits: 10 to 200.
        * Impact: Higher = slower tuning process; Lower = risk of instability.
    :param oscillation_threshold: Threshold ratio for zero-crossings.
        * UOM: Ratio (0.0 to 1.0)
        * Purpose: Trigger tuning if oscillations exceed this frequency.
        * Limits: 0.1 to 0.5.
        * Impact: Sensitivity to oscillation detection.
    :param kp_oscillation_penalty: Kp adjustment when oscillating.
        * UOM: PID Value Adder
        * Purpose: Reduce Kp to stop oscillation.
        * Limits: -1.0 to -0.01.
        * Impact: Magnitude of reduction per event.
    :param kd_oscillation_boost: Kd adjustment when oscillating.
        * UOM: PID Value Adder
        * Purpose: Increase Damping to stop oscillation.
        * Limits: 0.01 to 0.5.
        * Impact: Magnitude of boost per event.
    :param stability_std_dev: Max standard deviation for stability check.
        * UOM: Degrees
        * Purpose: Define "stable" operation.
        * Limits: 0.1 to 5.0.
        * Impact: How still the robot must be to increase gains.
    :param stability_mean_err: Max mean error for stability check.
        * UOM: Degrees
        * Purpose: Define "upright" operation.
        * Limits: 0.1 to 5.0.
        * Impact: How close to vertical required to increase gains.
    :param kp_stability_boost: Kp adjustment when stable.
        * UOM: PID Value Adder
        * Purpose: Gently increase stiffness when safe.
        * Limits: 0.01 to 0.1.
        * Impact: Rate of gain increase.
    :param steady_error_threshold: Error threshold for I-term boost.
        * UOM: Degrees
        * Purpose: Detect persistent lean.
        * Limits: 1.0 to 10.0.
        * Impact: Sensitivity to leaning.
    :param ki_boost: Ki adjustment for steady error.
        * UOM: PID Value Adder
        * Purpose: Increase I-term to correct lean.
        * Limits: 0.001 to 0.1.
        * Impact: Rate of I-term increase.
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

    # Tuning Aggression Decay
    start_aggression_first_run: float = 5.0
    start_aggression_normal: float = 1.0
    aggression_decay: float = 0.9995
    min_aggression: float = 0.1

    # Balance Point Finder
    balance_check_interval: int = 500           # Check every 5 seconds @ 100Hz
    balance_learning_rate: float = 0.05         # Adjustment per check (degrees)
    balance_max_deviation: float = 10.0         # Max deviation from 0 (degrees)
    balance_motor_threshold: float = 5.0        # Min average motor output to trigger
    balance_pitch_rate_threshold: float = 10.0  # Max pitch rate for stability (deg/s)


@dataclass(frozen=True)
class LedConfig:
    """
    Configuration for LED timings and patterns.

    :param setup_blink_interval: Blink speed during setup.
        * UOM: Seconds
    :param tuning_blink_interval: Blink speed during tuning.
        * UOM: Seconds
    :param countdown_blink_count_3: Flashes for count '3'.
    :param countdown_blink_count_2: Flashes for count '2'.
    :param countdown_blink_count_1: Flashes for count '1'.
    :param countdown_blink_on_time: Duration LED is ON during countdown flash.
        * UOM: Seconds
    :param countdown_blink_off_time: Duration LED is OFF during countdown flash.
        * UOM: Seconds
    :param countdown_pause_time: Pause between countdown numbers.
        * UOM: Seconds
    """
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
    """
    General Control Logic Parameters.

    :param yaw_correction_factor: Turning sensitivity.
        * UOM: Motor Units per Gyro Rate Unit
        * Purpose: scales yaw rate to differential motor output.
        * Limits: 0.0 to 2.0.
        * Impact: Higher = faster spins; Lower = slower turns.
    :param upright_threshold: Angle to consider "recovered" and upright.
        * UOM: Degrees
        * Purpose: Margin for exiting recovery mode.
        * Limits: 1.0 to 10.0.
        * Impact: Wider = easier to resume; Narrower = stricter safety.
    :param low_battery_log_threshold: Threshold to log battery warnings.
        * UOM: Factor (Compensation Multiplier)
        * Purpose: Warn user when voltage drop is significant.
    """
    yaw_correction_factor: float = 0.5
    upright_threshold: float = 5.0
    low_battery_log_threshold: float = 0.95


@dataclass(frozen=True)
class SystemTiming:
    """
    System-wide Timing Constants.

    :param setup_wait: Duration to wait for IMU convergence on boot.
        * UOM: Seconds
    :param calibration_pause: Duration to wait after calibration.
        * UOM: Seconds
    :param save_interval: Interval between auto-saves of config.
        * UOM: Seconds
    :param battery_log_interval: Min interval between battery log messages.
        * UOM: Seconds
    """
    setup_wait: float = 2.0
    calibration_pause: float = 1.0
    save_interval: float = 30.0
    battery_log_interval: float = 5.0


SYSTEM_TIMING = SystemTiming()


@contextmanager
def temp_pid_overrides(pid_params: PIDParams, **overrides):
    """
    Context manager to temporarily override PID parameters.
    Restores original values on exit.

    :param pid_params: The PIDParams instance to modify.
    :param overrides: Keyword arguments for fields to override (e.g. kp=0.0).
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
    Master Configuration Object.
    Aggregates all sub-configs and hardware mapping.

    :param pid: Active PID Parameters.
    :param battery: Battery estimation config.
    :param tuner: Continuous tuner config.
    :param led: LED pattern config.
    :param control: General control params.
    :param motor_l: Channel index for Left Motor.
    :param motor_r: Channel index for Right Motor.
    :param motor_l_invert: Boolean to invert Left Motor direction.
    :param motor_r_invert: Boolean to invert Right Motor direction.
    :param gyro_pitch_axis: Axis ('x','y','z') used for pitch rate.
    :param gyro_pitch_invert: Boolean to invert gyro polarity.
    :param accel_vertical_axis: Axis ('x','y','z') aligned with gravity.
    :param accel_vertical_invert: Boolean to invert vertical axis.
    :param accel_forward_axis: Axis ('x','y','z') aligned with forward motion.
    :param accel_forward_invert: Boolean to invert forward axis.
    :param i2c_bus: I2C Bus ID (1 for HAT, 0/3/etc for others).
    :param loop_time: Target loop duration.
        * UOM: Seconds
        * Default: 0.01 (100Hz)
    :param fall_angle_limit: Angle at which robot gives up balancing.
        * UOM: Degrees
        * Impact: Safety cutoff.
    :param complementary_alpha: Filter coefficient.
        * UOM: Ratio (0.0 to 1.0)
        * Impact: Higher = Trust Gyro (Smooth); Lower = Trust Accel (Fast).
    :param vibration_threshold: Count of oscillations to trigger tuning finish.
    """
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
    accel_vertical_axis: str = "z"
    accel_vertical_invert: bool = False
    accel_forward_axis: str = "y"
    accel_forward_invert: bool = False
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
        """
        Load configuration from disk.
        Handles migration from legacy flat formats to nested dataclasses.
        """
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

    def save(self) -> None:
        """Serialize and save the current configuration to disk."""
        try:
            CONFIG_FILE.write_text(json.dumps(asdict(self), indent=4))
            logger.info("Config saved.")
        except OSError as e:
            logger.error(f"Error saving config: {e}")
