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
    Configuration for the Proportional-Integral-Derivative (PID) controller.

    This class holds the tuning parameters used to maintain the robot's balance.
    """

    kp: float = 5.0
    """
    Proportional Gain.
    UOM: Dimensionless (Motor Output / Error).
    Purpose: Determines the reaction magnitude proportional to the current error.
    Impact: Higher values increase responsiveness but can cause high-frequency oscillations. Lower values feel sluggish.
    """

    ki: float = 0.0
    """
    Integral Gain.
    UOM: Dimensionless (Motor Output / (Error * Time)).
    Purpose: Corrects steady-state error by accumulating past errors over time.
    Impact: Higher values correct drift faster but can cause instability (windup) and low-frequency oscillations.
    """

    kd: float = 0.0
    """
    Derivative Gain.
    UOM: Dimensionless (Motor Output / (Error / Time)).
    Purpose: Dampens the system by reacting to the rate of change of the error.
    Impact: Higher values reduce overshoot and dampen oscillations but can amplify noise.
    """

    target_angle: float = 0.0
    """
    The target pitch angle for balancing.
    UOM: Degrees.
    Purpose: Defines the 'zero' point where the robot is considered upright.
    Impact: Adjust this to correct for center-of-mass offsets.
    """

    integral_limit: float = 20.0
    """
    Limit for the integral term (Anti-Windup).
    UOM: Motor Output Units (arbitrary scale, usually -100 to 100).
    Purpose: Prevents the integral term from growing indefinitely during sustained errors (e.g., holding the robot down).
    Impact: Lower values limit the maximum correction from the I-term. Higher values allow more accumulation but risk overshoot on recovery.
    """


@dataclass(frozen=True)
class BatteryConfig:
    """
    Configuration for the Battery Estimator.

    These parameters control how the robot guesses its battery level based on performance
    rather than a voltage sensor.
    """

    ema_alpha: float = 0.05
    """
    Exponential Moving Average (EMA) alpha for responsiveness smoothing.
    UOM: Factor (0.0 to 1.0).
    Purpose: Controls how much weight is given to the most recent responsiveness sample versus the history.
    Impact: Higher values (closer to 1.0) make the estimator react faster to changes but are noisier. Lower values are smoother but laggy.
    """

    factor_smoothing: float = 0.01
    """
    Smoothing factor applied to the final compensation factor.
    UOM: Factor (0.0 to 1.0).
    Purpose: Applies a very strong low-pass filter to the calculated compensation factor to prevent sudden jumps in motor output.
    Impact: Lower values mean the robot slowly adapts to battery drain. Higher values allow faster adaptation but can cause feedback loops if the motor output changes too quickly in response to its own compensation.
    """

    min_compensation: float = 0.5
    """
    Minimum allowed compensation factor.
    UOM: Multiplier (Ratio).
    Purpose: Hard floor to prevent the estimator from thinking the battery is dead or the motors are broken.
    Impact: Prevents division by zero or extreme boosting of motor signals.
    """

    max_compensation: float = 1.2
    """
    Maximum allowed compensation factor.
    UOM: Multiplier (Ratio).
    Purpose: Cap on how much the system can boost the motor output.
    Impact: Limits the risk of over-driving components if the estimator goes haywire.
    """

    min_pwm: float = 20.0
    """
    Minimum PWM command required to update the estimator.
    UOM: Motor Output Units (0-100).
    Purpose: We only estimate battery health when we are actually trying to move the motors significantly.
    Impact: Prevents noise at low speeds/idle from skewing the battery estimate.
    """

    baseline_samples: int = 100
    """
    Number of samples to collect at startup to establish the 'full battery' baseline.
    UOM: Count (number of loop iterations).
    Purpose: Determines the initial responsiveness of the robot when the battery is known to be fresh (or at least the starting state).
    Impact: Higher values give a more accurate baseline but delay the start of active estimation.
    """


@dataclass(frozen=True)
class TunerConfig:
    """
    Configuration for the Continuous Auto-Tuner.

    These heuristics determine how the robot self-adjusts its PID gains during operation.
    """

    cooldown_reset: int = 50
    """
    Cooldown period after a tuning adjustment is made.
    UOM: Loop iterations (counts).
    Purpose: Prevents the tuner from making rapid-fire changes before the system has settled.
    Impact: Higher values make the tuning slower and more stable.
    """

    oscillation_threshold: float = 0.15
    """
    Threshold for detecting oscillation (high frequency zero-crossing).
    UOM: Ratio (Zero Crossings / Total Samples).
    Purpose: If the error crosses zero more frequently than this ratio, we assume the system is oscillating (P too high).
    Impact: Lower values make the system more sensitive to vibration and more aggressive at reducing Kp.
    """

    kp_oscillation_penalty: float = -0.1
    """
    Adjustment to Kp when oscillation is detected.
    UOM: PID Gain Units (additive).
    Purpose: Reduces Kp to stop oscillation.
    Impact: A larger negative number reduces gain faster.
    """

    kd_oscillation_boost: float = 0.05
    """
    Adjustment to Kd when oscillation is detected.
    UOM: PID Gain Units (additive).
    Purpose: Increases damping (Kd) to help stop oscillation.
    Impact: Adds braking force.
    """

    stability_std_dev: float = 1.0
    """
    Standard Deviation threshold for 'Stable' state.
    UOM: Degrees.
    Purpose: If the error variance is below this, the robot is balancing well.
    Impact: Defines how 'still' the robot must be before we try to tighten the tuning.
    """

    stability_mean_err: float = 1.0
    """
    Mean Error threshold for 'Stable' state.
    UOM: Degrees.
    Purpose: Ensures the robot is not only still but also upright (not leaning) before boosting gains.
    Impact: Stricter requirement for stability.
    """

    kp_stability_boost: float = 0.02
    """
    Adjustment to Kp when stability is detected.
    UOM: PID Gain Units (additive).
    Purpose: Slowly increases Kp to make the robot 'stiffer' and more responsive if it is currently handling well.
    Impact: Helps find the highest stable gain over time.
    """

    steady_error_threshold: float = 3.0
    """
    Threshold for detecting steady-state error (lean).
    UOM: Degrees.
    Purpose: If the average error is consistently above this, we need more Integral term.
    Impact: Triggers Ki boost.
    """

    ki_boost: float = 0.005
    """
    Adjustment to Ki when steady error is detected.
    UOM: PID Gain Units (additive).
    Purpose: Increases Ki to correct for lean/drift.
    Impact: Higher values correct lean faster but risk windup.
    """


@dataclass(frozen=True)
class LedConfig:
    """
    Configuration for LED Blink Patterns.
    """

    setup_blink_interval: float = 0.05
    """
    Blink interval during setup phase.
    UOM: Seconds.
    Purpose: Fast blinking to indicate initialization.
    """

    tuning_blink_interval: float = 0.25
    """
    Blink interval during auto-tuning phase.
    UOM: Seconds.
    Purpose: Medium blinking to indicate the robot is in 'Tune' mode.
    """

    countdown_blink_count_3: int = 3
    """Number of blinks for the '3' count."""

    countdown_blink_count_2: int = 2
    """Number of blinks for the '2' count."""

    countdown_blink_count_1: int = 1
    """Number of blinks for the '1' count."""

    countdown_blink_on_time: float = 0.2
    """
    Duration LED is ON during countdown blinks.
    UOM: Seconds.
    """

    countdown_blink_off_time: float = 0.2
    """
    Duration LED is OFF during countdown blinks.
    UOM: Seconds.
    """

    countdown_pause_time: float = 0.5
    """
    Pause duration between countdown numbers.
    UOM: Seconds.
    """


@dataclass(frozen=True)
class ControlConfig:
    """
    General Control Loop Parameters.
    """

    yaw_correction_factor: float = 0.5
    """
    Gain for Yaw correction (Steering/Heading hold).
    UOM: Multiplier.
    Purpose: Multiplied by the Gyro Yaw Rate to counteract rotation.
    Impact: Higher values resist turning more strongly.
    """

    upright_threshold: float = 5.0
    """
    Angle threshold to consider the robot 'upright' during recovery.
    UOM: Degrees.
    Purpose: The robot must be within this angle of vertical to start the balancing sequence.
    Impact: Smaller values require more precision from the user when placing the robot.
    """

    low_battery_log_threshold: float = 0.95
    """
    Threshold for logging low battery warnings.
    UOM: Multiplier (Compensation Factor).
    Purpose: If the battery compensation factor drops below this (e.g., 0.95 means 5% boost needed), we log a warning.
    Impact: Controls verbosity regarding battery health.
    """


@dataclass(frozen=True)
class SystemTiming:
    """
    System-wide timing constants.
    """

    setup_wait: float = 2.0
    """
    Time to wait during setup for sensors to settle.
    UOM: Seconds.
    """

    calibration_pause: float = 1.0
    """
    Time to pause after calibration before proceeding.
    UOM: Seconds.
    """

    save_interval: float = 30.0
    """
    Interval between automatic config saves (if config is dirty).
    UOM: Seconds.
    Purpose: Prevents writing to disk too frequently (SD card wear leveling).
    """

    battery_log_interval: float = 5.0
    """
    Min interval between battery log messages.
    UOM: Seconds.
    Purpose: Prevents log spamming.
    """


@dataclass(frozen=True)
class TuningHeuristics:
    """
    Constants for the 'Startup' Auto-Tuning (Ziegler-Nichols approach).
    """

    kp_increment: float = 0.05
    """
    Amount to increment Kp per loop iteration while searching for oscillation.
    UOM: PID Gain Units.
    """

    kp_reduction: float = 0.6
    """
    Factor to reduce Kp by after oscillation is found (Ziegler-Nichols rule).
    UOM: Multiplier (0.0 to 1.0).
    Purpose: We find the 'Ultimate Gain' (Ku) then set Kp = 0.6 * Ku.
    """

    kd_ratio: float = 0.05
    """
    Ratio to set Kd relative to the tuned Kp.
    UOM: Multiplier.
    Purpose: Kd = Kp * kd_ratio.
    """

    ki_ratio: float = 0.005
    """
    Ratio to set Ki relative to the tuned Kp.
    UOM: Multiplier.
    Purpose: Ki = Kp * ki_ratio.
    """


SYSTEM_TIMING = SystemTiming()
TUNING_HEURISTICS = TuningHeuristics()


@contextmanager
def temp_pid_overrides(pid_params: PIDParams, **overrides):
    """
    Context manager to temporarily override PID parameters.
    Restores original values on exit.

    :param pid_params: The PIDParams instance to modify.
    :param overrides: Key-value pairs of attributes to override (e.g., ki=0.0).
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
    Master Configuration Class for the Balance Bot.

    Aggregates all sub-configurations and holds hardware-specific settings.
    """

    pid: PIDParams
    """Active PID Parameters."""

    battery: BatteryConfig = BatteryConfig()
    """Battery Estimation Configuration."""

    tuner: TunerConfig = TunerConfig()
    """Continuous Tuner Configuration."""

    led: LedConfig = LedConfig()
    """LED Pattern Configuration."""

    control: ControlConfig = ControlConfig()
    """Control Loop Configuration."""

    motor_l: int = 0
    """Left Motor Channel Index (0 or 1)."""

    motor_r: int = 1
    """Right Motor Channel Index (0 or 1)."""

    motor_l_invert: bool = False
    """True to invert the left motor direction."""

    motor_r_invert: bool = False
    """True to invert the right motor direction."""

    gyro_pitch_axis: str = "x"
    """Axis of the Gyro to use for pitch measurement ('x' or 'y')."""

    gyro_pitch_invert: bool = False
    """True to invert the pitch angle measurement."""

    loop_time: float = 0.01
    """
    Target time for a single control loop iteration.
    UOM: Seconds.
    Default: 0.01s (10ms) -> 100Hz.
    """

    # Operational Parameters
    fall_angle_limit: float = 45.0
    """
    Angle at which the robot assumes it has fallen and kills motors.
    UOM: Degrees.
    """

    complementary_alpha: float = 0.98
    """
    Alpha value for the Complementary Filter.
    UOM: Factor (0.0 to 1.0).
    Purpose: Determines the blend between Gyro (fast, drifting) and Accelerometer (slow, noisy).
    Impact: Higher values (e.g. 0.98) trust the Gyro more. Lower values trust the Accelerometer more.
    """

    vibration_threshold: int = 10
    """
    Count of zero-crossings to detect oscillation during the startup tuning phase.
    UOM: Count.
    """

    @staticmethod
    def _filter_keys(dataclass_type, data: dict[str, Any]) -> dict[str, Any]:
        """Helper to filter dictionary keys based on dataclass annotations."""
        return {k: v for k, v in data.items() if k in dataclass_type.__annotations__}

    @classmethod
    def load(cls) -> "RobotConfig":
        """
        Load configuration from the default JSON file.
        Falls back to defaults if the file is missing or invalid.
        Handles migration from legacy flat formats.
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
        """Save the current configuration to disk as JSON."""
        try:
            CONFIG_FILE.write_text(json.dumps(asdict(self), indent=4))
            logger.info("Config saved.")
        except OSError as e:
            logger.error(f"Error saving config: {e}")
