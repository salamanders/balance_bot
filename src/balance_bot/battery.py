from .config import BatteryConfig
from .utils import clamp


class BatteryEstimator:
    """
    Estimates relative battery voltage by monitoring the robot's physical responsiveness.

    Theory of Operation:
    --------------------
    DC Motor torque is proportional to current, which is proportional to voltage * duty_cycle.
    Angular acceleration is proportional to torque (Newton's 2nd Law: Tau = I * alpha).
    Therefore: Acceleration ~ Voltage * Duty_Cycle

    If the battery voltage drops, the same Duty_Cycle (PWM) produces less Acceleration.
    We define 'Responsiveness' as:
        Responsiveness = |Angular_Acceleration| / |PWM_Command|

    1. We measure a baseline Responsiveness when the robot first starts (assuming fresh battery).
    2. We continuously measure current Responsiveness.
    3. The ratio (Current / Baseline) gives us a 'Compensation Factor'.
       - Factor = 1.0: Battery is at the same level as startup.
       - Factor < 1.0: Battery is weaker.
    4. We use this factor to BOOST the motor command:
       Final_Command = Desired_Command / Compensation_Factor

    Example: If battery is half dead, Factor might be 0.8. We divide by 0.8 (multiply by 1.25)
    to command the motors 25% harder to get the expected result.
    """

    def __init__(self, config: BatteryConfig = BatteryConfig()):
        """
        Initialize the Battery Estimator.
        :param config: Configuration object containing smoothing factors and limits.
        """
        self.config = config

        self.samples_collected = 0
        self.baseline_responsiveness = 0.0
        self.current_responsiveness = 0.0

        # Factor to scale motor output (1.0 = Baseline, < 1.0 = Weaker Battery)
        # This value is slowly updated to prevent sudden jumps.
        self.compensation_factor = 1.0

    def update(self, pwm: float, angular_accel: float, loop_delta_time: float) -> float:
        """
        Update the estimator with new sensor data.

        :param pwm: The commanded PWM sent to motors (before compensation). Unit: 0-100.
        :param angular_accel: Measured angular acceleration. Unit: degrees/second^2.
        :param loop_delta_time: Time step in seconds.
        :return: Current compensation factor (Scalar, typically 0.5 to 1.2).
        """
        # Ignore small inputs where noise dominates signal
        if abs(pwm) < self.config.min_pwm:
            return self.compensation_factor

        # Calculate instantaneous responsiveness
        # Unit: (deg/s^2) / PWM_Unit
        raw_responsiveness = abs(angular_accel) / abs(pwm)

        # 1. ESTABLISH BASELINE (First N samples)
        if self.samples_collected < self.config.baseline_samples:
            # Accumulate running average for baseline
            self.baseline_responsiveness = (
                (self.baseline_responsiveness * self.samples_collected)
                + raw_responsiveness
            ) / (self.samples_collected + 1)
            self.samples_collected += 1
            self.current_responsiveness = self.baseline_responsiveness
            return 1.0

        # 2. RUNNING ESTIMATION
        # Smooth the current reading using Exponential Moving Average (EMA)
        self.current_responsiveness = (
            self.config.ema_alpha * raw_responsiveness
            + (1 - self.config.ema_alpha) * self.current_responsiveness
        )

        # Calculate Ratio: Current Performance / Baseline Performance
        if self.baseline_responsiveness > 0:
            ratio = self.current_responsiveness / self.baseline_responsiveness
        else:
            ratio = 1.0

        # 3. CALCULATE COMPENSATION FACTOR
        # Clamp the ratio to safe limits defined in config
        target_factor = clamp(
            ratio, self.config.min_compensation, self.config.max_compensation
        )

        # Apply very slow low-pass filter to the factor itself.
        # This prevents the robot from oscillating if the estimate is noisy.
        self.compensation_factor = (
            self.config.factor_smoothing * target_factor
            + (1 - self.config.factor_smoothing) * self.compensation_factor
        )

        return self.compensation_factor
