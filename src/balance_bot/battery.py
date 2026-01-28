from .config import BatteryConfig


class BatteryEstimator:
    def __init__(self, config: BatteryConfig = BatteryConfig()):
        """
        Estimates battery voltage drop by comparing expected vs actual responsiveness.
        :param config: Configuration object for battery estimation.
        """
        self.config = config

        self.samples_collected = 0
        self.baseline_responsiveness = 0.0
        self.current_responsiveness = 0.0

        # Factor to scale motor output (1.0 = Full Battery, < 1.0 = Low Battery)
        # We actually use this to BOOST output: Final = Command / Factor
        self.compensation_factor = 1.0

    def update(self, pwm: float, angular_accel: float, loop_delta_time: float) -> float:
        """
        Update the estimator with new data.

        :param pwm: The commanded PWM (before compensation)
        :param angular_accel: Measured angular acceleration (deg/s^2)
        :param loop_delta_time: Time step
        :return: Current compensation factor
        """
        if abs(pwm) < self.config.min_pwm:
            return self.compensation_factor

        # Responsiveness = Acceleration / PWM
        raw_responsiveness = abs(angular_accel) / abs(pwm)

        # 1. ESTABLISH BASELINE
        if self.samples_collected < self.config.baseline_samples:
            # Accumulate average
            self.baseline_responsiveness = (
                (self.baseline_responsiveness * self.samples_collected)
                + raw_responsiveness
            ) / (self.samples_collected + 1)
            self.samples_collected += 1
            self.current_responsiveness = self.baseline_responsiveness
            return 1.0

        # 2. RUNNING ESTIMATION
        # Smooth the current reading
        self.current_responsiveness = (
            self.config.ema_alpha * raw_responsiveness
            + (1 - self.config.ema_alpha) * self.current_responsiveness
        )

        # Calculate Ratio
        if self.baseline_responsiveness > 0:
            ratio = self.current_responsiveness / self.baseline_responsiveness
        else:
            ratio = 1.0

        # Update Factor (smoothly)
        # Factor should track the ratio.
        target_factor = max(
            self.config.min_compensation,
            min(self.config.max_compensation, ratio),
        )

        # Apply slow smoothing to the factor itself to avoid feedback loops
        self.compensation_factor = (
            self.config.factor_smoothing * target_factor
            + (1 - self.config.factor_smoothing) * self.compensation_factor
        )

        return self.compensation_factor
