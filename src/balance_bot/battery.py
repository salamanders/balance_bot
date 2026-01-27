class BatteryEstimator:
    EMA_ALPHA = 0.05
    FACTOR_SMOOTHING = 0.01
    MIN_COMPENSATION = 0.5
    MAX_COMPENSATION = 1.2

    def __init__(self, baseline_samples: int = 100, min_pwm: float = 20.0):
        """
        Estimates battery voltage drop by comparing expected vs actual responsiveness.

        :param baseline_samples: Number of samples to collect for establishing baseline.
        :param min_pwm: Minimum PWM value to consider for estimation (deadzone).
        """
        self.baseline_samples = baseline_samples
        self.min_pwm = min_pwm

        self.samples_collected = 0
        self.baseline_responsiveness = 0.0
        self.current_responsiveness = 0.0

        # Factor to scale motor output (1.0 = Full Battery, < 1.0 = Low Battery)
        # We actually use this to BOOST output: Final = Command / Factor
        self.compensation_factor = 1.0

    def update(self, pwm: float, angular_accel: float, dt: float) -> float:
        """
        Update the estimator with new data.

        :param pwm: The commanded PWM (before compensation)
        :param angular_accel: Measured angular acceleration (deg/s^2)
        :param dt: Time step
        :return: Current compensation factor
        """
        if abs(pwm) < self.min_pwm:
            return self.compensation_factor

        # Responsiveness = Acceleration / PWM
        raw_responsiveness = abs(angular_accel) / abs(pwm)

        # 1. ESTABLISH BASELINE
        if self.samples_collected < self.baseline_samples:
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
            self.EMA_ALPHA * raw_responsiveness
            + (1 - self.EMA_ALPHA) * self.current_responsiveness
        )

        # Calculate Ratio
        if self.baseline_responsiveness > 0:
            ratio = self.current_responsiveness / self.baseline_responsiveness
        else:
            ratio = 1.0

        # Update Factor (smoothly)
        # Factor should track the ratio.
        target_factor = max(self.MIN_COMPENSATION, min(self.MAX_COMPENSATION, ratio))

        # Apply slow smoothing to the factor itself to avoid feedback loops
        self.compensation_factor = (
            self.FACTOR_SMOOTHING * target_factor
            + (1 - self.FACTOR_SMOOTHING) * self.compensation_factor
        )

        return self.compensation_factor
