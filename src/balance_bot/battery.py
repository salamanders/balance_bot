class BatteryEstimator:
    def __init__(self, baseline_samples=100, min_pwm=20.0):
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

        # Exponential Moving Average alpha for smoothness
        self.ema_alpha = 0.05

    def update(self, pwm, angular_accel, dt):
        """
        Update the estimator with new data.

        :param pwm: The commanded PWM (before compensation) or effective PWM?
                    It should be the PWM sent to the motors.
                    Note: If we are compensating, we are sending HIGHER PWM.
                    We should use the PWM *value* that generated the torque.
        :param angular_accel: Measured angular acceleration (deg/s^2)
        :param dt: Time step
        :return: Current compensation factor
        """
        if abs(pwm) < self.min_pwm:
            return self.compensation_factor

        # Responsiveness = Acceleration / PWM
        # If PWM is positive (forward), Accel should be positive (pitch up? check physics later).
        # We just care about magnitude correlation.

        # Note: If pwm and accel have opposite signs (braking), the physics is different.
        # Ideally only measure when driving *into* the lean (accelerating the body).
        # For simplicity, use absolute values, assuming correlation.
        raw_responsiveness = abs(angular_accel) / abs(pwm)

        # 1. ESTABLISH BASELINE
        if self.samples_collected < self.baseline_samples:
            # Accumulate average
            self.baseline_responsiveness = (
                (self.baseline_responsiveness * self.samples_collected) + raw_responsiveness
            ) / (self.samples_collected + 1)
            self.samples_collected += 1
            self.current_responsiveness = self.baseline_responsiveness
            return 1.0

        # 2. RUNNING ESTIMATION
        # Smooth the current reading
        self.current_responsiveness = (
            self.ema_alpha * raw_responsiveness
            + (1 - self.ema_alpha) * self.current_responsiveness
        )

        # Calculate Ratio
        if self.baseline_responsiveness > 0:
            ratio = self.current_responsiveness / self.baseline_responsiveness
        else:
            ratio = 1.0

        # Update Factor (smoothly)
        # Factor should track the ratio.
        # If ratio is 0.8 (80% power), factor should be 0.8.
        # We clamp it to reasonable bounds (e.g. don't compensate if battery is DEAD dead)
        target_factor = max(0.5, min(1.2, ratio))

        # Apply slow smoothing to the factor itself to avoid feedback loops
        self.compensation_factor = (
            0.01 * target_factor + 0.99 * self.compensation_factor
        )

        return self.compensation_factor
