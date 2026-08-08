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

from ..configuration import BatteryConfig
from ..utils import clamp


class BatteryEstimator:
    """
    Estimates battery health based on motor responsiveness.

    Concept:
        As battery voltage drops, motors produce less torque for the same PWM command.
        Responsiveness = AngularAcceleration / PWM.

        We compare current responsiveness against a baseline (established at startup).
        If responsiveness drops, we increase the 'compensation factor' to boost PWM.
    """

    def __init__(self, config: BatteryConfig = BatteryConfig()):
        """
        Initialize the estimator.
        :param config: Tuning limits and smoothing factors.
        """
        self.config = config

        self.samples_collected = 0
        self.baseline_responsiveness = 0.0
        self.current_responsiveness = 0.0

        # Factor to scale motor output (1.0 = Full Battery, < 1.0 = Low Battery)
        # Usage: FinalOutput = PIDOutput / CompensationFactor
        # Example: If factor is 0.8 (20% drop), we divide by 0.8 (multiply by 1.25).
        self.compensation_factor = 1.0

    def update(self, pwm: float, angular_accel: float) -> float:
        """
        Update the estimator with new physical data.

        :param pwm: The commanded PWM sent to motors (before compensation).
        :param angular_accel: Measured angular acceleration (deg/s^2).
        :return: Current compensation factor (0.0 to 1.0ish).
        """
        if abs(pwm) < self.config.min_pwm:
            return self.compensation_factor

        # Calculate raw responsiveness sample
        raw_responsiveness = abs(angular_accel) / abs(pwm)

        # 1. ESTABLISH BASELINE (First N samples)
        if self.samples_collected < self.config.baseline_samples:
            # Accumulate average
            self.baseline_responsiveness = (
                (self.baseline_responsiveness * self.samples_collected) + raw_responsiveness
            ) / (self.samples_collected + 1)
            self.samples_collected += 1
            self.current_responsiveness = self.baseline_responsiveness
            return 1.0

        # 2. RUNNING ESTIMATION
        # Smooth the current reading (EMA)
        self.current_responsiveness = (
            self.config.ema_alpha * raw_responsiveness
            + (1 - self.config.ema_alpha) * self.current_responsiveness
        )

        # Calculate Ratio (Current / Baseline)
        if self.baseline_responsiveness > 1e-6:
            ratio = self.current_responsiveness / self.baseline_responsiveness
        else:
            ratio = 1.0

        # 3. CALCULATE COMPENSATION FACTOR
        # If ratio < 1.0, battery is weak.
        target_factor = clamp(ratio, self.config.min_compensation, self.config.max_compensation)

        # Apply slow smoothing to the factor itself to avoid feedback loops
        self.compensation_factor = (
            self.config.factor_smoothing * target_factor
            + (1 - self.config.factor_smoothing) * self.compensation_factor
        )

        return self.compensation_factor
