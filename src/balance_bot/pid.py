from .config import PIDParams
from .utils import clamp


class PIDController:
    """
    Standard PID Controller implementation.
    Features:
    - Anti-windup clamping for the Integral term.
    - Support for 'Derivative on Measurement' to reduce noise and setpoint kick.
    """

    def __init__(self, params: PIDParams):
        """
        Initialize the PID Controller.
        :param params: PIDParams dataclass containing Kp, Ki, Kd, etc.
        """
        self.params = params
        self.integral = 0.0
        self.last_error = 0.0

    def update(
        self,
        error: float,
        loop_delta_time: float,
        measurement_rate: float | None = None,
    ) -> float:
        """
        Calculate the Control Output based on error and time.

        Formula:
            Output = (Kp * Error) + (Ki * Integral) + (Kd * Derivative)

        :param error: The current error (Target - Measured).
                      Unit: Degrees.
        :param loop_delta_time: Time elapsed since the last update.
                                Unit: Seconds.
        :param measurement_rate: Optional. The rate of change of the measured variable (e.g., Gyro Rate).
                                 Unit: Degrees/Second.
                                 If provided, we use this for the D-term instead of calculating (Error - LastError).
                                 Why?
                                 1. Less noise (Gyro is smoother than differentiated Accel).
                                 2. No 'Derivative Kick' (Spike in output) if the Target Angle changes suddenly.

        :return: The control output value.
                 Unit: Arbitrary motor command units (typically scaled later to PWM).
        """
        # 1. INTEGRAL TERM
        # Accumulate error over time: Integral += Error * dt
        self.integral += error * loop_delta_time

        # Anti-windup: Clamp the integral to prevent it from growing indefinitely
        # if the robot is physically stuck or unable to correct the error.
        limit = self.params.integral_limit
        self.integral = clamp(self.integral, -limit, limit)

        # 2. DERIVATIVE TERM
        if measurement_rate is not None:
            # Derivative on Measurement (Preferred)
            # d(Error)/dt = d(Target - Measurement)/dt
            # If Target is constant, d(Target)/dt is 0.
            # So d(Error)/dt = -d(Measurement)/dt = -Rate
            derivative = -measurement_rate
        else:
            # Derivative on Error (Standard fallback)
            # Calculate slope of the error curve.
            # Note: This can be noisy if 'error' is noisy.
            derivative = (
                (error - self.last_error) / loop_delta_time
                if loop_delta_time > 0
                else 0.0
            )

        # 3. CALCULATE OUTPUT
        output = (
            (self.params.kp * error)
            + (self.params.ki * self.integral)
            + (self.params.kd * derivative)
        )

        # Save state for next loop
        self.last_error = error
        return output

    def reset(self) -> None:
        """
        Reset the internal state (integral and last error) of the controller.
        Call this when entering a new state (e.g., starting Balancing) to avoid
        stale data causing a jump.
        """
        self.integral = 0.0
        self.last_error = 0.0
