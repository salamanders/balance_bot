from ..config import PIDParams
from ..utils import clamp


class PIDController:
    """
    Standard PID Controller implementation with Anti-Windup and Derivative-on-Measurement.

    Logic:
        Output = (Kp * Error) + (Ki * Integral) + (Kd * Derivative)
    """

    def __init__(self, params: PIDParams):
        """
        Initialize the PID Controller.
        :param params: Configuration parameters (Kp, Ki, Kd, limits).
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
        Calculate the next control output.

        Derivative Calculation Strategy:
         - If `measurement_rate` is provided (e.g., from Gyro), use it directly.
           D-term = -Kd * rate. (Derivative on Measurement).
           This avoids "derivative kick" when the target changes and is less noisy.
         - Otherwise, calculate derivative from error difference.
           D-term = Kd * (error - last_error) / dt.

        Integral Strategy:
         - Accumulate error * dt.
         - Clamp sum to `integral_limit` (Anti-Windup).

        :param error: Current error (Target - Measured).
        :param loop_delta_time: Time elapsed since last update (seconds).
        :param measurement_rate: (Optional) Rate of change of the process variable.
        :return: Computed control output.
        """
        self.integral += error * loop_delta_time
        # Anti-windup
        limit = self.params.integral_limit
        self.integral = clamp(self.integral, -limit, limit)

        if measurement_rate is not None:
            # Derivative on Measurement
            # d(Error)/dt = d(Setpoint - Process)/dt
            # If Setpoint is constant, d(Error)/dt = -d(Process)/dt
            derivative = -measurement_rate
        else:
            # Derivative on Error
            derivative = (
                (error - self.last_error) / loop_delta_time
                if loop_delta_time > 0
                else 0.0
            )

        output = (
            (self.params.kp * error)
            + (self.params.ki * self.integral)
            + (self.params.kd * derivative)
        )

        self.last_error = error
        return output

    def reset(self) -> None:
        """
        Reset the internal state.
        Clears accumulated integral and last error history.
        """
        self.integral = 0.0
        self.last_error = 0.0
