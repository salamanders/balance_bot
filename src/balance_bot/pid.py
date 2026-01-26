from .config import PIDParams


class PIDController:
    def __init__(self, params: PIDParams):
        self.params = params
        self.integral = 0.0
        self.last_error = 0.0
        self.integral_limit = 20.0

    def update(self, error: float, dt: float) -> float:
        """
        Calculate PID output.
        :param error: Current error (Target - Measured)
        :param dt: Time step in seconds
        :return: Control output
        """
        self.integral += error * dt
        # Anti-windup
        self.integral = max(
            min(self.integral, self.integral_limit), -self.integral_limit
        )

        derivative = (error - self.last_error) / dt if dt > 0 else 0.0

        output = (
            (self.params.kp * error)
            + (self.params.ki * self.integral)
            + (self.params.kd * derivative)
        )

        self.last_error = error
        return output

    def reset(self) -> None:
        self.integral = 0.0
        self.last_error = 0.0
