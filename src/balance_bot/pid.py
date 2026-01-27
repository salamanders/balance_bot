from .config import PIDParams
from .utils import clamp


class PIDController:
    def __init__(self, params: PIDParams):
        self.params = params
        self.integral = 0.0
        self.last_error = 0.0

    def update(
        self, error: float, dt: float, measurement_rate: float | None = None
    ) -> float:
        """
        Calculate PID output.
        :param error: Current error (Target - Measured)
        :param dt: Time step in seconds
        :param measurement_rate: Optional rate of change of measurement (e.g. gyro rate).
                                 If provided, D term uses -measurement_rate.
                                 This avoids derivative kick on setpoint change and is less noisy.
        :return: Control output
        """
        self.integral += error * dt
        # Anti-windup
        limit = self.params.integral_limit
        self.integral = clamp(self.integral, -limit, limit)

        if measurement_rate is not None:
            # derivative of error = d(target - measurement)/dt
            # if target is constant, = - d(measurement)/dt = -measurement_rate
            derivative = -measurement_rate
        else:
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
