from simple_pid import PID
from ..configuration import LearningState
from ..utils import clamp


class PIDController:
    """
    Wrapper around simple-pid to maintain compatibility with existing interface
    and support specific features like Gyro-based Derivative and Integral Clamping.
    """

    def __init__(self, state: LearningState, integral_limit: float = 20.0):
        """
        Initialize the PID Controller.
        :param state: Reference to the shared LearningState object (mutable).
        :param integral_limit: Limit for the integral term (from HardwareConfig).
        """
        self.state = state
        self.integral_limit = integral_limit

        # We initialize with dummy values, updated in update()
        # differential_on_measurement=True means D term is based on d(Input)/dt, not d(Error)/dt.
        self.pid = PID(
            Kp=state.kp,
            Ki=state.ki,
            Kd=state.kd,
            setpoint=state.target_angle,
            output_limits=(None, None),  # We handle limits/clamping manually
            differential_on_measurement=True
        )

    def update(
        self,
        error: float,
        dt: float,
        measurement_rate: float | None = None,
    ) -> float:
        """
        Calculate the next control output.

        :param error: Current Error (Setpoint - ProcessVariable).
        :param dt: Time delta since last update.
        :param measurement_rate: Optional direct rate measurement (e.g. Gyro) for D-term.
        """
        # Sync parameters from mutable state
        self.pid.Kp = self.state.kp
        self.pid.Ki = self.state.ki
        self.pid.Kd = self.state.kd
        self.pid.setpoint = self.state.target_angle

        # simple-pid expects the "Input" (Process Variable), not the Error.
        # Since Error = Setpoint - Input, we have Input = Setpoint - Error.
        input_val = self.pid.setpoint - error

        output = 0.0

        # --- Derivative Strategy ---
        if measurement_rate is not None:
            # STRATEGY: Use Gyro Rate for D-term.
            # We temporarily disable the internal D-term calculation of simple-pid.
            real_kd = self.pid.Kd
            self.pid.Kd = 0.0

            # Compute P + I (and update internal state)
            output = self.pid(input_val, dt=dt)

            # Restore Kd for next time
            self.pid.Kd = real_kd

            # Add External D-term manually
            # D = -Kd * rate (Negative because rate is change of input, acts against change)
            output += -real_kd * measurement_rate
        else:
            # STRATEGY: Standard PID
            output = self.pid(input_val, dt=dt)

        # --- Integral Clamping ---
        # "Anti-Windup" via hard clamping of the accumulated integral.
        limit = self.integral_limit
        if limit > 0:
            if hasattr(self.pid, '_integral') and self.pid._integral is not None:
                original_integral = self.pid._integral
                clamped_integral = clamp(original_integral, -limit, limit)

                if clamped_integral != original_integral:
                    self.pid._integral = clamped_integral

                    # Adjust output to reflect clamping immediately.
                    delta_output = clamped_integral - original_integral
                    output += delta_output

        return output

    def reset(self) -> None:
        """Reset internal state."""
        self.pid.reset()
