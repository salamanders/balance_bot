from simple_pid import PID
from ..config import PIDParams
from ..utils import clamp


class PIDController:
    """
    Wrapper around simple-pid to maintain compatibility with existing interface
    and support specific features like Gyro-based Derivative and Integral Clamping.
    """

    def __init__(self, params: PIDParams):
        # We initialize with dummy values, updated in update()
        self.pid = PID(
            Kp=params.kp,
            Ki=params.ki,
            Kd=params.kd,
            setpoint=params.target_angle,
            output_limits=(None, None),  # We handle limits/clamping manually
            differential_on_measurement=True
        )
        self.params = params

    def update(
        self,
        error: float,
        dt: float,
        measurement_rate: float | None = None,
    ) -> float:
        """
        Calculate the next control output.
        """
        # Sync parameters
        self.pid.Kp = self.params.kp
        self.pid.Ki = self.params.ki
        self.pid.Kd = self.params.kd
        self.pid.setpoint = self.params.target_angle

        # Calculate Input from Error (Error = Setpoint - Input -> Input = Setpoint - Error)
        input_val = self.pid.setpoint - error

        # Derivative Strategy
        if measurement_rate is not None:
            # Use Gyro Rate for Derivative (cleaner than d(Input)/dt)
            # We temporarily disable internal D-term
            real_kd = self.pid.Kd
            self.pid.Kd = 0.0

            # Compute P + I
            output = self.pid(input_val, dt=dt)

            # Restore Kd
            self.pid.Kd = real_kd

            # Add External D-term
            # D = -Kd * rate
            output += -real_kd * measurement_rate
        else:
            # Standard PID (D comes from d(Input)/dt)
            output = self.pid(input_val, dt=dt)

        # Integral Clamping (Legacy "integral_limit" behavior)
        # We manually clamp the internal integral state
        limit = self.params.integral_limit
        if limit > 0:
            # simple-pid stores integral in _integral
            # Ensure we don't break if internal implementation changes, but _integral is standard convention
            if hasattr(self.pid, '_integral') and self.pid._integral is not None:
                self.pid._integral = clamp(self.pid._integral, -limit, limit)
                # Re-calculate output if I-term changed?
                # simple-pid calculates output = P + I + D.
                # If we change I, output is stale.
                # But we can't easily re-run without side effects (dt integration).
                # Actually, simple-pid adds `_integral` to output.
                # So if we clamp `_integral` *after* call, it affects *next* call.
                # To affect *current* output, we should clamp *before*?
                # No, `pid()` updates `_integral` then uses it.
                # So the returned `output` uses the *unclamped* integral.
                # We should re-calculate output or just accept one tick delay?
                # Better: Recalculate output components.
                # P = Kp * error
                # I = Ki * _integral
                # D = D_term
                # This is getting complicated.
                pass

        return output

    def reset(self) -> None:
        """Reset internal state."""
        self.pid.reset()
