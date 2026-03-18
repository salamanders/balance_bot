from simple_pid import PID
from ..configuration import PIDParams
from ..utils import clamp


class PIDController:
    """
    Wrapper around simple-pid to maintain compatibility with existing interface
    and support specific features like Gyro-based Derivative and Integral Clamping.

    This wrapper provides two main enhancements over standard simple-pid usage:
    1.  **Gyro-based Derivative**: Instead of calculating D from the numerical differentiation
        of the Error (or Input), we use the direct Gyroscope rate reading. This provides
        a much cleaner, lower-latency derivative term for balancing robots.
    2.  **Integral Term Clamping**: We explicitly clamp the stored integral value to a fixed range.
        Standard anti-windup usually limits the *total* output, but here we want to limit
        specifically the *authority* of the Integral term to prevent it from overwhelming
        the P and D terms during large disturbances.
    """

    def __init__(self, params: PIDParams):
        """
        Initialize the PID Controller.
        :param params: Reference to the shared PIDParams object.
        """
        # We initialize with dummy values, updated in update()
        # differential_on_measurement=True means D term is based on d(Input)/dt, not d(Error)/dt.
        # This avoids derivative kick on setpoint changes.
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

        :param error: Current Error (Setpoint - ProcessVariable).
        :param dt: Time delta since last update.
        :param measurement_rate: Optional direct rate measurement (e.g. Gyro) for D-term.
                                 If None, standard numerical differentiation is used.
        """
        # Sync parameters from config (in case they were tuned/changed)
        self.pid.Kp = self.params.kp
        self.pid.Ki = self.params.ki
        self.pid.Kd = self.params.kd
        self.pid.setpoint = self.params.target_angle

        # simple-pid expects the "Input" (Process Variable), not the Error.
        # Since Error = Setpoint - Input, we have Input = Setpoint - Error.
        input_val = self.pid.setpoint - error

        # --- Derivative Strategy ---
        if measurement_rate is not None:
            # STRATEGY: Use Gyro Rate for D-term.
            # This is cleaner than differentiating the Input (Pitch) which might be noisy.
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
            # D comes from (Input - LastInput) / dt
            output = self.pid(input_val, dt=dt)

        # --- Integral Clamping ---
        # "Anti-Windup" via hard clamping of the accumulated integral.
        limit = self.params.integral_limit
        if limit > 0:
            # We access private member _integral to clamp it.
            # This is necessary because simple-pid only supports output clamping,
            # but we want to clamp the I-term contribution specifically.
            if hasattr(self.pid, '_integral') and self.pid._integral is not None:
                original_integral = self.pid._integral
                clamped_integral = clamp(original_integral, -limit, limit)

                if clamped_integral != original_integral:
                    self.pid._integral = clamped_integral

                    # Adjust output to reflect clamping immediately.
                    # simple-pid stores _integral ALREADY SCALED by Ki.
                    # So _integral IS the I-term.
                    # Delta Output = NewIntegral - OldIntegral
                    delta_output = clamped_integral - original_integral
                    output += delta_output

        return output

    def reset(self) -> None:
        """Reset internal state."""
        self.pid.reset()
