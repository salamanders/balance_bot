import unittest
from unittest.mock import MagicMock
from balance_bot.reflex.pid import PIDController
from balance_bot.config import PIDParams

class TestPIDLogic(unittest.TestCase):
    def test_proportional(self):
        params = PIDParams(kp=10.0, ki=0.0, kd=0.0, target_angle=0.0)
        pid = PIDController(params)

        # Error = 5.0. Output should be Kp * Error = 50.0
        output = pid.update(error=5.0, dt=0.1)
        self.assertAlmostEqual(output, 50.0)

    def test_integral_clamping(self):
        # Ki=1.0, limit=20.0
        params = PIDParams(kp=0.0, ki=10.0, kd=0.0, integral_limit=20.0)
        pid = PIDController(params)

        # Update 1: Error=5, dt=1.0 -> Integral += 5. Output = 50.
        # Wait, simple-pid integrates: _integral += error * dt?
        # error = setpoint - input = 0 - (-5) = 5?
        # In update: input_val = setpoint - error = 0 - 5 = -5.
        # simple-pid: error = setpoint - input_val = 0 - (-5) = 5.
        # _integral += 5 * 1.0 = 5. Output = 10 * 5 = 50.

        output = pid.update(error=5.0, dt=1.0)
        self.assertAlmostEqual(output, 50.0)

        # Update 2: Error=5, dt=10.0 -> Integral += 50. Total=55.
        # Output = 10 * 55 = 550.
        # BUT limit is 20. So _integral should be clamped to 20.
        # However, output calculation happens BEFORE clamping in my implementation.
        # So output will be HUGE (550), but subsequent call will use clamped integral (20).

        output_huge = pid.update(error=5.0, dt=10.0)
        self.assertGreater(output_huge, 200.0) # Confirm it spiked

        # Check internal state clamped
        self.assertAlmostEqual(pid.pid._integral, 20.0)

        # Update 3: Error=0, dt=1.0. Integral stays at 20.
        # simple-pid integrates Ki * error * dt. So _integral IS the output contribution.
        # So clamping _integral to 20 means max I-term output is 20.
        # This matches docstring "UOM: Motor Output Units".

        output_clamped = pid.update(error=0.0, dt=1.0)
        self.assertAlmostEqual(output_clamped, 20.0)

    def test_derivative_on_measurement_gyro(self):
        # Kd=1.0
        params = PIDParams(kp=0.0, ki=0.0, kd=1.0, target_angle=0.0)
        pid = PIDController(params)

        # Scenario: Angle (Error) is 0. Rate is 10.0.
        # D-term should be -Kd * Rate = -10.0.
        # P and I are 0.

        output = pid.update(error=0.0, dt=0.1, measurement_rate=10.0)
        self.assertAlmostEqual(output, -10.0)

        # Scenario: Angle changes, Rate is 0.
        # D-term should be 0 (because we use Rate for D, not Angle diff).
        # We simulate Angle changing by calling update with different error?
        # But measurement_rate=0 passed explicitly.

        output = pid.update(error=5.0, dt=0.1, measurement_rate=0.0)
        self.assertAlmostEqual(output, 0.0) # P=0, I=0, D=-1*0=0.

    def test_derivative_on_error_fallback(self):
        # Kd=1.0
        params = PIDParams(kp=0.0, ki=0.0, kd=1.0, target_angle=0.0)
        pid = PIDController(params)

        # No measurement_rate passed. Should differentiate error.
        # Update 1: Error=0. Input=0. LastInput=Init(0?). D=0.
        pid.update(error=0.0, dt=0.1)

        # Update 2: Error=1.0. Input=-1.0.
        # D = -Kd * (Input - LastInput) / dt
        # D = -1.0 * (-1.0 - 0.0) / 0.1 = -1.0 * -10.0 = 10.0?
        # Wait. Derivative of Error. Error goes 0->1 (Increasing).
        # Slope is +10. Kd*Slope = 10.
        # simple-pid subtracts D term?
        # PID Output = Kp*e + Ki*int + Kd*der?
        # usually D term OPPOSES change.
        # simple-pid implements: `output = ... + Kd * derivative`?
        # Or `... - Kd * derivative` (if on measurement)?
        # simple-pid defaults `differential_on_measurement=True`.
        # So D = -Kd * (Input - LastInput) / dt.
        # Input goes 0 -> -1. (Error goes 0 -> 1).
        # D = -1 * (-1 - 0) / 0.1 = -1 * -10 = 10.
        # So it adds +10 to output.
        # Positive error -> Positive output to correct it.
        # Derivative is positive (error increasing). Output should increase to correct faster?
        # Yes.

        output = pid.update(error=1.0, dt=0.1)
        self.assertAlmostEqual(output, 10.0)

if __name__ == "__main__":
    unittest.main()
