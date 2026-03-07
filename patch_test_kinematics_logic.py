import re

with open('tests/test_kinematics_logic.py', 'r') as f:
    content = f.read()

replacement = """        # Drive and Measure Mock
        # We need to return specific responses for Left Pulse and Right Pulse.

        # Because we replaced drive_and_measure with execute_maneuver (ramping steps),
        # we need to mock execute_maneuver instead of drive_and_measure.

        # Scenario: Correct Setup (Not Swapped)
        # Forward -> +Y (Body Frame)
        # Pitch Back -> +X (Body Frame) (Nose Up)
        # Left Pulse -> Right Turn -> -Z (Body Frame)
        # Right Pulse -> Left Turn -> +Z (Body Frame)

        # Accel Lag (Inertial Force):
        # Forward Accel -> Backward Lag -> -Y

        # Left Pulse:
        # Expected: Right Turn (-Z) + Pitch Back (+X) + Forward Lag (-Y)
        l_gyro = glm.vec3(20, 0, -50) # Pitch Back (+X), Right Turn (-Z)
        l_accel_delta = glm.vec3(0, -0.5, 0) # Backward Lag (-Y)

        # Right Pulse:
        # Expected: Left Turn (+Z) + Pitch Back (+X) + Forward Lag (-Y)
        r_gyro = glm.vec3(20, 0, 50) # Pitch Back (+X), Left Turn (+Z)
        r_accel_delta = glm.vec3(0, -0.5, 0) # Backward Lag (-Y)

        def make_result(gyro, accel_delta):
            res = MagicMock()
            s = MagicMock(spec=IMUReading)
            s.gyro_raw = gyro
            s.accel_raw = accel_delta + glm.vec3(0, 0, 1.0) # Add baseline
            res.samples = [s] * 10
            return res

        hw.execute_maneuver.side_effect = [
            make_result(l_gyro, l_accel_delta),
            make_result(r_gyro, r_accel_delta)
        ]"""

content = re.sub(r'        # Drive and Measure Mock.*?\]', replacement, content, flags=re.DOTALL)

with open('tests/test_kinematics_logic.py', 'w') as f:
    f.write(content)
