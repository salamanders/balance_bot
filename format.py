import re

with open('tests/test_kinematics_logic.py', 'r') as f:
    content = f.read()

content = content.replace("        def make_result(gyro, accel_delta):\n            res = MagicMock()\n            s = MagicMock(spec=IMUReading)\n            s.gyro_raw = gyro\n            s.accel_raw = accel_delta + glm.vec3(0, 0, 1.0) # Add baseline\n            res.samples = [s] * 10\n            return res",
"""        def make_result(gyro, accel_delta):
            res = MagicMock()
            s = MagicMock(spec=IMUReading)
            s.gyro_raw = gyro
            s.accel_raw = accel_delta + glm.vec3(0, 0, 1.0) # Add baseline
            res.samples = [s] * 10
            return res""")

with open('tests/test_kinematics_logic.py', 'w') as f:
    f.write(content)
