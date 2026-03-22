import re

with open('src/balance_bot/hardware/robot_hardware.py', 'r') as f:
    content = f.read()

# When we mock `hw.sensor = MagicMock()`, the background thread might crash, or not get the updated mock fast enough, or we need to just bypass the background thread logic when `sensor` is replaced directly in tests.

# Wait, `get_accel_data` and `get_gyro_data` are mock. Since they run in the background, `read_imu_converted` immediately returns the OLD value (vec(0,0,0)) because the thread hasn't run yet!
# We can force a synchronous thread execution by modifying read_imu_raw to read the sensor directly if not running or if it's a test.
# Actually, since it's a daemon thread, in tests it might just not run immediately.

new_read_imu_raw = """    def read_imu_raw(self) -> tuple[glm.vec3, glm.vec3]:
        \"\"\"
        Returns raw accelerometer and gyro data instantly from the background thread buffer.
        :return: Tuple of (accel_dict, gyro_dict).
        \"\"\"
        if self.sensor is None:
            raise RuntimeError("IMU Sensor not initialized (Bus Unknown?)")

        # Synchronous fallback for tests when sensor is mocked
        if not self._sensor_running:
            try:
                accel = self.sensor.get_accel_data()
                gyro = self.sensor.get_gyro_data()

                bias_vec = glm.vec3(
                    self.learning_state.gyro_bias_x,
                    self.learning_state.gyro_bias_y,
                    self.learning_state.gyro_bias_z
                )
                gyro = gyro - bias_vec

                self._last_accel = accel
                self._last_gyro = gyro
                self._imu_consecutive_errors = 0
            except OSError:
                self._imu_consecutive_errors += 1

        with self._sensor_lock:
            accel = self._last_accel
            gyro = self._last_gyro
            errors = self._imu_consecutive_errors

        if errors > self.hw_config.imu_max_retries:
            logger.error(f"IMU Failed {errors} times in a row. Returning cached data indefinitely to avoid fatal crash.")
        elif errors > 0:
            logger.debug(f"IMU Glitch ({errors}/{self.hw_config.imu_max_retries}). Using cached data.")

        return accel, gyro"""

content = re.sub(r'    def read_imu_raw\(self\) -> tuple\[glm\.vec3, glm\.vec3\]:.*?            return accel, gyro', new_read_imu_raw, content, flags=re.DOTALL)

with open('src/balance_bot/hardware/robot_hardware.py', 'w') as f:
    f.write(content)


# Also in the tests, we need to stop the thread when mocking:
with open('tests/test_imu_logic.py', 'r') as f:
    content = f.read()

content = content.replace("hw.sensor = MagicMock()", "hw.stop_sensor_thread(); hw.sensor = MagicMock()")

with open('tests/test_imu_logic.py', 'w') as f:
    f.write(content)

with open('tests/test_imu_resilience.py', 'r') as f:
    content = f.read()

content = content.replace("hw.sensor = MagicMock()", "hw.stop_sensor_thread(); hw.sensor = MagicMock()")

with open('tests/test_imu_resilience.py', 'w') as f:
    f.write(content)

with open('tests/test_imu_yaw_roll.py', 'r') as f:
    content = f.read()

content = content.replace("hw.sensor = MagicMock()", "hw.stop_sensor_thread(); hw.sensor = MagicMock()")

with open('tests/test_imu_yaw_roll.py', 'w') as f:
    f.write(content)
