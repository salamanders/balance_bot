import re

with open('src/balance_bot/hardware/robot_hardware.py', 'r') as f:
    content = f.read()

replacement = """        except OSError:
            self._imu_consecutive_errors += 1
            if self._imu_consecutive_errors > self.hw_config.imu_max_retries:
                logger.error(f"IMU Failed {self._imu_consecutive_errors} times in a row. Raising Error.")
            else:
                logger.error(f"IMU Glitch. Halting actuators.")

            if self.pz:
                try:
                    self.pz.stop()
                except Exception:
                    pass
            raise OSError("IMU I2C communication failed. Halting system instead of using stale data.")"""

pattern = r'        except OSError:.*?raise OSError\("IMU I2C communication failed\. Halting system instead of using stale data\."\)'

content = re.sub(pattern, replacement, content, flags=re.DOTALL)

with open('src/balance_bot/hardware/robot_hardware.py', 'w') as f:
    f.write(content)
