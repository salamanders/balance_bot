import re

with open('src/balance_bot/hardware/robot_hardware.py', 'r') as f:
    content = f.read()

replacement = """        except OSError:
            self._imu_consecutive_errors += 1
            if self._imu_consecutive_errors > self.hw_config.imu_max_retries:
                logger.error(f"IMU Failed {self._imu_consecutive_errors} times in a row. Raising Error.")
                if self.pz:
                    try:
                        self.pz.stop()
                    except Exception:
                        pass
                raise OSError("IMU I2C communication failed repeatedly")

            # I2C glitch - do not return stale data, must raise if exceeding limit
            # If we don't exceed, we can try to return the last good but it is risky
            # The instructions explicitly say: "Instead of returning `self._last_accel` infinitely for glitches, ensure it explicitly calls `self.pz.stop()` to disarm actuators and then strictly raises the exception to halt the system rather than falling back to "best effort"."
            # Wait, the prompt says "ensure it explicitly calls self.pz.stop() to disarm actuators and then strictly raises the exception to halt the system rather than falling back to "best effort"."

            logger.error(f"IMU Glitch. Halting actuators.")
            if self.pz:
                try:
                    self.pz.stop()
                except Exception:
                    pass
            raise OSError("IMU I2C communication failed. Halting system instead of using stale data.")"""

pattern = r'        except OSError:.*?return self\._last_accel, self\._last_gyro'

content = re.sub(pattern, replacement, content, flags=re.DOTALL)

with open('src/balance_bot/hardware/robot_hardware.py', 'w') as f:
    f.write(content)
