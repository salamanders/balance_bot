import re

with open('src/balance_bot/hardware/piconzero.py', 'r') as f:
    content = f.read()

replacement = """        except OSError:
            # We must attempt to explicitly write 0 to PWMs before passing
            try:
                self.bus.write_byte_data(self.I2C_ADDRESS, 0, 0)
                self.bus.write_byte_data(self.I2C_ADDRESS, 1, 0)
            except Exception:
                pass"""

content = re.sub(r'        except OSError:\n            pass # Best effort cleanup', replacement, content, flags=re.DOTALL)

with open('src/balance_bot/hardware/piconzero.py', 'w') as f:
    f.write(content)
