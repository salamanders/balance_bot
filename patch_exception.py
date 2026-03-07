import re

with open('src/balance_bot/hardware/piconzero.py', 'r') as f:
    content = f.read()

# Make sure cleanup doesn't swallow
replacement = """        except OSError:
            # We must attempt to explicitly write 0 to PWMs before passing
            try:
                self.bus.write_byte_data(self.I2C_ADDRESS, 0, 0)
                self.bus.write_byte_data(self.I2C_ADDRESS, 1, 0)
            except Exception:
                pass
            print("OSError during PiconZero cleanup. Halting.")
            raise"""

content = re.sub(r'        except OSError:\n            # We must attempt to explicitly write 0 to PWMs before passing\n            try:\n                self\.bus\.write_byte_data\(self\.I2C_ADDRESS, 0, 0\)\n                self\.bus\.write_byte_data\(self\.I2C_ADDRESS, 1, 0\)\n            except Exception:\n                pass', replacement, content, flags=re.DOTALL)

with open('src/balance_bot/hardware/piconzero.py', 'w') as f:
    f.write(content)
