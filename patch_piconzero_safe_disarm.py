import re

with open('src/balance_bot/hardware/piconzero.py', 'r') as f:
    content = f.read()

replacement1 = """        # Explicitly try to disarm actuators on failure before raising
        if self.bus:
            try:
                self.bus.write_byte_data(self.I2C_ADDRESS, 0, 0)
            except Exception:
                pass
            try:
                self.bus.write_byte_data(self.I2C_ADDRESS, 1, 0)
            except Exception:
                pass
        raise"""

content = re.sub(r'        # Explicitly try to disarm actuators on failure before raising\n        if self\.bus:\n            try:\n                self\.bus\.write_byte_data\(self\.I2C_ADDRESS, 0, 0\)\n                self\.bus\.write_byte_data\(self\.I2C_ADDRESS, 1, 0\)\n            except Exception:\n                pass\n        raise', replacement1, content, flags=re.DOTALL)


replacement2 = """        except OSError:
            # We must attempt to explicitly write 0 to PWMs before passing
            try:
                self.bus.write_byte_data(self.I2C_ADDRESS, 0, 0)
            except Exception:
                pass
            try:
                self.bus.write_byte_data(self.I2C_ADDRESS, 1, 0)
            except Exception:
                pass
            print("OSError during PiconZero cleanup. Halting.")
            raise"""

content = re.sub(r'        except OSError:\n            # We must attempt to explicitly write 0 to PWMs before passing\n            try:\n                self\.bus\.write_byte_data\(self\.I2C_ADDRESS, 0, 0\)\n                self\.bus\.write_byte_data\(self\.I2C_ADDRESS, 1, 0\)\n            except Exception:\n                pass\n            print\("OSError during PiconZero cleanup\. Halting\."\)\n            raise', replacement2, content, flags=re.DOTALL)

with open('src/balance_bot/hardware/piconzero.py', 'w') as f:
    f.write(content)
