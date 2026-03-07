import re

with open('src/balance_bot/hardware/piconzero.py', 'r') as f:
    content = f.read()

replacement = """    def _retry(self, func: Callable[[], Any], name: str) -> Any:
        \"\"\"Internal helper to retry I2C operations.\"\"\"
        for _ in range(self.retries):
            try:
                return func()
            except Exception:
                if self.debug:
                    print(f"Error in {name}(), retrying")
                time.sleep(0.005)

        # Explicitly try to disarm actuators on failure before raising
        if self.bus:
            try:
                self.bus.write_byte_data(self.I2C_ADDRESS, 0, 0)
                self.bus.write_byte_data(self.I2C_ADDRESS, 1, 0)
            except Exception:
                pass
        raise OSError(f"PiconZero {name}() failed after {self.retries} retries")"""

# Regex substitution
content = re.sub(r'    def _retry.*?raise OSError\(f"PiconZero \{name\}\(\) failed after \{self\.retries\} retries"\)', replacement, content, flags=re.DOTALL)

with open('src/balance_bot/hardware/piconzero.py', 'w') as f:
    f.write(content)
