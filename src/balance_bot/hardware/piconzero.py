# Python library for 4tronix Picon Zero
# Refactored for Python 3 and code deduplication.

import smbus
import time

bus = smbus.SMBus(1)  # For modern Raspberry Pi (Rev 2+)
pzaddr = 0x22  # I2C address of Picon Zero

# Definitions of Commands to Picon Zero
RESET = 20

# General variables
DEBUG = False
RETRIES = 10   # max number of retries for I2C calls

def _retry(func, name):
    """Internal helper to retry I2C operations."""
    for _ in range(RETRIES):
        try:
            return func()
        except Exception:
            if DEBUG:
                print(f"Error in {name}(), retrying")
            time.sleep(0.005)
    raise OSError(f"PiconZero {name}() failed after retries")

def setMotor(motor, value):
    if 0 <= motor <= 1 and -128 <= value < 128:
        _retry(lambda: bus.write_byte_data(pzaddr, motor, value), "setMotor")

def stop():
    setMotor(0, 0)
    setMotor(1, 0)

def init(debug=False):
    global DEBUG
    DEBUG = debug
    _retry(lambda: bus.write_byte_data(pzaddr, RESET, 0), "init")
    time.sleep(0.1)
    if DEBUG:
        print("Debug is", DEBUG)

def cleanup():
    _retry(lambda: bus.write_byte_data(pzaddr, RESET, 0), "cleanup")
    time.sleep(0.001)
