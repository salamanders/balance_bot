"""
Balance Bot Package Initialization.

This module sets up a compatibility shim for `smbus`.
The `mpu6050-raspberrypi` library depends on the `smbus` package (specifically C-based).
We standardize on `smbus2` (pure Python) and alias it to `smbus` so that legacy
dependencies continue to work without the actual `smbus` package being installed.
"""
import sys
import smbus2

# Shim: Alias smbus2 to smbus for dependencies that expect 'import smbus'
sys.modules["smbus"] = smbus2
