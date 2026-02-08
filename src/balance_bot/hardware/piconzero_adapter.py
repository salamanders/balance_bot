import smbus
from . import piconzero as pz

class PiconZeroAdapter:
    """
    Adapter to wrap the module-based piconzero driver into a class
    compatible with the MotorDriver protocol.
    """
    def __init__(self, bus_number=1):
        self.bus_number = bus_number

        # piconzero module initializes 'bus = smbus.SMBus(1)' at import time.
        # If we need a different bus, or if the current bus handle is stale/invalid
        # (e.g. due to other SMBus instances probing the same bus), we must refresh it.

        # Always force a refresh of the bus connection to ensure it's clean.
        # This handles cases where:
        # 1. Bus number changed.
        # 2. Bus number is same (1), but the handle is stale or interfered with by WiringCheck probing.
        try:
            if hasattr(pz.bus, 'close'):
                pz.bus.close()
        except Exception:
            pass

        # Open new one
        pz.bus = smbus.SMBus(bus_number)

    def init(self) -> None:
        """Initialize the motor driver hardware."""
        pz.init()

    def cleanup(self) -> None:
        """Release hardware resources."""
        pz.cleanup()

    def stop(self) -> None:
        """Stop all motors immediately."""
        pz.stop()

    def set_retries(self, retries: int) -> None:
        """Set the number of I2C retries."""
        pz.RETRIES = retries

    def set_motor(self, motor: int, value: int) -> None:
        """
        Set speed for a specific motor.
        :param motor: Motor channel index (0 or 1).
        :param value: Speed (-100 to 100).
        """
        pz.setMotor(motor, value)

    def set_motors(self, motor_0_val: int, motor_1_val: int) -> None:
        """
        Set speed for both motors.
        The module doesn't support block write for motors, so we call setMotor twice.
        :param motor_0_val: Speed for Motor 0 (-100 to 100).
        :param motor_1_val: Speed for Motor 1 (-100 to 100).
        """
        pz.setMotor(0, motor_0_val)
        pz.setMotor(1, motor_1_val)
