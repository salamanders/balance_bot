import time
import logging
import smbus2
from ..utils import clamp

logger = logging.getLogger(__name__)

# Constants
I2C_ADDRESS = 0x22
RETRIES = 10

# Registers
REG_MOTOR_A = 0
REG_RESET = 20


class PiconZero:
    """
    Driver for 4tronix Picon Zero board.

    This is a modernized version of the original 4tronix driver, adapted for
    modern Python (3.10+) with type hints, logging, and integrated into
    the balance_bot project structure.
    """

    def __init__(self, bus_number: int = 1, retries: int = RETRIES):
        """
        Initialize the PiconZero driver.
        :param bus_number: I2C bus number (usually 1 for modern Pi, 0 for old).
        :param retries: Number of retries for I2C operations.
        """
        self.bus: smbus2.SMBus | None = None
        self.bus = smbus2.SMBus(bus_number)
        self.retries = retries
        logger.info(f"PiconZero initialized on bus {bus_number} with {retries} retries")

    def set_retries(self, retries: int) -> None:
        """Set the number of retries for I2C operations."""
        self.retries = retries

    def _i2c_op(self, method, reg: int, *args, op_name: str = "operation"):
        """Generic I2C operation with retries."""
        if self.bus is None:
            return None
        for _ in range(self.retries):
            try:
                return method(I2C_ADDRESS, reg, *args)
            except Exception:
                pass
        logger.error(f"Failed to {op_name} register {reg}")
        return None

    def _write_byte(self, reg: int, value: int) -> None:
        """Low-level I2C byte write with retries."""
        self._i2c_op(self.bus.write_byte_data, reg, value, op_name="write byte to")

    def _write_block(self, reg: int, data: list[int]) -> None:
        """Low-level I2C block write with retries."""
        self._i2c_op(self.bus.write_i2c_block_data, reg, data, op_name="write block to")

    def _read_word(self, reg: int) -> int:
        """Low-level I2C word read with retries."""
        val = self._i2c_op(self.bus.read_word_data, reg, op_name="read word from")
        return val if val is not None else 0

    def get_revision(self) -> tuple[int, int]:
        """
        Get the board revision.
        :return: Tuple of (major, minor) revision.
        """
        rval = self._read_word(0)
        return (rval // 256, rval % 256)

    def set_motor(self, motor: int, value: int) -> None:
        """
        Set motor speed.
        :param motor: Motor index (0 or 1).
        :param value: Speed -100 to 100.
        """
        if motor not in (0, 1):
            return

        # Original code check: value >= -128 and value < 128
        # We will clamp it to be safe
        safe_value = int(clamp(value, -127, 127))
        self._write_byte(motor, safe_value)

    def set_motors(self, motor_0_val: int, motor_1_val: int) -> None:
        """
        Set speed for both motors simultaneously.

        FIX: Use individual set_motor calls. The Picon Zero firmware does not
        appear to support I2C block writes for the motor registers (0 and 1).
        """
        self.set_motor(0, motor_0_val)
        self.set_motor(1, motor_1_val)

    def forward(self, speed: int) -> None:
        """
        Drive both motors forward.
        :param speed: Speed value.
        """
        self.set_motors(speed, speed)

    def reverse(self, speed: int) -> None:
        """
        Drive both motors in reverse.
        :param speed: Speed value.
        """
        self.set_motors(-speed, -speed)

    def spin_left(self, speed: int) -> None:
        """
        Spin robot left (Tank turn).
        :param speed: Speed value.
        """
        self.set_motors(-speed, speed)

    def spin_right(self, speed: int) -> None:
        """
        Spin robot right (Tank turn).
        :param speed: Speed value.
        """
        self.set_motors(speed, -speed)

    def stop(self) -> None:
        """Stop all motors."""
        self.set_motors(0, 0)

    def init(self) -> None:
        """Initialize/Reset the board."""
        self._write_byte(REG_RESET, 0)
        time.sleep(0.01)

    def cleanup(self) -> None:
        """Cleanup resources (reset board)."""
        self._write_byte(REG_RESET, 0)
        time.sleep(0.001)
