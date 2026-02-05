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
REG_OUT_CFG_0 = 2
REG_OUTPUT_0 = 8
REG_IN_CFG_0 = 14
REG_SET_BRIGHT = 18
REG_UPDATE_NOW = 19
REG_RESET = 20


class PiconZero:
    """
    Driver for 4tronix Picon Zero board.

    This is a modernized version of the original 4tronix driver, adapted for
    modern Python (3.10+) with type hints, logging, and integrated into
    the balance_bot project structure.
    """

    def __init__(self, bus_number: int = 1):
        """
        Initialize the PiconZero driver.
        :param bus_number: I2C bus number (usually 1 for modern Pi, 0 for old).
        """
        self.bus: smbus2.SMBus | None = None
        self.bus = smbus2.SMBus(bus_number)
        logger.info(f"PiconZero initialized on bus {bus_number}")

    def _write_byte(self, reg: int, value: int) -> None:
        """Low-level I2C byte write with retries."""
        if self.bus is None:
            return
        for _ in range(RETRIES):
            try:
                self.bus.write_byte_data(I2C_ADDRESS, reg, value)
                return
            except Exception:
                pass
        logger.error(f"Failed to write byte to register {reg}")

    def _write_block(self, reg: int, data: list[int]) -> None:
        """Low-level I2C block write with retries."""
        if self.bus is None:
            return
        for _ in range(RETRIES):
            try:
                self.bus.write_i2c_block_data(I2C_ADDRESS, reg, data)
                return
            except Exception:
                pass
        logger.error(f"Failed to write block to register {reg}")

    def _read_word(self, reg: int) -> int:
        """Low-level I2C word read with retries."""
        if self.bus is None:
            return 0
        for _ in range(RETRIES):
            try:
                return self.bus.read_word_data(I2C_ADDRESS, reg)
            except Exception:
                pass
        logger.error(f"Failed to read word from register {reg}")
        return 0

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
        Set speed for both motors simultaneously using a block write.
        :param motor_0_val: Speed for Motor 0 (-100 to 100).
        :param motor_1_val: Speed for Motor 1 (-100 to 100).
        """
        safe_val0 = int(clamp(motor_0_val, -127, 127))
        safe_val1 = int(clamp(motor_1_val, -127, 127))
        self._write_block(REG_MOTOR_A, [safe_val0, safe_val1])

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

    def read_input(self, channel: int) -> int:
        """
        Read data from selected input channel (0-3).
        :param channel: Input channel index.
        :return: Read value.
        """
        if 0 <= channel <= 3:
            return self._read_word(channel + 1)
        return 0

    def set_output_config(self, output: int, mode: int) -> None:
        """
        Set configuration of selected output.
        :param output: 0-5
        :param mode: 0: On/Off, 1: PWM, 2: Servo, 3: WS2812B
        """
        if 0 <= output <= 5 and 0 <= mode <= 3:
            self._write_byte(REG_OUT_CFG_0 + output, mode)

    def set_input_config(self, channel: int, mode: int, pullup: bool = False) -> None:
        """
        Set configuration of selected input channel.
        :param channel: 0-3
        :param mode: 0: Digital, 1: Analog
        :param pullup: Enable pullup resistor (for Digital mode)
        """
        if 0 <= channel <= 3 and 0 <= mode <= 3:
            val = mode
            if mode == 0 and pullup:
                val = 128
            self._write_byte(REG_IN_CFG_0 + channel, val)

    def set_output(self, channel: int, value: int) -> None:
        """
        Set output data for selected output channel.
        :param channel: 0-5
        :param value: Depends on mode (0-100 for PWM, -100 to 100 for Servo)
        """
        if 0 <= channel <= 5:
            self._write_byte(REG_OUTPUT_0 + channel, value)

    def set_pixel(
        self, pixel: int, red: int, green: int, blue: int, update: bool = True
    ) -> None:
        """
        Set the color of an individual pixel (Output 5 must be WS2812B).
        :param pixel: Pixel index.
        :param red: Red component (0-255).
        :param green: Green component (0-255).
        :param blue: Blue component (0-255).
        :param update: Whether to update immediately.
        """
        pixel_data = [pixel, red, green, blue]
        self._write_block(1 if update else 0, pixel_data)

    def set_all_pixels(self, red: int, green: int, blue: int, update: bool = True) -> None:
        """
        Set all pixels to a color.
        :param red: Red component (0-255).
        :param green: Green component (0-255).
        :param blue: Blue component (0-255).
        :param update: Whether to update immediately.
        """
        pixel_data = [100, red, green, blue]
        self._write_block(1 if update else 0, pixel_data)

    def update_pixels(self) -> None:
        """Trigger update of pixels."""
        self._write_byte(REG_UPDATE_NOW, 0)

    def set_brightness(self, brightness: int) -> None:
        """
        Set overall brightness of pixel array.
        :param brightness: Brightness value (0-255).
        """
        self._write_byte(REG_SET_BRIGHT, brightness)

    def init(self) -> None:
        """Initialize/Reset the board."""
        self._write_byte(REG_RESET, 0)
        time.sleep(0.01)

    def cleanup(self) -> None:
        """Cleanup resources (reset board)."""
        self._write_byte(REG_RESET, 0)
        time.sleep(0.001)
