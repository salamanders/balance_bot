# Python library for 4tronix Picon Zero
# Note that all I2C accesses are wrapped in try clauses with repeats

import smbus
import time

# I2C address of Picon Zero
PZ_ADDR = 0x22

# Definitions of Commands to Picon Zero
CMD_MOTORA = 0
CMD_OUTCFG0 = 2
CMD_OUTPUT0 = 8
CMD_INCFG0 = 14
CMD_SETBRIGHT = 18
CMD_UPDATENOW = 19
CMD_RESET = 20
CMD_INPERIOD0 = 21

# General variables
DEBUG = False
RETRIES = 10   # max number of retries for I2C calls

# For revision 1 Raspberry Pi, change to bus = smbus.SMBus(0)
bus = smbus.SMBus(1)


def get_revision() -> list[int] | None:
    """Get Version and Revision info."""
    for _ in range(RETRIES):
        try:
            rval = bus.read_word_data(PZ_ADDR, 0)
            return [rval // 256, rval % 256]
        except OSError:
            if DEBUG:
                print("Error in get_revision(), retrying")
    return None


def set_motor(motor: int, value: int) -> None:
    """
    Set motor speed.
    :param motor: must be in range 0..1
    :param value: must be in range -128 - +127
    Values of -127, -128, +127 are treated as always ON, no PWM
    """
    if 0 <= motor <= 1 and -128 <= value < 128:
        for _ in range(RETRIES):
            try:
                bus.write_byte_data(PZ_ADDR, motor, value)
                break
            except OSError:
                if DEBUG:
                    print("Error in set_motor(), retrying")


def forward(speed: int) -> None:
    set_motor(0, speed)
    set_motor(1, speed)


def reverse(speed: int) -> None:
    set_motor(0, -speed)
    set_motor(1, -speed)


def spin_left(speed: int) -> None:
    set_motor(0, -speed)
    set_motor(1, speed)


def spin_right(speed: int) -> None:
    set_motor(0, speed)
    set_motor(1, -speed)


def stop() -> None:
    set_motor(0, 0)
    set_motor(1, 0)


def read_input(channel: int) -> int | None:
    """
    Read data for selected input channel (analog or digital).
    Channel is in range 0 to 3.
    """
    if 0 <= channel <= 3:
        for _ in range(RETRIES):
            try:
                return bus.read_word_data(PZ_ADDR, channel + 1)
            except OSError:
                if DEBUG:
                    print("Error in read_input(), retrying")
    return None


def set_output_config(output: int, value: int) -> None:
    """
    Set configuration of selected output.
    0: On/Off, 1: PWM, 2: Servo, 3: WS2812B
    """
    if 0 <= output <= 5 and 0 <= value <= 3:
        for _ in range(RETRIES):
            try:
                bus.write_byte_data(PZ_ADDR, CMD_OUTCFG0 + output, value)
                break
            except OSError:
                if DEBUG:
                    print("Error in set_output_config(), retrying")


def set_input_config(channel: int, value: int, pullup: bool = False, period: int = 2000) -> None:
    """
    Set configuration of selected input channel.
    0: Digital, 1: Analog, 2: DS18B20, 4: DutyCycle 5: Pulse Width
    """
    if 0 <= channel <= 3 and 0 <= value <= 5:
        if value == 0 and pullup:
            value = 128
        for _ in range(RETRIES):
            try:
                bus.write_byte_data(PZ_ADDR, CMD_INCFG0 + channel, value)
                if value in (4, 5):
                    bus.write_word_data(PZ_ADDR, CMD_INPERIOD0 + channel, period)
                break
            except OSError:
                if DEBUG:
                    print("Error in set_input_config(), retrying")


def set_output(channel: int, value: int) -> None:
    """
    Set output data for selected output channel.
    Mode  Name    Type    Values
    0     On/Off  Byte    0 is OFF, 1 is ON
    1     PWM     Byte    0 to 100 percentage of ON time
    2     Servo   Byte    -100 to + 100 Position in degrees
    3     WS2812B 4 Bytes 0:Pixel ID, 1:Red, 2:Green, 3:Blue
    """
    if 0 <= channel <= 5:
        for _ in range(RETRIES):
            try:
                bus.write_byte_data(PZ_ADDR, CMD_OUTPUT0 + channel, value)
                break
            except OSError:
                if DEBUG:
                    print("Error in set_output(), retrying")


def set_pixel(pixel: int, red: int, green: int, blue: int, update: bool = True) -> None:
    """Set the colour of an individual pixel (always output 5)."""
    pixel_data = [pixel, red, green, blue]
    for _ in range(RETRIES):
        try:
            bus.write_i2c_block_data(PZ_ADDR, int(update), pixel_data)
            break
        except OSError:
            if DEBUG:
                print("Error in set_pixel(), retrying")


def set_all_pixels(red: int, green: int, blue: int, update: bool = True) -> None:
    """Set all pixels to the same color."""
    pixel_data = [100, red, green, blue]
    for _ in range(RETRIES):
        try:
            bus.write_i2c_block_data(PZ_ADDR, int(update), pixel_data)
            break
        except OSError:
            if DEBUG:
                print("Error in set_all_pixels(), retrying")


def update_pixels() -> None:
    for _ in range(RETRIES):
        try:
            bus.write_byte_data(PZ_ADDR, CMD_UPDATENOW, 0)
            break
        except OSError:
            if DEBUG:
                print("Error in update_pixels(), retrying")


def set_brightness(brightness: int) -> None:
    """Set the overall brightness of pixel array."""
    for _ in range(RETRIES):
        try:
            bus.write_byte_data(PZ_ADDR, CMD_SETBRIGHT, brightness)
            break
        except OSError:
            if DEBUG:
                print("Error in set_brightness(), retrying")


def init(debug: bool = False) -> None:
    """Initialise the Board (same as cleanup)."""
    # Note: In the original code, 'DEBUG = debug' set a local variable,
    # leaving the global DEBUG False (unless manually set).
    # We replicate that behavior here by using the 'debug' argument locally
    # for this function's logic, while other functions use the global DEBUG.

    for _ in range(RETRIES):
        try:
            bus.write_byte_data(PZ_ADDR, CMD_RESET, 0)
            break
        except OSError:
            if debug:
                print("Error in init(), retrying")
    time.sleep(0.01)  # 1ms delay (actually 10ms here, replicating original 0.01)
    if debug:
        print("Debug is", debug)


def cleanup() -> None:
    """Cleanup the Board (same as init)."""
    for _ in range(RETRIES):
        try:
            bus.write_byte_data(PZ_ADDR, CMD_RESET, 0)
            break
        except OSError:
            if DEBUG:
                print("Error in cleanup(), retrying")
    time.sleep(0.001)   # 1ms delay
