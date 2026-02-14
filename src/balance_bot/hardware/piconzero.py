# Python library for 4tronix Picon Zero
# Refactored for Python 3 and code deduplication.

import smbus
import time

bus = smbus.SMBus(1)  # For modern Raspberry Pi (Rev 2+)
pzaddr = 0x22  # I2C address of Picon Zero

# Definitions of Commands to Picon Zero
MOTORA = 0
OUTCFG0 = 2
OUTPUT0 = 8
INCFG0 = 14
SETBRIGHT = 18
UPDATENOW = 19
RESET = 20
INPERIOD0 = 21

# General variables
DEBUG = False
RETRIES = 10   # max number of retries for I2C calls

def _retry(func, name):
    """Internal helper to retry I2C operations."""
    for _ in range(RETRIES):
        try:
            return func()
        except Exception:
    for i in range(RETRIES):
        try:
            return func()
        except:
            if DEBUG:
                print(f"Error in {name}(), retrying")
    raise OSError(f"PiconZero {name}() failed after retries")

def getRevision():
    def _read():
        rval = bus.read_word_data(pzaddr, 0)
        return [rval // 256, rval % 256]
    return _retry(_read, "getRevision")

def setMotor(motor, value):
    if 0 <= motor <= 1 and -128 <= value < 128:
        _retry(lambda: bus.write_byte_data(pzaddr, motor, value), "setMotor")

def forward(speed):
    setMotor(0, speed)
    setMotor(1, speed)

def reverse(speed):
    setMotor(0, -speed)
    setMotor(1, -speed)

def spinLeft(speed):
    setMotor(0, -speed)
    setMotor(1, speed)

def spinRight(speed):
    setMotor(0, speed)
    setMotor(1, -speed)

def stop():
    setMotor(0, 0)
    setMotor(1, 0)

def readInput(channel):
    if 0 <= channel <= 3:
        return _retry(lambda: bus.read_word_data(pzaddr, channel + 1), "readInput")

def setOutputConfig(output, value):
    if 0 <= output <= 5 and 0 <= value <= 3:
        _retry(lambda: bus.write_byte_data(pzaddr, OUTCFG0 + output, value), "setOutputConfig")

def setInputConfig(channel, value, pullup=False, period=2000):
    if 0 <= channel <= 3 and 0 <= value <= 5:
        if value == 0 and pullup:
            value = 128

        def _do_config():
            bus.write_byte_data(pzaddr, INCFG0 + channel, value)
            if value == 4 or value == 5:
                bus.write_word_data(pzaddr, INPERIOD0 + channel, period)

        _retry(_do_config, "setInputConfig")

def setOutput(channel, value):
    if 0 <= channel <= 5:
        _retry(lambda: bus.write_byte_data(pzaddr, OUTPUT0 + channel, value), "setOutput")

def setPixel(Pixel, Red, Green, Blue, Update=True):
    pixelData = [Pixel, Red, Green, Blue]
    _retry(lambda: bus.write_i2c_block_data(pzaddr, Update, pixelData), "setPixel")

def setAllPixels(Red, Green, Blue, Update=True):
    pixelData = [100, Red, Green, Blue]
    _retry(lambda: bus.write_i2c_block_data(pzaddr, Update, pixelData), "setAllPixels")

def updatePixels():
    _retry(lambda: bus.write_byte_data(pzaddr, UPDATENOW, 0), "updatePixels")

def setBrightness(brightness):
    _retry(lambda: bus.write_byte_data(pzaddr, SETBRIGHT, brightness), "setBrightness")

def init(debug=False):
    global DEBUG
    DEBUG = debug
    _retry(lambda: bus.write_byte_data(pzaddr, RESET, 0), "init")
    time.sleep(0.01)
    if DEBUG:
#---------------------------------------------
# Get Version and Revision info
def getRevision():
    def _read():
        rval = bus.read_word_data(pzaddr, 0)
        return [rval//256, rval%256]
    return _retry(_read, "getRevision")
#---------------------------------------------


#---------------------------------------------
# motor must be in range 0..1
# value must be in range -128 - +127
# values of -127, -128, +127 are treated as always ON,, no PWM
def setMotor (motor, value):
    if (motor>=0 and motor<=1 and value>=-128 and value<128):
        _retry(lambda: bus.write_byte_data(pzaddr, motor, value), "setMotor")

def forward (speed):
    setMotor (0, speed)
    setMotor (1, speed)

def reverse (speed):
    setMotor (0, -speed)
    setMotor (1, -speed)

def spinLeft (speed):
    setMotor (0, -speed)
    setMotor (1, speed)

def spinRight (speed):
    setMotor (0, speed)
    setMotor (1, -speed)

def stop():
    setMotor (0, 0)
    setMotor (1, 0)

#---------------------------------------------

#---------------------------------------------
# Read data for selected input channel (analog or digital)
# Channel is in range 0 to 3
def readInput (channel):
    if (channel>=0 and channel <=3):
        return _retry(lambda: bus.read_word_data(pzaddr, channel + 1), "readInput")

#---------------------------------------------

#---------------------------------------------
# Set configuration of selected output
# 0: On/Off, 1: PWM, 2: Servo, 3: WS2812B
def setOutputConfig (output, value):
    if (output>=0 and output<=5 and value>=0 and value<=3):
        _retry(lambda: bus.write_byte_data(pzaddr, OUTCFG0 + output, value), "setOutputConfig")
#---------------------------------------------

#---------------------------------------------
# Set configuration of selected input channel
# 0: Digital, 1: Analog, 2: DS18B20, 4: DutyCycle 5: Pulse Width
def setInputConfig (channel, value, pullup = False, period = 2000):
    if (channel >= 0 and channel <= 3 and value >= 0 and value <= 5):
        if (value == 0 and pullup == True):
            value = 128
        def _write():
            bus.write_byte_data(pzaddr, INCFG0 + channel, value)
            if (value == 4 or value == 5):
                bus.write_word_data(pzaddr, INPERIOD0 + channel, period)
        _retry(_write, "setInputConfig")
#---------------------------------------------

#---------------------------------------------
# Set output data for selected output channel
# Mode  Name    Type    Values
# 0     On/Off  Byte    0 is OFF, 1 is ON
# 1     PWM     Byte    0 to 100 percentage of ON time
# 2     Servo   Byte    -100 to + 100 Position in degrees
# 3     WS2812B 4 Bytes 0:Pixel ID, 1:Red, 2:Green, 3:Blue
def setOutput (channel, value):
    if (channel>=0 and channel<=5):
        _retry(lambda: bus.write_byte_data(pzaddr, OUTPUT0 + channel, value), "setOutput")
#---------------------------------------------

#---------------------------------------------
# Set the colour of an individual pixel (always output 5)
def setPixel (Pixel, Red, Green, Blue, Update=True):
    pixelData = [Pixel, Red, Green, Blue]
    _retry(lambda: bus.write_i2c_block_data(pzaddr, Update, pixelData), "setPixel")

def setAllPixels (Red, Green, Blue, Update=True):
    pixelData = [100, Red, Green, Blue]
    _retry(lambda: bus.write_i2c_block_data(pzaddr, Update, pixelData), "setAllPixels")

def updatePixels ():
    _retry(lambda: bus.write_byte_data(pzaddr, UPDATENOW, 0), "updatePixels")

#---------------------------------------------

#---------------------------------------------
# Set the overall brightness of pixel array
def setBrightness (brightness):
    _retry(lambda: bus.write_byte_data(pzaddr, SETBRIGHT, brightness), "setBrightness")
#---------------------------------------------

#---------------------------------------------
# Initialise the Board (same as cleanup)
def init (debug=False):
    global DEBUG
    DEBUG = debug
    _retry(lambda: bus.write_byte_data(pzaddr, RESET, 0), "init")

    time.sleep(0.01)  #1ms delay to allow time to complete
    if (DEBUG):
        print("Debug is", DEBUG)

def cleanup():
    _retry(lambda: bus.write_byte_data(pzaddr, RESET, 0), "cleanup")
    time.sleep(0.001)
#---------------------------------------------
# Cleanup the Board (same as init)
def cleanup ():
    _retry(lambda: bus.write_byte_data(pzaddr, RESET, 0), "cleanup")

    time.sleep(0.001)   # 1ms delay to allow time to complete
#---------------------------------------------
