# Python library for 4tronix Picon Zero
# Note that all I2C accesses are wrapped in try clauses with repeats

from __future__ import print_function
import smbus, time

bus = smbus.SMBus(1) # For revision 1 Raspberry Pi, change to bus = smbus.SMBus(0)

pzaddr = 0x22 # I2C address of Picon Zero

#---------------------------------------------
# Definitions of Commands to Picon Zero
MOTORA = 0
OUTCFG0 = 2
OUTPUT0 = 8
INCFG0 = 14
SETBRIGHT = 18
UPDATENOW = 19
RESET = 20
INPERIOD0 = 21
#---------------------------------------------

#---------------------------------------------
# General variables
DEBUG = False
RETRIES = 10   # max number of retries for I2C calls
#---------------------------------------------

#---------------------------------------------
# Get Version and Revision info
def getRevision():
    for i in range(RETRIES):
        try:
            rval = bus.read_word_data (pzaddr, 0)
            return [rval//256, rval%256]
        except:
            if (DEBUG):
                print("Error in getRevision(), retrying")
    raise OSError("PiconZero getRevision() failed after retries")
#---------------------------------------------


#---------------------------------------------
# motor must be in range 0..1
# value must be in range -128 - +127
# values of -127, -128, +127 are treated as always ON,, no PWM
def setMotor (motor, value):
    if (motor>=0 and motor<=1 and value>=-128 and value<128):
        for i in range(RETRIES):
            try:
                bus.write_byte_data (pzaddr, motor, value)
                return
            except:
                if (DEBUG):
                    print("Error in setMotor(), retrying")
        raise OSError("PiconZero setMotor() failed after retries")

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
        for i in range(RETRIES):
            try:
                return bus.read_word_data (pzaddr, channel + 1)
            except:
                if (DEBUG):
                    print("Error in readChannel(), retrying")
    raise OSError("PiconZero readInput() failed after retries")

#---------------------------------------------

#---------------------------------------------
# Set configuration of selected output
# 0: On/Off, 1: PWM, 2: Servo, 3: WS2812B
def setOutputConfig (output, value):
    if (output>=0 and output<=5 and value>=0 and value<=3):
        for i in range(RETRIES):
            try:
                bus.write_byte_data (pzaddr, OUTCFG0 + output, value)
                return
            except:
                if (DEBUG):
                    print("Error in setOutputConfig(), retrying")
        raise OSError("PiconZero setOutputConfig() failed after retries")
#---------------------------------------------

#---------------------------------------------
# Set configuration of selected input channel
# 0: Digital, 1: Analog, 2: DS18B20, 4: DutyCycle 5: Pulse Width
def setInputConfig (channel, value, pullup = False, period = 2000):
    if (channel >= 0 and channel <= 3 and value >= 0 and value <= 5):
        if (value == 0 and pullup == True):
            value = 128
        for i in range(RETRIES):
            try:
                bus.write_byte_data (pzaddr, INCFG0 + channel, value)
                if (value == 4 or value == 5):
                    bus.write_word_data (pzaddr, INPERIOD0 + channel, period)
                return
            except:
                if (DEBUG):
                    print("Error in setInputConfig(), retrying")
        raise OSError("PiconZero setInputConfig() failed after retries")
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
        for i in range(RETRIES):
            try:
                bus.write_byte_data (pzaddr, OUTPUT0 + channel, value)
                return
            except:
                if (DEBUG):
                    print("Error in setOutput(), retrying")
        raise OSError("PiconZero setOutput() failed after retries")
#---------------------------------------------

#---------------------------------------------
# Set the colour of an individual pixel (always output 5)
def setPixel (Pixel, Red, Green, Blue, Update=True):
    pixelData = [Pixel, Red, Green, Blue]
    for i in range(RETRIES):
        try:
            bus.write_i2c_block_data (pzaddr, Update, pixelData)
            return
        except:
            if (DEBUG):
                print("Error in setPixel(), retrying")
    raise OSError("PiconZero setPixel() failed after retries")

def setAllPixels (Red, Green, Blue, Update=True):
    pixelData = [100, Red, Green, Blue]
    for i in range(RETRIES):
        try:
            bus.write_i2c_block_data (pzaddr, Update, pixelData)
            return
        except:
            if (DEBUG):
                print("Error in setAllPixels(), retrying")
    raise OSError("PiconZero setAllPixels() failed after retries")

def updatePixels ():
    for i in range(RETRIES):
        try:
            bus.write_byte_data (pzaddr, UPDATENOW, 0)
            return
        except:
            if (DEBUG):
                print("Error in updatePixels(), retrying")
    raise OSError("PiconZero updatePixels() failed after retries")

#---------------------------------------------

#---------------------------------------------
# Set the overall brightness of pixel array
def setBrightness (brightness):
    for i in range(RETRIES):
        try:
            bus.write_byte_data (pzaddr, SETBRIGHT, brightness)
            return
        except:
            if (DEBUG):
                print("Error in setBrightness(), retrying")
    raise OSError("PiconZero setBrightness() failed after retries")
#---------------------------------------------

#---------------------------------------------
# Initialise the Board (same as cleanup)
def init (debug=False):
    DEBUG = debug
    for i in range(RETRIES):
        try:
            bus.write_byte_data (pzaddr, RESET, 0)
            break
        except:
            if (DEBUG):
                print("Error in init(), retrying")
    else:
        # Loop failed
        raise OSError("PiconZero init() failed after retries")

    time.sleep(0.01)  #1ms delay to allow time to complete
    if (DEBUG):
        print("Debug is", DEBUG)
#---------------------------------------------

#---------------------------------------------
# Cleanup the Board (same as init)
def cleanup ():
    for i in range(RETRIES):
        try:
            bus.write_byte_data (pzaddr, RESET, 0)
            break
        except:
            if (DEBUG):
                print("Error in cleanup(), retrying")
    else:
        # Loop failed
        raise OSError("PiconZero cleanup() failed after retries")

    time.sleep(0.001)   # 1ms delay to allow time to complete
#---------------------------------------------