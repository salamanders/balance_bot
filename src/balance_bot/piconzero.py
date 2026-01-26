# Python library for 4tronix Picon Zero
# Note that all I2C accesses are wrapped in try clauses with repeats

import time
import smbus2 as smbus

try:
    bus = smbus.SMBus(1) # For revision 1 Raspberry Pi, change to bus = smbus.SMBus(0)
except (FileNotFoundError, PermissionError, OSError):
    bus = None

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
#---------------------------------------------

#---------------------------------------------
# General variables
DEBUG = False
RETRIES = 10   # max number of retries for I2C calls
#---------------------------------------------

#---------------------------------------------
# Get Version and Revision info
def getRevision():
    if bus is None:
        return [0, 0]
    for i in range(RETRIES):
        try:
            rval = bus.read_word_data (pzaddr, 0)
            return [rval // 256, rval % 256]
        except Exception:
            if (DEBUG):
                print("Error in getRevision(), retrying")
#---------------------------------------------


#---------------------------------------------
# motor must be in range 0..1
# value must be in range -128 - +127
# values of -127, -128, +127 are treated as always ON,, no PWM
def setMotor (motor, value):
    if bus is None:
        return
    if (motor>=0 and motor<=1 and value>=-128 and value<128):
        for i in range(RETRIES):
            try:
                bus.write_byte_data (pzaddr, motor, value)
                break
            except Exception:
                if (DEBUG):
                    print("Error in setMotor(), retrying")

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
    if bus is None:
        return 0
    if (channel>=0 and channel <=3):
        for i in range(RETRIES):
            try:
                return bus.read_word_data (pzaddr, channel + 1)
            except Exception:
                if (DEBUG):
                    print("Error in readChannel(), retrying")

#---------------------------------------------

#---------------------------------------------
# Set configuration of selected output
# 0: On/Off, 1: PWM, 2: Servo, 3: WS2812B
def setOutputConfig (output, value):
    if bus is None:
        return
    if (output>=0 and output<=5 and value>=0 and value<=3):
        for i in range(RETRIES):
            try:
                bus.write_byte_data (pzaddr, OUTCFG0 + output, value)
                break
            except Exception:
                if (DEBUG):
                    print("Error in setOutputConfig(), retrying")
#---------------------------------------------

#---------------------------------------------
# Set configuration of selected input channel
# 0: Digital, 1: Analog
def setInputConfig (channel, value, pullup = False):
    if bus is None:
        return
    if (channel>=0 and channel <=3 and value>=0 and value<=3):
        if (value==0 and pullup):
            value = 128
        for i in range(RETRIES):
            try:
                bus.write_byte_data (pzaddr, INCFG0 + channel, value)
                break
            except Exception:
                if (DEBUG):
                    print("Error in setInputConfig(), retrying")
#---------------------------------------------

#---------------------------------------------
# Set output data for selected output channel
# Mode  Name    Type    Values
# 0     On/Off  Byte    0 is OFF, 1 is ON
# 1     PWM     Byte    0 to 100 percentage of ON time
# 2     Servo   Byte    -100 to + 100 Position in degrees
# 3     WS2812B 4 Bytes 0:Pixel ID, 1:Red, 2:Green, 3:Blue
def setOutput (channel, value):
    if bus is None:
        return
    if (channel>=0 and channel<=5):
        for i in range(RETRIES):
            try:
                bus.write_byte_data (pzaddr, OUTPUT0 + channel, value)
                break
            except Exception:
                if (DEBUG):
                    print("Error in setOutput(), retrying")
#---------------------------------------------

#---------------------------------------------
# Set the colour of an individual pixel (always output 5)
def setPixel (Pixel, Red, Green, Blue, Update=True):
    if bus is None:
        return
    pixelData = [Pixel, Red, Green, Blue]
    for i in range(RETRIES):
        try:
            bus.write_i2c_block_data (pzaddr, Update, pixelData)
            break
        except Exception:
            if (DEBUG):
                print("Error in setPixel(), retrying")

def setAllPixels (Red, Green, Blue, Update=True):
    if bus is None:
        return
    pixelData = [100, Red, Green, Blue]
    for i in range(RETRIES):
        try:
            bus.write_i2c_block_data (pzaddr, Update, pixelData)
            break
        except Exception:
            if (DEBUG):
                print("Error in setAllPixels(), retrying")

def updatePixels ():
    if bus is None:
        return
    for i in range(RETRIES):
        try:
            bus.write_byte_data (pzaddr, UPDATENOW, 0)
            break
        except Exception:
            if (DEBUG):
                print("Error in updatePixels(), retrying")

#---------------------------------------------

#---------------------------------------------
# Set the overall brightness of pixel array
def setBrightness (brightness):
    if bus is None:
        return
    for i in range(RETRIES):
        try:
            bus.write_byte_data (pzaddr, SETBRIGHT, brightness)
            break
        except Exception:
            if (DEBUG):
                print("Error in setBrightness(), retrying")
#---------------------------------------------

#---------------------------------------------
# Initialise the Board (same as cleanup)
def init (debug=False):
    global DEBUG
    DEBUG = debug
    if bus is None:
        if DEBUG:
            print("Simulator Mode: Bus is None")
        return
    for i in range(RETRIES):
        try:
            bus.write_byte_data (pzaddr, RESET, 0)
            break
        except Exception:
            if (DEBUG):
                print("Error in init(), retrying")
    time.sleep(0.01)  #1ms delay to allow time to complete
    if (DEBUG):
        print("Debug is", DEBUG)
#---------------------------------------------

#---------------------------------------------
# Cleanup the Board (same as init)
def cleanup ():
    if bus is None:
        return
    for i in range(RETRIES):
        try:
            bus.write_byte_data (pzaddr, RESET, 0)
            break
        except Exception:
            if (DEBUG):
                print("Error in cleanup(), retrying")
    time.sleep(0.001)   # 1ms delay to allow time to complete
#---------------------------------------------
