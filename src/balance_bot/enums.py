from enum import Enum, IntEnum

class Orientation(str, Enum):
    FRONT = "front"
    BACK = "back"

class MotorSide(str, Enum):
    LEFT = "left"
    RIGHT = "right"

class Axis(str, Enum):
    X = "x"
    Y = "y"
    Z = "z"

class Direction(IntEnum):
    FORWARD = 1
    BACKWARD = -1
