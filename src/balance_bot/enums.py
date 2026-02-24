from enum import Enum, IntEnum, auto

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

class BotState(Enum):
    IDLE = auto()
    KICKUP = auto()
    BALANCING = auto()
    CRASHED = auto()
    FATAL_ERROR = auto()
