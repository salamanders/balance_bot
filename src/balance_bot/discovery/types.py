from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Any, Dict

class Atom(Enum):
    """
    Discrete units of knowledge ("Atoms") in the Knowledge Graph.
    """
    HARDWARE_BUS = auto()       # I2C Bus IDs (Motor, IMU)
    GRAVITY_VECTOR = auto()     # Which raw axis is Down?
    STATIC_STABILITY = auto()   # Is the robot still?
    MOTOR_PRESENCE = auto()     # Do we have motors?
    FRICTION_THRESHOLD = auto() # PWM required to move
    PITCH_AXIS = auto()         # Which axis is the wheel axle?
    MOTOR_PHASING = auto()      # Do motors spin together?
    MOTOR_POLARITY = auto()     # Does +Power make me stand up?
    CHASSIS_HANDEDNESS = auto() # Which motor is Left vs Right?
    TRIM_CALIBRATION = auto()   # Straight line trim

@dataclass
class ExperimentResult:
    """
    The outcome of an experiment.
    """
    success: bool
    data: Dict[Atom, Any] = field(default_factory=dict)
    error: str | None = None
    retry_suggested: bool = False
