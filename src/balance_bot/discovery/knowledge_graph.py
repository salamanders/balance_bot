import json
import logging
from pathlib import Path
from typing import Any, Dict, Optional, Type, TypeVar

from ..utils import Vector3
from ..enums import Axis
from ..config import RobotConfig, PIDParams, ControlConfig, BatteryConfig, TunerConfig, LedConfig
from .types import Atom

logger = logging.getLogger(__name__)

DISCOVERY_STATE_FILE = Path("discovery_state.json")

T = TypeVar("T")

class DiscoveryContext:
    """
    Manages the Knowledge Graph (The "Atoms") and its persistence.
    """

    def __init__(self, filename: Path = DISCOVERY_STATE_FILE):
        self.filename = filename
        self.knowledge: Dict[Atom, Any] = {}
        self.load()

    def has_atom(self, atom: Atom) -> bool:
        return atom in self.knowledge

    def get(self, atom: Atom, default: Any = None) -> Any:
        return self.knowledge.get(atom, default)

    def update(self, new_knowledge: Dict[Atom, Any]):
        """
        Update the knowledge graph with new atoms.
        Triggers a save.
        """
        self.knowledge.update(new_knowledge)
        self.save()

    def forget_all(self):
        """Reset the brain."""
        self.knowledge = {}
        if self.filename.exists():
            self.filename.unlink()
        logger.info("Knowledge Graph reset.")

    def save(self):
        """Persist to disk."""
        data = {}
        for k, v in self.knowledge.items():
            key_str = k.name
            val_json = self._serialize(v)
            data[key_str] = val_json

        try:
            with open(self.filename, "w") as f:
                json.dump(data, f, indent=4)
            logger.debug("Discovery state saved.")
        except Exception as e:
            logger.error(f"Failed to save discovery state: {e}")

    def load(self):
        """Load from disk."""
        if not self.filename.exists():
            return

        try:
            with open(self.filename, "r") as f:
                data = json.load(f)

            for k_str, v_json in data.items():
                if k_str in Atom.__members__:
                    atom = Atom[k_str]
                    self.knowledge[atom] = self._deserialize(atom, v_json)

            logger.info(f"Loaded {len(self.knowledge)} atoms from disk.")
        except Exception as e:
            logger.error(f"Failed to load discovery state: {e}")

    def _serialize(self, obj: Any) -> Any:
        if isinstance(obj, Vector3):
            return obj._asdict()
        if isinstance(obj, Axis):
            return obj.value # "x", "y", "z"
        if isinstance(obj, dict):
             # Handle nested dicts
             return {str(k): self._serialize(v) for k, v in obj.items()}
        return obj

    def _deserialize(self, atom: Atom, data: Any) -> Any:
        # Atom-specific logic
        if atom == Atom.GRAVITY_VECTOR:
            return Vector3.from_dict(data)

        # PITCH_AXIS is a dict with strings.
        # We don't convert inner strings to Enums automatically here,
        # unless we explicitly know the structure.
        # It's safer to let build_config handle the conversion from string to Enum
        # or do it here if we want the knowledge graph to hold rich objects.

        # Let's keep knowledge graph as JSON-compatible dicts for complex types
        # except Vector3 which is special.

        # Generic handling
        if isinstance(data, dict) and "x" in data and "y" in data and "z" in data:
            return Vector3.from_dict(data)

        return data

    def build_config(self) -> RobotConfig:
        """
        Construct a RobotConfig from the discovered knowledge.
        Raises ValueError if critical atoms are missing.
        """
        if not self.has_atom(Atom.HARDWARE_BUS):
            raise ValueError("Missing Hardware Bus")

        buses = self.get(Atom.HARDWARE_BUS) # {"motor": int, "imu": int}

        c = RobotConfig(
            pid=PIDParams(),
            motor_i2c_bus=buses.get("motor"),
            imu_i2c_bus=buses.get("imu"),
        )

        # 1. Gravity (Vertical Axis)
        if self.has_atom(Atom.GRAVITY_VECTOR):
            grav: Vector3 = self.get(Atom.GRAVITY_VECTOR)
            # Find dominant axis
            mapping = {"x": grav.x, "y": grav.y, "z": grav.z}
            sorted_axes = sorted(mapping.items(), key=lambda x: abs(x[1]), reverse=True)
            axis_name = sorted_axes[0][0]
            axis_val = sorted_axes[0][1]

            c.accel_vertical_axis = Axis(axis_name)
            # If gravity (Down) reads Negative, then sensor Axis matches Down.
            # We want Vertical to be Up (Positive).
            # So if reads Negative (-1G), we Invert it (True) to get Positive (+1G).
            # If reads Positive (+1G), we Invert it (True) -> -1G? No.
            # Wait. If Z reads +1G (Up), Invert=False.
            # If Z reads -1G (Down), Invert=True -> +1G.
            c.accel_vertical_invert = axis_val < 0

        # 2. Pitch Axis (and Forward Axis)
        if self.has_atom(Atom.PITCH_AXIS):
            p_data = self.get(Atom.PITCH_AXIS)
            # p_data is {"axis": "x", "invert": bool, ...}
            if isinstance(p_data, dict):
                c.gyro_pitch_axis = Axis(p_data["axis"])
                c.gyro_pitch_invert = p_data["invert"]

                if "forward_axis" in p_data:
                    c.accel_forward_axis = Axis(p_data["forward_axis"])
                    c.accel_forward_invert = p_data["forward_invert"]
            else:
                 c.gyro_pitch_axis = Axis(p_data) # Legacy/Fallback

        # 3. Friction
        if self.has_atom(Atom.FRICTION_THRESHOLD):
            c.min_power_visible = int(self.get(Atom.FRICTION_THRESHOLD))

        # 4. Motor Mapping & Phasing & Polarity
        # Default Logic
        inv_l = False
        inv_r = False

        # A. Phasing (Alignment)
        if self.has_atom(Atom.MOTOR_PHASING):
            phasing = self.get(Atom.MOTOR_PHASING)
            if phasing.get("invert_right"):
                inv_r = not inv_r # Toggle

        # B. Polarity (Direction)
        if self.has_atom(Atom.MOTOR_POLARITY):
            polarity = self.get(Atom.MOTOR_POLARITY)
            if polarity.get("invert_both"):
                inv_l = not inv_l
                inv_r = not inv_r

        c.motor_l_invert = inv_l
        c.motor_r_invert = inv_r
        c.motor_phasing_verified = True
        c.motor_direction_verified = True

        # C. Handedness (Channel Swap)
        if self.has_atom(Atom.CHASSIS_HANDEDNESS):
            mapping = self.get(Atom.CHASSIS_HANDEDNESS)
            c.motor_l = mapping["left"]
            c.motor_r = mapping["right"]
            c.motor_channels_verified = True

        # 5. Trim
        if self.has_atom(Atom.TRIM_CALIBRATION):
            c.motor_trim = self.get(Atom.TRIM_CALIBRATION)
            c.motor_trim_verified = True

        return c
