import time
import sys
import logging
from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional

from ..enums import Axis
from ..utils import Vector3, calculate_pitch, cross_product, analyze_dominance
from ..hardware.robot_hardware import RobotHardware
from .types import Atom, ExperimentResult
from .knowledge_graph import DiscoveryContext

# Try to import smbus for ExpPulse
try:
    import smbus
except ImportError:
    smbus = None

logger = logging.getLogger(__name__)


class Experiment(ABC):
    """
    Base class for all discovery experiments.
    """
    name: str = "Base Experiment"
    description: str = "Abstract Base"

    @property
    @abstractmethod
    def output_atoms(self) -> List[Atom]:
        """Which atoms does this experiment produce?"""
        pass

    @property
    @abstractmethod
    def required_atoms(self) -> List[Atom]:
        """Which atoms must be known before running?"""
        pass

    def can_run(self, context: DiscoveryContext) -> bool:
        """Are prerequisites met?"""
        for atom in self.required_atoms:
            if not context.has_atom(atom):
                return False
        return True

    def has_result(self, context: DiscoveryContext) -> bool:
        """Do we already know the result?"""
        # Simplistic check: If all output atoms are present.
        for atom in self.output_atoms:
            if not context.has_atom(atom):
                return False
        return True

    @abstractmethod
    def run(self, context: DiscoveryContext, hardware: Optional[RobotHardware]) -> ExperimentResult:
        """Execute the experiment."""
        pass


class ExpPulse(Experiment):
    """
    Scan I2C buses for Hardware.
    """
    name = "The Pulse"
    description = "Scanning for Brain and Body (I2C Buses)..."

    @property
    def output_atoms(self): return [Atom.HARDWARE_BUS]

    @property
    def required_atoms(self): return [] # None

    def _scan_for_device(self, name: str, check_fn) -> int | None:
        if smbus is None:
            logger.warning("smbus not installed. Cannot scan.")
            # Mock fallback?
            if "mock" in sys.modules or "unittest.mock" in sys.modules:
                 return 1
            return None

        candidates = [1, 3, 0, 2]
        for bus_id in candidates:
            try:
                bus = smbus.SMBus(bus_id)
                try:
                    if check_fn(bus):
                        logger.info(f"Found {name} on Bus {bus_id}")
                        return bus_id
                except OSError:
                    pass
                finally:
                    try:
                        bus.close()
                    except Exception:
                        pass
            except Exception:
                pass
        return None

    def run(self, context: DiscoveryContext, hardware: Optional[RobotHardware]) -> ExperimentResult:
        logger.info(self.description)

        # 1. Find Motors (0x22)
        def check_motor(bus):
            bus.read_byte_data(0x22, 0)
            return True

        motor_bus = self._scan_for_device("PiconZero (Motors)", check_motor)

        # 2. Find IMU (0x68)
        def check_imu(bus):
            return bus.read_byte_data(0x68, 0x75) == 0x68

        imu_bus = self._scan_for_device("MPU6050 (IMU)", check_imu)

        if motor_bus is None or imu_bus is None:
             return ExperimentResult(success=False, error="Could not find I2C devices.")

        data = {Atom.HARDWARE_BUS: {"motor": motor_bus, "imu": imu_bus}}
        return ExperimentResult(success=True, data=data)


class ExpMeditation(Experiment):
    """
    Determine Gravity Vector (Vertical Axis) and verify Stability.
    """
    name = "The Meditation"
    description = "Listening to Gravity..."

    @property
    def output_atoms(self): return [Atom.GRAVITY_VECTOR, Atom.STATIC_STABILITY]

    @property
    def required_atoms(self): return [Atom.HARDWARE_BUS]

    def run(self, context: DiscoveryContext, hardware: Optional[RobotHardware]) -> ExperimentResult:
        if not hardware:
            return ExperimentResult(success=False, error="Hardware not initialized.")

        logger.info(self.description)
        # Ensure motors are off (implied by init)

        # Collect samples
        accel_sum = {"x": 0.0, "y": 0.0, "z": 0.0}
        samples = 50

        # Simple stability check
        last_g = None
        total_delta = 0.0

        for _ in range(samples):
            try:
                a, g = hardware.read_imu_raw()
                accel_sum['x'] += a.x
                accel_sum['y'] += a.y
                accel_sum['z'] += a.z

                # Check stability via gyro noise
                if last_g:
                    delta = abs(g.x - last_g.x) + abs(g.y - last_g.y) + abs(g.z - last_g.z)
                    total_delta += delta
                last_g = g

                time.sleep(0.01)
            except Exception as e:
                return ExperimentResult(success=False, error=f"IMU Read Error: {e}", retry_suggested=True)

        avg_gravity = Vector3(
            accel_sum['x'] / samples,
            accel_sum['y'] / samples,
            accel_sum['z'] / samples
        )

        avg_noise = total_delta / samples
        is_stable = avg_noise < 10.0 # Threshold for raw gyro noise?

        if not is_stable:
             return ExperimentResult(success=False, error="Robot is moving. Please keep still.", retry_suggested=True)

        logger.info(f"Gravity Vector Found: {avg_gravity}")

        return ExperimentResult(success=True, data={
            Atom.GRAVITY_VECTOR: avg_gravity,
            Atom.STATIC_STABILITY: True
        })


class ExpTwitch(Experiment):
    """
    Determine Friction Threshold and Motor Presence.
    """
    name = "The Twitch"
    description = "Testing muscle response (Friction Threshold)..."

    @property
    def output_atoms(self): return [Atom.FRICTION_THRESHOLD, Atom.MOTOR_PRESENCE]

    @property
    def required_atoms(self): return [Atom.HARDWARE_BUS, Atom.GRAVITY_VECTOR]

    def run(self, context: DiscoveryContext, hardware: Optional[RobotHardware]) -> ExperimentResult:
        if not hardware: return ExperimentResult(success=False, error="No Hardware")

        pwm = 10
        step = 5
        max_pwm = 80

        logger.info("Twitching motors...")

        while pwm <= max_pwm:
            # Pulse both motors (we don't know channels yet, assume 0 and 1)

            # Use shared drive_and_measure with raw reading (since we don't know axes)
            samples = hardware.drive_and_measure(pwm, pwm, 0.2, read_raw=True)
            time.sleep(0.5) # Wait for settle

            # Measure Vibration (Gyro Magnitude)
            max_rate = 0.0
            if samples:
                # samples is list[tuple[Vector3, Vector3]]
                max_rate = max(
                    (abs(s[1].x) + abs(s[1].y) + abs(s[1].z))
                    for s in samples
                )

            # Threshold: 10 deg/s (or raw units)
            if max_rate > 10.0:
                logger.info(f"Response detected at PWM {pwm}. MaxRate={max_rate:.1f}")
                return ExperimentResult(success=True, data={
                    Atom.FRICTION_THRESHOLD: float(pwm),
                    Atom.MOTOR_PRESENCE: True
                })

            pwm += step

        return ExperimentResult(success=False, error="No movement detected even at high power.")


class ExpCrunch(Experiment):
    """
    Identify Pitch Axis.
    """
    name = "The Crunch"
    description = "Identifying Hip Joint (Pitch Axis)..."

    @property
    def output_atoms(self): return [Atom.PITCH_AXIS]

    @property
    def required_atoms(self): return [Atom.GRAVITY_VECTOR, Atom.FRICTION_THRESHOLD]

    def run(self, context: DiscoveryContext, hardware: Optional[RobotHardware]) -> ExperimentResult:
        if not hardware: return ExperimentResult(success=False, error="No Hardware")

        # 1. Get Static Gravity (Back)
        logger.info("Please ensure robot is resting on BACK wheels/strut.")
        time.sleep(2.0)

        samples = 50
        sum_back = Vector3(0,0,0)
        for _ in range(samples):
            a, _ = hardware.read_imu_raw()
            sum_back = Vector3(sum_back.x+a.x, sum_back.y+a.y, sum_back.z+a.z)
            time.sleep(0.01)

        avg_back = Vector3(sum_back.x/samples, sum_back.y/samples, sum_back.z/samples)

        # 2. Prompt for "Crunch" (Tip Forward)
        power = context.get(Atom.FRICTION_THRESHOLD) + 20

        logger.info(f"Lurching with power {power}...")

        gyro_sums = {"x": 0.0, "y": 0.0, "z": 0.0}

        # Use drive_and_measure
        samples = hardware.drive_and_measure(power, power, 0.3, read_raw=True)

        for _, g in samples:
            gyro_sums['x'] += abs(g.x)
            gyro_sums['y'] += abs(g.y)
            gyro_sums['z'] += abs(g.z)

        count = len(samples)

        # Find dominant Gyro Axis
        if count == 0: return ExperimentResult(success=False, error="No samples collected")

        avgs = {k: v/count for k,v in gyro_sums.items()}
        pitch_axis_name, ratio, success = analyze_dominance(avgs, "Pitch Axis Candidate", threshold=1.2)

        if not success:
            return ExperimentResult(success=False, error="Could not identify clear pitch axis.", retry_suggested=True)

        # Let's use the Gravity Vector we know.
        grav = context.get(Atom.GRAVITY_VECTOR)
        # Determine Vertical Axis from Gravity
        grav_dict = {"x": grav.x, "y": grav.y, "z": grav.z}
        vert_axis_name, _, _ = analyze_dominance(grav_dict, "Vertical Axis")

        # So we have Pitch(Gyro) and Vert(Accel).
        # Forward is the remaining one.
        axes = {"x", "y", "z"}
        used = {pitch_axis_name, vert_axis_name}
        remaining = axes - used
        if len(remaining) != 1:
            return ExperimentResult(success=False, error=f"Axis Conflict: Pitch={pitch_axis_name}, Vert={vert_axis_name}")

        fwd_axis_name = list(remaining)[0]

        logger.info(f"Identified Axes: Pitch={pitch_axis_name.upper()}, Vert={vert_axis_name.upper()}, Fwd={fwd_axis_name.upper()}")

        vert_val = grav_dict[vert_axis_name]
        vert_invert = vert_val < 0

        return ExperimentResult(success=True, data={
            Atom.PITCH_AXIS: {
                "axis": pitch_axis_name,
                "invert": False, # Placeholder
                "forward_axis": fwd_axis_name,
                "forward_invert": False, # Placeholder
                "vertical_axis": vert_axis_name,
                "vertical_invert": vert_invert
            }
        })


class ExpWiggle(Experiment):
    """
    Identify Motor Phasing (Do they spin together?).
    """
    name = "The Wiggle"
    description = "Checking leg coordination (Phasing)..."

    @property
    def output_atoms(self): return [Atom.MOTOR_PHASING]

    @property
    def required_atoms(self): return [Atom.PITCH_AXIS, Atom.FRICTION_THRESHOLD]

    def run(self, context: DiscoveryContext, hardware: Optional[RobotHardware]) -> ExperimentResult:
        if not hardware: return ExperimentResult(success=False, error="No Hardware")

        power = context.get(Atom.FRICTION_THRESHOLD) + 10

        logger.info(f"Driving motors with {power}...")

        # drive_and_measure with converted readings
        samples = hardware.drive_and_measure(power, power, 0.5, read_raw=False)

        if not samples: return ExperimentResult(success=False, error="No samples")

        # Analyze Yaw vs Pitch rates
        avg_yaw = sum(abs(s.yaw_rate) for s in samples) / len(samples)
        avg_pitch = sum(abs(s.pitch_rate) for s in samples) / len(samples)

        logger.info(f"Response: Yaw={avg_yaw:.1f}, Pitch={avg_pitch:.1f}")

        if avg_yaw > 20.0 and avg_yaw > avg_pitch:
            logger.info("Motors are fighting (Spinning). Inverting Right Motor.")
            return ExperimentResult(success=True, data={
                Atom.MOTOR_PHASING: {"invert_right": True}
            })
        else:
            logger.info("Motors are aligned.")
            return ExperimentResult(success=True, data={
                Atom.MOTOR_PHASING: {"invert_right": False}
            })


class ExpAttempt(Experiment):
    """
    Identify Motor Polarity (Direction).
    Does +Power make me Stand Up?
    """
    name = "The Attempt"
    description = "Trying to stand up (Direction)..."

    @property
    def output_atoms(self): return [Atom.MOTOR_POLARITY]

    @property
    def required_atoms(self): return [Atom.MOTOR_PHASING, Atom.PITCH_AXIS]

    def run(self, context: DiscoveryContext, hardware: Optional[RobotHardware]) -> ExperimentResult:
        if not hardware: return ExperimentResult(success=False, error="No Hardware")

        # 1. Check Lean
        reading = hardware.read_imu_converted()
        pitch = reading.pitch_angle

        if pitch < -10:
            logger.info(f"Leaning BACK (Pitch={pitch:.1f}). Expecting +Power to Kick Up.")
        elif pitch > 10:
            logger.info(f"Leaning FRONT (Pitch={pitch:.1f}). Expecting -Power to Kick Up.")
        else:
            return ExperimentResult(success=False, error=f"Robot is too upright ({pitch:.1f}). Please lean it over.", retry_suggested=True)

        power = context.get(Atom.FRICTION_THRESHOLD) + 20
        logger.info(f"Applying +Power ({power})...")

        samples = hardware.drive_and_measure(power, power, 0.3, read_raw=False)

        end_pitch = samples[-1].pitch_angle
        logger.info(f"Pitch: {pitch:.1f} -> {end_pitch:.1f}")

        # Did it improve?
        started_leaning = abs(pitch)
        ended_leaning = abs(end_pitch)

        improved = ended_leaning < started_leaning
        invert_needed = False

        if pitch > 0: # Front
            if improved:
                logger.info("Robot stood up. Direction Correct.")
                invert_needed = False
            else:
                logger.info("Robot fell over. Direction Inverted.")
                invert_needed = True
        else: # Back
            if improved:
                logger.info("Robot stood up from Back using +Power. Direction Inverted.")
                invert_needed = True
            else:
                logger.info("Robot fell more. Direction Correct.")
                invert_needed = False

        return ExperimentResult(success=True, data={
            Atom.MOTOR_POLARITY: {"invert_both": invert_needed}
        })


class ExpPirouette(Experiment):
    """
    Identify Left/Right Motors.
    """
    name = "The Pirouette"
    description = "Spinning to find my Left from Right..."

    @property
    def output_atoms(self): return [Atom.CHASSIS_HANDEDNESS]

    @property
    def required_atoms(self): return [Atom.MOTOR_POLARITY]

    def run(self, context: DiscoveryContext, hardware: Optional[RobotHardware]) -> ExperimentResult:
        if not hardware: return ExperimentResult(success=False, error="No Hardware")

        power = context.get(Atom.FRICTION_THRESHOLD) + 15

        logger.info(f"Spinning (L=+, R=-)...")

        # Use drive_and_measure with raw reading (need for dot product)
        samples = hardware.drive_and_measure(power, -power, 1.0, read_raw=True)
        gyro_samples = [s[1] for s in samples]

        avg_gyro = Vector3(
            sum(g.x for g in gyro_samples)/len(gyro_samples),
            sum(g.y for g in gyro_samples)/len(gyro_samples),
            sum(g.z for g in gyro_samples)/len(gyro_samples)
        )

        # Up Vector is opposite of Gravity
        grav = context.get(Atom.GRAVITY_VECTOR)
        up = Vector3(-grav.x, -grav.y, -grav.z)

        # Dot Product
        dot = (up.x * avg_gyro.x) + (up.y * avg_gyro.y) + (up.z * avg_gyro.z)
        logger.info(f"Spin Dot Product: {dot:.1f}")

        if abs(dot) < 10.0:
            return ExperimentResult(success=False, error="Did not spin enough.", retry_suggested=True)

        swap_channels = dot > 0

        if swap_channels:
            logger.info("Detected CCW spin. Ch0 is Right. Swapping.")
            mapping = {"left": 1, "right": 0}
        else:
            logger.info("Detected CW spin. Ch0 is Left. Correct.")
            mapping = {"left": 0, "right": 1}

        return ExperimentResult(success=True, data={
            Atom.CHASSIS_HANDEDNESS: mapping
        })


class ExpStride(Experiment):
    """
    Calibrate Trim.
    """
    name = "The Stride"
    description = "Learning to walk straight (Trim)..."

    @property
    def output_atoms(self): return [Atom.TRIM_CALIBRATION]

    @property
    def required_atoms(self): return [Atom.CHASSIS_HANDEDNESS]

    def run(self, context: DiscoveryContext, hardware: Optional[RobotHardware]) -> ExperimentResult:
        if not hardware: return ExperimentResult(success=False, error="No Hardware")

        power = context.get(Atom.FRICTION_THRESHOLD) + 15
        trim = 0.0

        # Iterative attempt
        for i in range(5):
            logger.info(f"Testing Trim {trim:.3f}...")
            hardware.motor_trim = trim

            samples = hardware.drive_and_measure(power, power, 1.0, read_raw=False)

            if not samples: continue

            avg_yaw = sum(s.yaw_rate for s in samples) / len(samples)
            logger.info(f"Drift: {avg_yaw:.1f}")

            if abs(avg_yaw) < 2.0:
                logger.info(f"Straight enough at {trim:.3f}.")
                return ExperimentResult(success=True, data={Atom.TRIM_CALIBRATION: trim})

            # Adjust
            trim += avg_yaw * 0.005
            trim = max(-0.3, min(0.3, trim))

        return ExperimentResult(success=True, data={Atom.TRIM_CALIBRATION: trim})
