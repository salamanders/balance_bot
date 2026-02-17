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
        # Ideally, we check if the SPECIFIC result from this exp is known.
        # But for now, presence of atoms is enough.
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
            logger.warning("[SENSORY] I cannot reach my nervous system (smbus not installed).")
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
                        logger.info(f"[SENSORY] I feel a spark of life from {name} on Bus {bus_id}.")
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
        logger.info(f"[THOUGHT] {self.description}")

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
             logger.warning("[THOUGHT] I feel incomplete. I cannot find my limbs or inner ear.")
             # In Mock Mode, we might not find them but we want to proceed?
             # If hardware was initialized with defaults, we might be okay.
             # But here we are discovering.
             # Assume Mock Mode returns 1 if mocked.
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

        logger.info(f"[THOUGHT] {self.description}")
        # Ensure motors are off (implied by init)

        # Collect samples
        accel_sum = {"x": 0.0, "y": 0.0, "z": 0.0}
        samples = 50

        # Simple stability check
        # We need raw data because we don't know axes yet.
        # RobotHardware read_imu_raw() works.

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
        # Ideally we check if it's "still".

        if not is_stable:
             logger.warning("[SENSORY] The world is spinning too fast. I must be still to meditate.")
             return ExperimentResult(success=False, error="Robot is moving. Please keep still.", retry_suggested=True)

        logger.info(f"[SENSORY] I can feel 'Down'. It is pulling along {avg_gravity}.")

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

        logger.info("[THOUGHT] I will twitch my muscles to see when I move.")

        while pwm <= max_pwm:
            # Pulse both motors (we don't know channels yet, assume 0 and 1)
            # RobotHardware maps motor_l/r. Default 0/1.
            # We want to twitch "Something".

            # Pulse
            hardware.set_motors(pwm, pwm)

            # Measure Vibration (Gyro Magnitude)
            max_rate = 0.0
            start = time.time()
            while time.time() - start < 0.2:
                _, g = hardware.read_imu_raw()
                rate = abs(g.x) + abs(g.y) + abs(g.z)
                max_rate = max(max_rate, rate)
                time.sleep(0.01)

            hardware.stop()
            time.sleep(0.5) # Wait for settle

            # Threshold: 10 deg/s (or raw units)
            # Assuming raw units are consistent.
            if max_rate > 10.0:
                logger.info(f"[SENSORY] I felt a shiver at power {pwm}. My legs are connected.")
                return ExperimentResult(success=True, data={
                    Atom.FRICTION_THRESHOLD: float(pwm),
                    Atom.MOTOR_PRESENCE: True
                })

            pwm += step

        logger.warning("[THOUGHT] I am pushing as hard as I can, but I feel nothing.")
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
        # We assume we are starting from a stable position (Back or Front).
        # We need to measure "State A", then "State B".
        # State A: Current Resting Position.
        # State B: Tipped Forward Position.

        # Wait for stability
        logger.info("[THOUGHT] I need to be resting comfortably on my back.")
        time.sleep(2.0)

        samples = 50
        sum_back = Vector3(0,0,0)
        for _ in range(samples):
            a, _ = hardware.read_imu_raw()
            sum_back = Vector3(sum_back.x+a.x, sum_back.y+a.y, sum_back.z+a.z)
            time.sleep(0.01)

        avg_back = Vector3(sum_back.x/samples, sum_back.y/samples, sum_back.z/samples)

        # 2. Prompt for "Crunch" (Tip Forward)
        # Since we want to be autonomous, can we do this via motor lurch?
        # "The Crunch: Send a Lurch command... monitor which axis had highest velocity?"
        # The design doc says: "Send a 'Lurch' command... Observation: Which Gyro axis had highest velocity?"
        # That identifies the Gyro Pitch Axis.
        # It does NOT necessarily identify the Accel Pitch Axis (Forward).
        # But Gyro Pitch is the main one.

        # Let's try the dynamic approach (Design Doc Method D).
        power = context.get(Atom.FRICTION_THRESHOLD) + 20

        logger.info(f"[THOUGHT] I will do a crunch with power {power} to find my hips.")

        gyro_sums = {"x": 0.0, "y": 0.0, "z": 0.0}
        gyro_vectors = []
        count = 0

        hardware.set_motors(power, power)
        start = time.time()
        while time.time() - start < 0.3:
            _, g = hardware.read_imu_raw()
            gyro_sums['x'] += abs(g.x)
            gyro_sums['y'] += abs(g.y)
            gyro_sums['z'] += abs(g.z)
            gyro_vectors.append(g)
            count += 1
            time.sleep(0.01)
        hardware.stop()

        # Settle
        time.sleep(0.5)

        # Measure End State (Gravity changed?)
        samples = 20
        sum_end = Vector3(0,0,0)
        for _ in range(samples):
             a, _ = hardware.read_imu_raw()
             sum_end = Vector3(sum_end.x+a.x, sum_end.y+a.y, sum_end.z+a.z)
             time.sleep(0.01)
        avg_end = Vector3(sum_end.x/samples, sum_end.y/samples, sum_end.z/samples)


        # Find dominant Gyro Axis
        if count == 0: return ExperimentResult(success=False, error="No samples collected")

        avgs = {k: v/count for k,v in gyro_sums.items()}
        pitch_axis_name, ratio, success = analyze_dominance(avgs, "Pitch Axis Candidate", threshold=1.2)

        if not success:
            logger.warning("[THOUGHT] My movement was confusing. I couldn't isolate my hips.")
            return ExperimentResult(success=False, error="Could not identify clear pitch axis.", retry_suggested=True)

        # Determine Polarity (Invert) via Sensor Fusion
        # Calculate Delta Accel (Observed Acceleration Change)
        delta_accel = Vector3(
             avg_end.x - avg_back.x,
             avg_end.y - avg_back.y,
             avg_end.z - avg_back.z
        )

        # Calculate Avg Gyro
        avg_gyro_vec = Vector3(
             sum(g.x for g in gyro_vectors)/len(gyro_vectors),
             sum(g.y for g in gyro_vectors)/len(gyro_vectors),
             sum(g.z for g in gyro_vectors)/len(gyro_vectors)
        )

        # Gravity Vector (known)
        grav = context.get(Atom.GRAVITY_VECTOR)

        # Cross Product: Gravity x Delta_Accel gives the rotation vector observed by Accel
        # (Assuming Delta_Accel is due to rotation relative to Gravity)
        observed_rot = cross_product(grav, delta_accel)

        # Compare signs on the Pitch Axis
        obs_val = getattr(observed_rot, pitch_axis_name)
        gyro_val = getattr(avg_gyro_vec, pitch_axis_name)

        # If they have same sign, Gyro is Correct.
        # If opposite, Gyro is Inverted.
        # NOTE: This assumes Standard Right Hand Rule for both sensors.
        # If Gyro says +5, and Accel says we rotated +5, then Invert=False.

        # Check for zero (unlikely during motion)
        if abs(obs_val) < 1.0 or abs(gyro_val) < 1.0:
             logger.warning("[THOUGHT] I moved, but the sensors disagree or are too quiet.")
             # Fallback to False
             pitch_invert = False
        else:
             pitch_invert = (obs_val * gyro_val) < 0 # If product is negative, signs oppose -> Invert
             if pitch_invert:
                  logger.info("[SENSORY] My inner ear is wired backwards. I will compensate.")
             else:
                  logger.info("[SENSORY] My senses agree on which way is up.")

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

        logger.info(f"[LEARNED] My Body: Hip={pitch_axis_name.upper()}, Spine={vert_axis_name.upper()}, Face={fwd_axis_name.upper()}")

        # Construct result data
        # Vert Invert logic
        vert_val = grav_dict[vert_axis_name]
        vert_invert = vert_val < 0

        # Accel Forward Invert
        # Default to False (Arbitrary "Forward")
        fwd_invert = False

        return ExperimentResult(success=True, data={
            Atom.PITCH_AXIS: {
                "axis": pitch_axis_name,
                "invert": pitch_invert,
                "forward_axis": fwd_axis_name,
                "forward_invert": fwd_invert,
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

        logger.info(f"[THOUGHT] I'm going to wiggle my legs to see if they work together.")

        samples = []
        hardware.set_motors(power, power)
        start = time.time()
        while time.time() - start < 0.5:
            samples.append(hardware.read_imu_converted()) # Uses partially configured HW
            time.sleep(0.01)
        hardware.stop()

        if not samples: return ExperimentResult(success=False, error="No samples")

        # Analyze Yaw vs Pitch rates
        avg_yaw = sum(abs(s.yaw_rate) for s in samples) / len(samples)
        avg_pitch = sum(abs(s.pitch_rate) for s in samples) / len(samples)

        logger.info(f"[SENSORY] Motion detected: Twist={avg_yaw:.1f}, Nod={avg_pitch:.1f}")

        # If Spinning, Yaw is high. If Driving, Pitch is high (or at least Yaw is low).
        if avg_yaw > 20.0 and avg_yaw > avg_pitch:
            # Spinning -> Motors are fighting.
            # One needs inversion.
            logger.info("[THOUGHT] My legs are fighting each other! I'll swap one.")
            return ExperimentResult(success=True, data={
                Atom.MOTOR_PHASING: {"invert_right": True}
            })
        else:
            logger.info("[THOUGHT] My legs are moving in unison.")
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
            logger.info(f"[THOUGHT] I am leaning BACK ({pitch:.1f} deg).")
        elif pitch > 10:
            logger.info(f"[THOUGHT] I am leaning FORWARD ({pitch:.1f} deg).")
        else:
            return ExperimentResult(success=False, error=f"I am too upright ({pitch:.1f}). Please tip me over.", retry_suggested=True)

        power = context.get(Atom.FRICTION_THRESHOLD) + 20

        logger.info(f"[THOUGHT] I am going to try to stand up using power {power}...")
        hardware.set_motors(power, power)

        samples = []
        start = time.time()
        while time.time() - start < 0.3:
            samples.append(hardware.read_imu_converted())
            time.sleep(0.01)
        hardware.stop()

        end_pitch = samples[-1].pitch_angle
        logger.info(f"[SENSORY] I moved from {pitch:.1f} to {end_pitch:.1f}.")

        # Did it improve?
        started_leaning = abs(pitch)
        ended_leaning = abs(end_pitch)

        improved = ended_leaning < started_leaning
        invert_needed = False

        if pitch > 0: # Front
            if improved:
                logger.info("[LEARNED] Pushing forward makes me stand up. My instincts are correct.")
                invert_needed = False
            else:
                logger.info("[LEARNED] Pushing forward made me fall. I need to reverse my instincts.")
                invert_needed = True
        else: # Back
            if improved:
                logger.info("[LEARNED] Pushing forward made me stand up (reversed). Reversing instincts.")
                invert_needed = True
            else:
                logger.info("[LEARNED] Pushing forward made me fall more. Instincts are correct.")
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

        # Spin: Motor 0 Fwd, Motor 1 Back.
        power = context.get(Atom.FRICTION_THRESHOLD) + 15

        logger.info(f"[THOUGHT] I will spin to figure out which leg is which.")

        hardware.set_motors(power, -power)

        gyro_samples = []
        start = time.time()
        while time.time() - start < 1.0:
            # We need RAW gyro to check against Gravity Vector
            _, g = hardware.read_imu_raw()
            gyro_samples.append(g)
            time.sleep(0.01)
        hardware.stop()

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
        logger.info(f"[SENSORY] Spin Value: {dot:.1f}")

        if abs(dot) < 10.0:
            return ExperimentResult(success=False, error="Did not spin enough.", retry_suggested=True)

        swap_channels = dot > 0

        if swap_channels:
            logger.info("[LEARNED] I was spinning the wrong way. Swapping left and right.")
            mapping = {"left": 1, "right": 0}
        else:
            logger.info("[LEARNED] I spun the way I expected. Left and Right are correct.")
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

        logger.info(f"[THOUGHT] I will practice walking straight.")

        # Iterative attempt
        for i in range(5):
            # hardware.motor_trim = trim (It's public).
            hardware.motor_trim = trim

            hardware.set_motors(power, power)
            samples = []
            start = time.time()
            while time.time() - start < 1.0:
                samples.append(hardware.read_imu_converted())
                time.sleep(0.01)
            hardware.stop()

            if not samples: continue

            avg_yaw = sum(s.yaw_rate for s in samples) / len(samples)
            logger.info(f"[SENSORY] Drift at trim {trim:.3f}: {avg_yaw:.1f}")

            if abs(avg_yaw) < 2.0:
                logger.info(f"[LEARNED] I can walk straight with trim {trim:.3f}.")
                return ExperimentResult(success=True, data={Atom.TRIM_CALIBRATION: trim})

            # Adjust
            trim += avg_yaw * 0.005
            trim = max(-0.3, min(0.3, trim))

        return ExperimentResult(success=True, data={Atom.TRIM_CALIBRATION: trim})
