import time
import sys
import smbus
from typing import Callable
from .diagnostics import run_diagnostics
from .config import RobotConfig
from .hardware.robot_hardware import RobotHardware, IMUReading
from .enums import Axis
from .utils import analyze_dominance, cross_product, Vector3


class MeasureResult:
    """Helper to analyze a sequence of IMU readings."""
    def __init__(self, samples: list[IMUReading]):
        self.samples = samples

    def __bool__(self):
        return bool(self.samples)

    @property
    def avg_yaw_rate(self) -> float:
        if not self.samples: return 0.0
        return sum(s.yaw_rate for s in self.samples) / len(self.samples)

    @property
    def avg_yaw_rate_abs(self) -> float:
        if not self.samples: return 0.0
        return sum(abs(s.yaw_rate) for s in self.samples) / len(self.samples)

    @property
    def avg_accel_raw(self) -> Vector3:
        if not self.samples: return Vector3.zero()
        valid = [s.accel_raw for s in self.samples if s.accel_raw]
        if not valid: return Vector3.zero()
        return sum(valid, Vector3.zero()) / len(valid)

    @property
    def avg_gyro_raw(self) -> Vector3:
        if not self.samples: return Vector3.zero()
        valid = [s.gyro_raw for s in self.samples if s.gyro_raw]
        if not valid: return Vector3.zero()
        return sum(valid, Vector3.zero()) / len(valid)

    @property
    def max_rate(self) -> float:
        if not self.samples: return 0.0
        return max(abs(s.pitch_rate) + abs(s.yaw_rate) + abs(s.roll_rate) for s in self.samples)

    @property
    def pitch_start(self) -> float:
        return self.samples[0].pitch_angle if self.samples else 0.0

    @property
    def pitch_end(self) -> float:
        return self.samples[-1].pitch_angle if self.samples else 0.0


class WiringCheck:
    """
    Zero-Knowledge "Self-Discovery" Wiring Check.
    Implements a state-machine that incrementally discovers robot configuration.
    See LEARN.md for specification.
    """

    def __init__(self):
        self.config = RobotConfig.load()
        self.hw = None

    def init_hw(self):
        """
        Initialize hardware with current known config.
        If config is incomplete, fills in safe defaults for discovery.
        """
        if self.hw:
            try:
                self.hw.stop()
                self.hw.cleanup()
            except Exception:
                pass

        if self.config.motor_i2c_bus is None or self.config.imu_i2c_bus is None:
            return

        self.hw = RobotHardware(
            motor_l=self.config.motor_l,
            motor_r=self.config.motor_r,
            invert_l=self.config.motor_l_invert,
            invert_r=self.config.motor_r_invert,
            gyro_axis=self.config.gyro_pitch_axis or Axis.X,
            gyro_invert=self.config.gyro_pitch_invert,
            gyro_yaw_axis=self.config.gyro_yaw_axis or Axis.Z,
            gyro_yaw_invert=self.config.gyro_yaw_invert,
            gyro_roll_axis=self.config.gyro_roll_axis or Axis.Y,
            gyro_roll_invert=self.config.gyro_roll_invert,
            accel_vertical_axis=self.config.accel_vertical_axis or Axis.Z,
            accel_vertical_invert=self.config.accel_vertical_invert,
            accel_forward_axis=self.config.accel_forward_axis or Axis.Y,
            accel_forward_invert=self.config.accel_forward_invert,
            motor_i2c_bus=self.config.motor_i2c_bus,
            imu_i2c_bus=self.config.imu_i2c_bus,
            motor_trim=self.config.motor_trim,
        )
        self.hw.init()

    def cleanup(self):
        if self.hw:
            self.hw.stop()
            self.hw.cleanup()

    def wait_for_stability(self, duration: float = 2.0, threshold: float = 2.0):
        """
        Wait for the robot to be stable (gyro rates low) for a duration.
        """
        print(f"Waiting for stability (rates < {threshold} deg/s) for {duration}s...")
        start_stable_time = None
        last_log = 0.0

        while True:
            try:
                if not self.hw: self.init_hw()
                reading = self.hw.read_imu_converted()
                rate = abs(reading.pitch_rate) + abs(reading.yaw_rate) + abs(reading.roll_rate)

                if rate < threshold:
                    if start_stable_time is None:
                        start_stable_time = time.time()
                    elif time.time() - start_stable_time >= duration:
                        print("  [STABLE] Robot is still.")
                        return
                else:
                    if start_stable_time is not None:
                        start_stable_time = None
                        if time.time() - last_log > 1.0:
                            print(f"  [MOVING] Rate {rate:.1f} > {threshold}. Waiting...")
                            last_log = time.time()
                time.sleep(0.05)
            except KeyboardInterrupt:
                raise
            except Exception as e:
                print(f"  [Error reading IMU] {e}")
                time.sleep(0.1)

    def collect_data(self, left_power: float, right_power: float, duration: float, sample_interval: float = 0.01) -> MeasureResult:
        """
        Drive motors for a duration and collect IMU readings.
        """
        samples = []
        try:
            if left_power != 0 or right_power != 0:
                self.hw.set_motors(left_power, right_power)

            # Allow motor state to propagate?
            # Actually, if power is 0, we assume 'measure' mode.
            # But we should ensure motors are set if requested.

            start = time.time()
            while time.time() - start < duration:
                samples.append(self.hw.read_imu_converted())
                time.sleep(sample_interval)
        finally:
            self.hw.stop()
        return MeasureResult(samples)

    # Legacy alias
    drive_and_measure = collect_data

    def run(self):
        """Main Knowledge Dependency Loop."""
        print("Beginning Self-Discovery Protocol...")
        run_diagnostics()

        while True:
            c = self.config
            print("\n---------------------------------------------------")
            print("Checking Knowledge Base...")

            if c.motor_i2c_bus is None or c.imu_i2c_bus is None:
                print("-> [MISSING] I2C Bus Assignments.")
                self.discover_buses()
                self.config.save()
                continue

            if c.accel_vertical_axis is None:
                print("-> [MISSING] Spatial Orientation (Vertical/Forward/Pitch).")
                self.init_hw()
                self.calibrate_static_orientation()
                self.config.save()
                continue

            if self.hw is None: self.init_hw()

            if c.min_power_visible == 0:
                print("-> [MISSING] Minimum Power Threshold (Friction).")
                self.find_min_power()
                self.config.save()
                continue

            if not c.motor_phasing_verified:
                print("-> [MISSING] Motor Phasing (Spin vs Drive).")
                self.align_motors_phase()
                self.config.save()
                continue

            if not c.motor_direction_verified:
                print("-> [MISSING] Motor Polarity (Stand Up Direction).")
                self.determine_motor_direction()
                self.config.save()
                continue

            if not c.motor_channels_verified:
                print("-> [MISSING] Left/Right Identification.")
                self.deduce_left_right_autonomous()
                self.config.save()
                continue

            if not c.motor_trim_verified:
                print("-> [MISSING] Motor Trim (Straight Drive).")
                self.calibrate_motor_trim()
                self.config.motor_trim_verified = True
                self.config.save()
                continue

            if c.control.kickup_power_forward == 0.0:
                print("-> [MISSING] Kick-Up Dynamics.")
                self.find_flop_thresholds()
                self.config.save()
                continue

            print("-> [VERIFYING] Final Configuration Check.")
            self.verify_final_configuration()

            print("\n[SUCCESS] Hardware Verified. Ready for Agent (Main Brain).")
            # (Summary print skipped for brevity of thought, but included in file)
            print("Summary:")
            print(f"  Buses: Motor={c.motor_i2c_bus}, IMU={c.imu_i2c_bus}")
            print(f"  Axes: Vert={c.accel_vertical_axis}, Fwd={c.accel_forward_axis}, Pitch={c.gyro_pitch_axis}, Yaw={c.gyro_yaw_axis}")
            break

        self.cleanup()

    # --- Tier 1: Hardware Connectivity ---
    def _scan_for_device(self, name: str, check_fn) -> int | None:
        candidates = [1, 3, 0, 2]
        for bus_id in candidates:
            try:
                bus = smbus.SMBus(bus_id)
                try:
                    if check_fn(bus):
                        print(f"  [FOUND] {name} on Bus {bus_id}")
                        return bus_id
                except OSError: pass
                finally:
                    try: bus.close()
                    except Exception: pass
            except Exception: pass
        return None

    def discover_buses(self):
        print("Scanning I2C Buses...")
        if (found := self._scan_for_device("PiconZero (Motors)", lambda b: b.read_byte_data(0x22, 0) or True)):
            self.config.motor_i2c_bus = found
        else:
            print("  [FAILURE] Could not find PiconZero.")
            sys.exit(1)

        if (found := self._scan_for_device("MPU6050 (IMU)", lambda b: b.read_byte_data(0x68, 0x75) == 0x68)):
            self.config.imu_i2c_bus = found
        else:
            print("  [FAILURE] Could not find MPU6050.")
            sys.exit(1)

    # --- Tier 2: The Physical World (Sensors) ---
    def calibrate_static_orientation(self):
        print(">>> Calibrating Orientation <<<")
        print("Please place the robot on the FLOOR (Back wheel).")
        self.wait_for_stability(2.0)

        # 1. Read Gravity (Vertical)
        print("  Measuring Gravity...")
        avg_back = self.collect_data(0, 0, 0.5).avg_accel_raw

        vert, _, _ = analyze_dominance(dict(avg_back.items()), "Vertical (Gravity)")
        self.config.accel_vertical_axis = Axis(vert)
        self.config.accel_vertical_invert = avg_back[vert] < 0
        print(f"  -> Vertical Axis: {vert.upper()} (Invert: {self.config.accel_vertical_invert})")

        # 2. Read Lean (Forward)
        sorted_axes = sorted(avg_back.items(), key=lambda x: abs(x[1]), reverse=True)
        forward_axis = sorted_axes[1][0]
        pitch_axis = sorted_axes[2][0]
        self.config.accel_forward_axis = Axis(forward_axis)

        # 3. Determine Pitch Axis (Cross Product)
        print("\n  Now TIP ROBOT FORWARD to Front Wheel.")
        input("Press Enter to measure FRONT position...")
        avg_front = self.collect_data(0, 0, 0.5).avg_accel_raw

        axis_vec = cross_product(avg_back, avg_front)
        detected_pitch, _, _ = analyze_dominance({k: abs(v) for k, v in axis_vec.items()}, "Pitch Axis (CrossProd)")

        if detected_pitch != pitch_axis:
             print(f"  [WARNING] Mismatch: CrossProd={detected_pitch} vs Magnitude={pitch_axis}. Trusting CrossProd.")
             pitch_axis = detected_pitch

        self.config.gyro_pitch_axis = Axis(pitch_axis)
        self.config.gyro_pitch_invert = axis_vec[pitch_axis] < 0
        print(f"  -> Pitch Axis: {pitch_axis.upper()} (Invert: {self.config.gyro_pitch_invert})")

        # Deduce Yaw/Roll
        remaining = list({'x', 'y', 'z'} - {vert, pitch_axis})
        if len(remaining) == 1:
            self.config.gyro_yaw_axis = Axis(vert)
            self.config.gyro_roll_axis = Axis(forward_axis)

        self.init_hw()

        # Deduce Forward Inversion
        delta_fwd = avg_front[forward_axis] - avg_back[forward_axis]
        self.config.accel_forward_invert = delta_fwd < 0
        print(f"  -> Forward Axis: {forward_axis.upper()} (Invert: {self.config.accel_forward_invert})")

    # --- Tier 3a: Friction Threshold ---
    def find_min_power(self):
        print(">>> Finding Minimum Power <<<")
        pwm = 10
        found = False
        while pwm <= 100:
            print(f"  Testing PWM {pwm}...")
            result = self.collect_data(pwm, pwm, 0.2)
            time.sleep(0.5)

            if result and result.max_rate > 10.0:
                print(f"  [FOUND] Motion detected at PWM {pwm}.")
                self.config.min_power_visible = pwm
                found = True
                break
            pwm += 5

        if not found:
             print("  [FAILURE] Robot did not move.")
             sys.exit(1)

    # --- Tier 3b: Phasing ---
    def align_motors_phase(self):
        print(">>> Verifying Motor Phasing <<<")
        for attempt in range(1, 4):
            print(f"  [Attempt {attempt}] Checking Phasing...")
            self.init_hw()
            power = self.config.min_power_visible + 10

            result = self.collect_data(power, power, 0.5)
            time.sleep(1.0)

            print(f"  -> Avg Yaw Rate: {result.avg_yaw_rate_abs:.1f} deg/s")

            if result.avg_yaw_rate_abs > 40.0:
                print("  -> Spinning detected. Inverting Right Motor.")
                self.config.motor_r_invert = not self.config.motor_r_invert
                continue
            else:
                print("  -> Motors Aligned.")
                self.config.motor_phasing_verified = True
                return

        print("  [FAILURE] Could not align motor phases.")
        sys.exit(1)

    # --- Tier 3c: Direction ---
    def determine_motor_direction(self):
        print(">>> Verifying Motor Direction <<<")
        for attempt in range(1, 4):
            print(f"  [Attempt {attempt}] Checking Direction...")
            self.init_hw()

            # Start Pitch (Quick measure)
            start_pitch = self.collect_data(0, 0, 0.1).pitch_end
            if abs(start_pitch) < 10:
                print("  [WARNING] Robot too upright. Lean it.")
                input("Press Enter...")
                continue

            power = self.config.min_power_visible + 20
            print(f"  Pulsing +{power}...")
            end_pitch = self.collect_data(power, power, 0.3).pitch_end
            time.sleep(1.0)

            print(f"  Pitch: {start_pitch:.1f} -> {end_pitch:.1f}")

            if abs(abs(start_pitch) - abs(end_pitch)) < 2.0:
                print("  [WARNING] Movement too small.")
                continue

            if abs(end_pitch) < abs(start_pitch):
                print("  [SUCCESS] Stood Up (Correct).")
                self.config.motor_direction_verified = True
                return
            else:
                print("  [FAILURE] Dug In (Inverted). Flipping both.")
                self.config.motor_l_invert = not self.config.motor_l_invert
                self.config.motor_r_invert = not self.config.motor_r_invert
                continue

        print("  [FAILURE] Could not determine direction.")
        sys.exit(1)

    # --- Tier 4: The Human Anchor ---
    def deduce_left_right_autonomous(self):
        print(">>> Autonomous Left/Right Verification <<<")
        print("I am going to spin...")
        self.wait_for_stability()

        for _ in range(3):
            self.init_hw()
            print("  Measuring Gravity...")
            accel = self.collect_data(0, 0, 0.2).avg_accel_raw
            up_vector = -accel

            power = self.config.min_power_visible + 15
            print(f"  Spinning {power}...")

            # Use raw gyro from new collect_data capability!
            avg_gyro = self.collect_data(power, -power, 1.5).avg_gyro_raw

            dot_prod = (up_vector.x * avg_gyro.x) + (up_vector.y * avg_gyro.y) + (up_vector.z * avg_gyro.z)
            print(f"  Dot Product: {dot_prod:.2f}")

            if abs(dot_prod) < 10.0:
                 print("  [WARNING] Spin rate too low.")
                 continue

            if dot_prod > 0: # CCW (Left)
                print("  -> Detected Left Spin. Ch0=Right, Ch1=Left.")
                if self.config.motor_l == 0:
                    print("  -> Swapping Channels.")
                    self.config.motor_l, self.config.motor_r = self.config.motor_r, self.config.motor_l
            else: # CW (Right)
                print("  -> Detected Right Spin. Ch0=Left, Ch1=Right.")
                if self.config.motor_l == 1:
                     print("  -> Swapping Channels.")
                     self.config.motor_l, self.config.motor_r = self.config.motor_r, self.config.motor_l

            # Verify Yaw Polarity
            yaw_axis = self.config.gyro_yaw_axis
            yaw_invert = self.config.gyro_yaw_invert
            raw_yaw = getattr(avg_gyro, yaw_axis.value)
            current_yaw_rate = -raw_yaw if yaw_invert else raw_yaw

            if (dot_prod > 0 and current_yaw_rate < 0) or (dot_prod < 0 and current_yaw_rate > 0):
                print("  -> Inverting Gyro Yaw.")
                self.config.gyro_yaw_invert = not self.config.gyro_yaw_invert

            self.config.motor_channels_verified = True
            return

        print("  [FAILURE] Could not deduce Left/Right.")
        sys.exit(1)

    # --- Tier 6: The Stride (Trim) ---
    def calibrate_motor_trim(self):
        print(">>> Motor Trim Calibration <<<")
        self.wait_for_stability()

        for attempt in range(10):
            self.init_hw()
            print(f"  [Attempt {attempt+1}] Trim: {self.config.motor_trim:.3f}")

            power = self.config.min_power_visible + 15
            avg_yaw = self.collect_data(power, power, 1.0).avg_yaw_rate
            print(f"    Avg Yaw Drift: {avg_yaw:.2f} deg/s")

            if abs(avg_yaw) < 2.0:
                print("  [SUCCESS] Drift is negligible.")
                return

            self.config.motor_trim += (avg_yaw * 0.005)
            self.config.motor_trim = max(-0.3, min(0.3, self.config.motor_trim))
            print(f"    -> New Trim: {self.config.motor_trim:.3f}")

        print("  [WARNING] Could not perfectly trim motors.")

    # --- Tier 5: Dynamics ---
    def _wait_for_start_condition(self, check_fn: Callable[[float], bool] | None, msg: str):
        print(msg)
        while True:
            self.wait_for_stability(1.0)
            if check_fn is None: return

            pitch = self.hw.read_imu_converted().pitch_angle
            if check_fn(pitch):
                print("  [OK] Position Verified.")
                return
            print(f"  [WAITING] Position incorrect (Pitch={pitch:.1f}). Please adjust.")
            time.sleep(1.0)

    def _perform_flop_test(self, name: str, start_msg: str, power_sign: float,
                           success_check: Callable[[float], bool], reset_msg: str,
                           start_check: Callable[[float], bool] | None = None) -> float | None:
        print(f"\n[Test] {name}")
        self._wait_for_start_condition(start_check, start_msg)

        power = self.config.min_power_visible + 10
        while power <= 100:
            print(f"  Trying Power {power}...")
            p = power * power_sign

            # Using collect_data for side effects (driving)
            self.collect_data(p, p, 0.4)
            time.sleep(1.0)

            if success_check(self.hw.read_imu_converted().pitch_angle):
                print(f"  [SUCCESS] Flopped at {power}.")
                return power

            power += 5
            self._wait_for_start_condition(start_check, reset_msg)
        return None

    def find_flop_thresholds(self):
        print(">>> Dynamic Kick-Up Calibration <<<")
        self.init_hw()
        if (fwd := self._perform_flop_test("Forward Flop", "Place on BACK.", 1.0, lambda p: p > 10, "Reset to Back...", lambda p: p < -10)):
            self.config.control.kickup_power_forward = fwd
        if (bwd := self._perform_flop_test("Backward Flop", "Place on FRONT.", -1.0, lambda p: p < -10, "Reset to Front...", lambda p: p > 10)):
            self.config.control.kickup_power_backward = bwd

    def verify_final_configuration(self):
        self.init_hw()
        print("  Running Autonomous Verification...")
        power = self.config.min_power_visible + 10

        # 1. Straight
        avg_yaw = self.collect_data(power, power, 1.0).avg_yaw_rate_abs
        print(f"    Straight Yaw Rate: {avg_yaw:.1f} deg/s")
        if avg_yaw > 40.0:
             print("  [FAILURE] Robot spun.")
             sys.exit(1)

        # 2. Turn Right (CW -> Negative Yaw)
        avg_yaw_signed = self.collect_data(power, -power, 1.0).avg_yaw_rate
        print(f"    Turn Yaw Rate: {avg_yaw_signed:.1f} deg/s")
        if avg_yaw_signed > -10.0:
             print("  [FAILURE] Did not turn Right (expected negative yaw).")
             sys.exit(1)

        print("  [PASS] Configuration Verified.")

if __name__ == "__main__":
    try:
        WiringCheck().run()
    except KeyboardInterrupt:
        print("\nInterrupted.")
    except Exception as e:
        print(f"\nCRITICAL ERROR: {e}")
        import traceback
        traceback.print_exc()
