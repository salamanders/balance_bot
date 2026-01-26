import time
import math
import os
import json
import sys

# --- HARDWARE IMPORTS ---
try:
    if os.environ.get("MOCK_HARDWARE"):
        raise ImportError("Mock requested")
    from . import piconzero as pz
    from mpu6050 import mpu6050
except (ImportError, OSError):
    print("Running in Mock Mode")
    from .mocks import MockPiconZero
    from .mocks import MockMPU6050 as mpu6050

    pz = MockPiconZero()

from .leds import LedController
from .tuner import ContinuousTuner
from .battery import BatteryEstimator

# --- CONFIGURATION ---
MOTOR_L = 0  # PiconZero Channel 0
MOTOR_R = 1  # PiconZero Channel 1
KP_START = 5.0  # Starting Proportional Gain
LOOP_TIME = 0.01  # 10ms loop (100Hz)
CONFIG_FILE = "pid_config.json"
FORCE_CALIB_FILE = "force_calibration.txt"


# --- STATE MANAGEMENT ---
class RobotState:
    def __init__(self):
        self.running = True
        self.mode = "CALIBRATE"  # CALIBRATE -> TUNE -> BALANCE -> RECOVER
        self.sensor = mpu6050(0x68)
        self.pitch = 0.0
        self.pid_kp = KP_START
        self.pid_ki = 0.0
        self.pid_kd = 0.0
        self.last_error = 0.0
        self.integral = 0.0
        self.vibration_counter = 0
        self.target_angle = 0.0
        self.tuner = ContinuousTuner()
        self.battery = BatteryEstimator()
        self.config_dirty = False
        self.last_save_time = time.time()
        self.load_config()

    def load_config(self):
        force = False
        if os.path.exists(FORCE_CALIB_FILE):
            print(f"-> Force calibration file found: {FORCE_CALIB_FILE}")
            force = True
        if "--force-calibration" in sys.argv:
            print("-> Force calibration flag found")
            force = True

        if not force and os.path.exists(CONFIG_FILE):
            try:
                with open(CONFIG_FILE, "r") as f:
                    data = json.load(f)
                    self.pid_kp = data.get("pid_kp", KP_START)
                    self.pid_ki = data.get("pid_ki", 0.0)
                    self.pid_kd = data.get("pid_kd", 0.0)
                    self.target_angle = data.get("target_angle", 0.0)
                    print(
                        f"-> Loaded Config: Kp={self.pid_kp:.2f} Ki={self.pid_ki:.2f} Kd={self.pid_kd:.2f} Target={self.target_angle:.2f}"
                    )
                    self.mode = "BALANCE"
            except (json.JSONDecodeError, OSError) as e:
                print(f"-> Error loading config: {e}")

    def save_config(self):
        data = {
            "pid_kp": self.pid_kp,
            "pid_ki": self.pid_ki,
            "pid_kd": self.pid_kd,
            "target_angle": self.target_angle,
        }
        try:
            with open(CONFIG_FILE, "w") as f:
                json.dump(data, f)
            print("-> Config saved.")
        except OSError as e:
            print(f"-> Error saving config: {e}")

    def get_pitch(self):
        # The library returns dicts: {'x': val, 'y': val, 'z': val}
        accel = self.sensor.get_accel_data()
        gyro = self.sensor.get_gyro_data()

        # Calculate Accelerometer Angle (assuming Y is forward)
        acc_angle = math.degrees(math.atan2(accel["y"], accel["z"]))

        # Complementary Filter: Mix Gyro (fast) + Accel (stable)
        gyro_rate = gyro["x"]
        self.pitch = 0.98 * (self.pitch + gyro_rate * LOOP_TIME) + 0.02 * acc_angle
        # Return pitch, pitch_rate (accel check), and yaw_rate
        return self.pitch, gyro_rate, gyro["z"]


# --- MAIN LOOP ---
def main():
    bot = RobotState()
    led = LedController()
    pz.init()
    print(">>> ROBOT ALIVE. Hold vertical for STEP 1.")

    # Fast blink for setup/waiting
    led.signal_setup()
    setup_start = time.time()
    while time.time() - setup_start < 2:
        led.update()
        time.sleep(0.01)

    try:
        while bot.running:
            start_time = time.time()
            current_pitch, pitch_rate, yaw_rate = bot.get_pitch()
            led.update()

            # --- STEP 1: CALIBRATION ---
            if bot.mode == "CALIBRATE":
                led.signal_setup()
                bot.target_angle = current_pitch
                print(f"-> Calibrated Vertical at: {bot.target_angle:.2f}")
                print("-> Let it wobble gently (Step 2)...")
                bot.mode = "TUNE"
                # Brief pause for user to react, but keep LED updating
                pause_start = time.time()
                while time.time() - pause_start < 1:
                    led.update()
                    time.sleep(0.01)

            # --- STEP 2: AUTO-TUNING (The "Wobble" Phase) ---
            elif bot.mode == "TUNE":
                led.signal_tuning()
                error = bot.target_angle - current_pitch

                bot.pid_kp += 0.05

                if (error > 0 and bot.last_error < 0) or (
                    error < 0 and bot.last_error > 0
                ):
                    bot.vibration_counter += 1

                if bot.vibration_counter > 10:
                    bot.pid_kp *= 0.6
                    bot.pid_kd = bot.pid_kp * 0.05
                    bot.pid_ki = bot.pid_kp * 0.005
                    print(f"-> Tuned! Kp={bot.pid_kp:.2f} Kd={bot.pid_kd:.2f}")
                    bot.save_config()
                    bot.mode = "BALANCE"
                    bot.vibration_counter = 0

                output = bot.pid_kp * error
                drive(output, 0)
                bot.last_error = error

            # --- STEP 3: BALANCING ---
            elif bot.mode == "BALANCE":
                led.signal_ready()
                error = bot.target_angle - current_pitch

                bot.integral += error * LOOP_TIME
                bot.integral = max(min(bot.integral, 20), -20)
                derivative = (error - bot.last_error) / LOOP_TIME

                output = (
                    (bot.pid_kp * error)
                    + (bot.pid_ki * bot.integral)
                    + (bot.pid_kd * derivative)
                )

                turn_correction = -yaw_rate * 0.5

                # Continuous Tuning
                kp_n, ki_n, kd_n = bot.tuner.update(error)
                if kp_n != 0 or ki_n != 0 or kd_n != 0:
                    bot.pid_kp += kp_n
                    bot.pid_ki += ki_n
                    bot.pid_kd += kd_n
                    # Safety limits
                    bot.pid_kp = max(0.1, bot.pid_kp)
                    bot.pid_ki = max(0.0, bot.pid_ki)
                    bot.pid_kd = max(0.0, bot.pid_kd)
                    bot.config_dirty = True
                    print(
                        f"-> Tuned: P={bot.pid_kp:.2f} I={bot.pid_ki:.3f} D={bot.pid_kd:.2f}"
                    )

                # Battery Estimation
                # We need Angular Acceleration. Approximate with (rate_now - rate_last) / dt
                # But we don't store rate_last in bot state yet.
                # However, derivative term in PID is d(Error)/dt = -d(Pitch)/dt = -PitchRate.
                # So PitchRate ~ -Derivative (if target is constant).
                # But we need d(PitchRate)/dt.
                # Let's approximate angular accel using change in PitchRate
                if not hasattr(bot, "last_pitch_rate"):
                    bot.last_pitch_rate = pitch_rate

                ang_accel = (pitch_rate - bot.last_pitch_rate) / LOOP_TIME
                bot.last_pitch_rate = pitch_rate

                comp_factor = bot.battery.update(output, ang_accel, LOOP_TIME)
                if comp_factor < 0.95 and (time.time() * 10) % 50 < 1: # Print occasionally
                    print(f"-> Low Battery? Compensating: {int(comp_factor*100)}%")

                drive(output / comp_factor, turn_correction)
                bot.last_error = error

                # Save config occasionally if dirty (every 30s) or on fall
                if bot.config_dirty and (time.time() - bot.last_save_time > 30):
                    bot.save_config()
                    bot.last_save_time = time.time()
                    bot.config_dirty = False

                if abs(error) > 45:
                    print("!!! FELL OVER !!!")
                    drive(0, 0)
                    if bot.config_dirty:
                        bot.save_config()
                        bot.config_dirty = False
                    bot.mode = "RECOVER"

            # --- STEP 4: RECOVER ---
            elif bot.mode == "RECOVER":
                led.signal_off()
                drive(0, 0)
                error = bot.target_angle - current_pitch

                if abs(error) < 5.0:
                    print(f"-> Upright detected! (Pitch: {current_pitch:.2f})")
                    led.countdown()
                    bot.mode = "BALANCE"
                    bot.last_error = error
                    bot.integral = 0.0
                    bot.vibration_counter = 0

            elapsed = time.time() - start_time
            if elapsed < LOOP_TIME:
                time.sleep(LOOP_TIME - elapsed)

    except KeyboardInterrupt:
        pass
    finally:
        if bot.config_dirty:
            bot.save_config()
        pz.stop()
        led.signal_off()
        print("\nMotors Stopped.")


def drive(speed, turn):
    left = speed + turn
    right = speed - turn

    left = int(max(min(left, 100), -100))
    right = int(max(min(right, 100), -100))

    pz.setMotor(MOTOR_L, left)
    pz.setMotor(MOTOR_R, right)


if __name__ == "__main__":
    main()
