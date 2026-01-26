import time
import piconzero as pz
from mpu6050 import mpu6050
import math

# --- CONFIGURATION ---
MOTOR_L = 0                 # PiconZero Channel 0
MOTOR_R = 1                 # PiconZero Channel 1
KP_START = 5.0              # Starting Proportional Gain
LOOP_TIME = 0.01            # 10ms loop (100Hz)
TARGET_ANGLE = 0.0          # Populated during calibration

# --- STATE MANAGEMENT ---
class RobotState:
    def __init__(self):
        self.running = True
        self.mode = "CALIBRATE"  # CALIBRATE -> TUNE -> BALANCE
        self.sensor = mpu6050(0x68)
        self.pitch = 0.0
        self.pid_kp = KP_START
        self.pid_ki = 0.0
        self.pid_kd = 0.0
        self.last_error = 0.0
        self.integral = 0.0
        self.vibration_counter = 0

    def get_pitch(self):
        # The library returns dicts: {'x': val, 'y': val, 'z': val}
        accel = self.sensor.get_accel_data()
        gyro = self.sensor.get_gyro_data()
        
        # Calculate Accelerometer Angle (assuming Y is forward)
        # We need math here but let's keep it concise using a rough approximation 
        # for small angles or standard atan2 if we want to be proper.
        acc_angle = math.degrees(math.atan2(accel['y'], accel['z']))
        
        # Complementary Filter: Mix Gyro (fast) + Accel (stable)
        # Note: Check if your Gyro X or Y is the pitch axis!
        # Usually it is X if Y is forward.
        gyro_rate = gyro['x'] 
        self.pitch = 0.98 * (self.pitch + gyro_rate * LOOP_TIME) + 0.02 * acc_angle
        return self.pitch, gyro['z'] # Return pitch and yaw_rate

# --- MAIN LOOP ---
def main():
    bot = RobotState()
    pz.init()
    print(">>> ROBOT ALIVE. Hold vertical for STEP 1.")
    time.sleep(2) # Give user a moment to grab it

    try:
        while bot.running:
            start_time = time.time()
            current_pitch, yaw_rate = bot.get_pitch()

            # --- STEP 1: CALIBRATION ---
            if bot.mode == "CALIBRATE":
                # Simple lazy calibration: Current angle is "Zero"
                TARGET_ANGLE = current_pitch
                print(f"-> Calibrated Vertical at: {TARGET_ANGLE:.2f}")
                print("-> Let it wobble gently (Step 2)...")
                bot.mode = "TUNE"
                time.sleep(1)

            # --- STEP 2: AUTO-TUNING (The "Wobble" Phase) ---
            elif bot.mode == "TUNE":
                error = TARGET_ANGLE - current_pitch
                
                # Slowly ramp up Kp until it feels "stiff" (simple heuristic)
                bot.pid_kp += 0.05
                
                # Check for oscillation (sign flip)
                if (error > 0 and bot.last_error < 0) or (error < 0 and bot.last_error > 0):
                    bot.vibration_counter += 1
                
                # If we oscillated 10 times, we found our limit
                if bot.vibration_counter > 10:
                    bot.pid_kp *= 0.6  # Safe margin (Ziegler-Nicholsish)
                    bot.pid_kd = bot.pid_kp * 0.05
                    bot.pid_ki = bot.pid_kp * 0.005
                    print(f"-> Tuned! Kp={bot.pid_kp:.2f} Kd={bot.pid_kd:.2f}")
                    bot.mode = "BALANCE"
                    bot.vibration_counter = 0

                # Simple P-drive during tuning
                output = bot.pid_kp * error
                drive(output, 0)
                bot.last_error = error

            # --- STEP 3: BALANCING ---
            elif bot.mode == "BALANCE":
                # PID Calculation
                error = TARGET_ANGLE - current_pitch
                
                bot.integral += error * LOOP_TIME
                bot.integral = max(min(bot.integral, 20), -20) # Clamp
                derivative = (error - bot.last_error) / LOOP_TIME
                
                output = (bot.pid_kp * error) + (bot.pid_ki * bot.integral) + (bot.pid_kd * derivative)
                
                # Precession Correction (Yaw)
                # If gyro Z is spinning, counter-spin motors
                turn_correction = -yaw_rate * 0.5
                
                # Vibration Guard (The "Vibrating" check)
                if abs(error - bot.last_error) > 5.0: # Sudden jerk
                    bot.vibration_counter += 1
                    if bot.vibration_counter > 10:
                        print("-> Vibrating! Relaxing gains...")
                        bot.pid_kp *= 0.9
                        bot.vibration_counter = 0

                drive(output, turn_correction)
                bot.last_error = error
                
                # Safety Kill Switch (Fell over)
                if abs(error) > 45:
                    print("!!! FELL OVER !!!")
                    bot.running = False

            # Loop Timing
            elapsed = time.time() - start_time
            if elapsed < LOOP_TIME:
                time.sleep(LOOP_TIME - elapsed)

    except KeyboardInterrupt:
        pass
    finally:
        pz.stop()
        print("\nMotors Stopped.")

def drive(speed, turn):
    # Mix speed and turn
    left = speed + turn
    right = speed - turn
    
    # Clamp to +/- 100 for Picon Zero
    left = int(max(min(left, 100), -100))
    right = int(max(min(right, 100), -100))
    
    pz.setMotor(MOTOR_L, left)
    pz.setMotor(MOTOR_R, right)

if __name__ == "__main__":
    main()
