import sys
import time
import math
import smbus
import threading
import piconzero as pz  # Requires: sudo apt-get install python-smbus && wget http://4tronix.co.uk/piconz.sh -O piconz.sh && bash piconz.sh

# ==========================================
# HARDWARE CONFIGURATION
# ==========================================
MPU6050_ADDR = 0x68
MOTOR_L = 0  # Picon Zero Motor A
MOTOR_R = 1  # Picon Zero Motor B
Loop_Time = 0.01  # 100Hz control loop

# ==========================================
# MPU-6050 DRIVER (Embedded for pristine single-file portability)
# ==========================================
class MPU6050:
    def __init__(self, address=MPU6050_ADDR, bus=1):
        self.bus = smbus.SMBus(bus)
        self.address = address
        # Wake up MPU6050
        self.bus.write_byte_data(self.address, 0x6B, 0)
        # Set Gyro Range to +/- 250 deg/s
        self.bus.write_byte_data(self.address, 0x1B, 0)
        # Set Accel Range to +/- 2g
        self.bus.write_byte_data(self.address, 0x1C, 0)
    
    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(self.address, addr)
        low = self.bus.read_byte_data(self.address, addr+1)
        val = (high << 8) + low
        if val > 32768: val = val - 65536
        return val

    def get_data(self):
        # Read Accelerometer
        acc_x = self.read_raw_data(0x3B) / 16384.0
        acc_y = self.read_raw_data(0x3D) / 16384.0
        acc_z = self.read_raw_data(0x3F) / 16384.0
        # Read Gyroscope
        gyro_x = self.read_raw_data(0x43) / 131.0
        gyro_y = self.read_raw_data(0x45) / 131.0
        gyro_z = self.read_raw_data(0x47) / 131.0
        return {'ax': acc_x, 'ay': acc_y, 'az': acc_z, 
                'gx': gyro_x, 'gy': gyro_y, 'gz': gyro_z}

# ==========================================
# PID CONTROLLER CLASS
# ==========================================
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        
    def compute(self, error, dt):
        self.integral += error * dt
        # Anti-windup clamp
        self.integral = max(min(self.integral, 20), -20)
        derivative = (error - self.prev_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error
        return output

    def relax(self, factor=0.9):
        """Reduce gains if vibrating"""
        self.kp *= factor
        self.kd *= factor
        print(f"  -> Vibrations detected! Relaxing PID: Kp={self.kp:.2f}")

# ==========================================
# MAIN ROBOT LOGIC
# ==========================================
class BalanceBot:
    def __init__(self):
        print("Initializing Hardware...")
        pz.init()
        self.mpu = MPU6050()
        
        # State variables
        self.running = True
        self.state = "CALIBRATING_VERTICAL"
        self.pitch = 0.0
        self.target_angle = 0.0
        self.gyro_offset_y = 0.0
        
        # PID Placeholders (Auto-tuned later)
        self.pid = PID(kp=0, ki=0, kd=0) 
        
        # Adaptive features
        self.yaw_correction = 0.0
        self.vibration_counter = 0
        self.history = []
        
        # Start the loop
        self.thread = threading.Thread(target=self.control_loop)
        self.thread.start()

    def get_pitch(self):
        # Complementary Filter: 98% Gyro, 2% Accel
        data = self.mpu.get_data()
        
        # Calculate accel angle (assuming Z is vertical, Y is forward axis)
        # Note: You might need to swap X/Y depending on mounting orientation!
        # This assumes the bonnet is mounted flat with pins forward/back.
        acc_angle = math.degrees(math.atan2(data['ay'], data['az']))
        
        # Integrate gyro
        gyro_rate = data['gx'] - self.gyro_offset_y # Use X gyro for pitch if Y is forward
        
        # Filter
        self.pitch = 0.98 * (self.pitch + gyro_rate * Loop_Time) + 0.02 * acc_angle
        
        return self.pitch, gyro_rate, data['gz'] # gz is yaw rate

    def calibrate_vertical(self):
        """Step 1: User holds robot vertical. We zero offsets."""
        print("\n[STEP 1] HOLD VERTICAL. Calibrating sensors...")
        gyro_sum = 0
        acc_angle_sum = 0
        for _ in range(100):
            data = self.mpu.get_data()
            gyro_sum += data['gx']
            acc_angle_sum += math.degrees(math.atan2(data['ay'], data['az']))
            time.sleep(Loop_Time)
            
        self.gyro_offset_y = gyro_sum / 100.0
        self.target_angle = acc_angle_sum / 100.0
        # Initialize pitch to current angle so we don't jerk start
        self.pitch = self.target_angle 
        print(f"  -> Vertical Set at {self.target_angle:.2f} deg")
        print(f"  -> Gyro Drift Cancelled: {self.gyro_offset_y:.2f}")
        time.sleep(1)
        self.state = "AUTO_TUNING"

    def auto_tune(self):
        """Step 2: User lets it wobble. We find Kp."""
        print("\n[STEP 2] LETTING IT WOBBLE. Detecting PID values...")
        print("  -> Gently cup hands around it. Don't let it fall.")
        
        # Start with safe conservative values
        test_kp = 5.0
        oscillation_count = 0
        last_sign = 0
        
        # Ramp up Kp until we see consistent oscillation (Ultimate Gain method simplified)
        start_time = time.time()
        
        while time.time() - start_time < 8.0: # 8 second tuning window
            current_pitch, _, _ = self.get_pitch()
            error = self.target_angle - current_pitch
            
            # Simple P-only control for tuning
            output = test_kp * error
            self.drive_motors(output, 0)
            
            # Detect zero crossing (oscillation)
            if math.copysign(1, error) != last_sign:
                oscillation_count += 1
                last_sign = math.copysign(1, error)
            
            # Slowly increase aggression
            test_kp += 0.05
            time.sleep(Loop_Time)

        # Heuristic: If it oscillated a lot, we found the limit. 
        # Ziegler-Nichols suggests 0.6 * K_ultimate.
        # Since motors are "crappy", we'll be conservative.
        final_kp = test_kp * 0.5 
        final_kd = final_kp * 0.05 # Generic starting ratio
        final_ki = final_kp * 0.005
        
        print(f"  -> Tuned! Kp: {final_kp:.2f}, Ki: {final_ki:.4f}, Kd: {final_kd:.2f}")
        self.pid = PID(final_kp, final_ki, final_kd)
        self.state = "BALANCING"

    def drive_motors(self, balance_output, yaw_correction):
        # Clip max speed
        balance_output = max(min(balance_output, 100), -100)
        
        # Apply differential for yaw (precession correction)
        # Note: Check motor wiring. If one goes backward, flip the +/- or swap wires
        left_speed = balance_output + yaw_correction
        right_speed = balance_output - yaw_correction
        
        # PiconZero setMotor takes channel (0/1) and speed (-100 to 100)
        pz.setMotor(MOTOR_L, int(left_speed))
        pz.setMotor(MOTOR_R, int(right_speed))

    def control_loop(self):
        while self.running:
            loop_start = time.time()
            
            if self.state == "CALIBRATING_VERTICAL":
                self.calibrate_vertical()
                
            elif self.state == "AUTO_TUNING":
                self.auto_tune()
                
            elif self.state == "BALANCING":
                current_pitch, _, yaw_rate = self.get_pitch()
                
                # Safety: Cutoff if fell over
                if abs(current_pitch - self.target_angle) > 40:
                    pz.stop()
                    print("!!! FELL OVER - MOTORS OFF !!!")
                    self.running = False
                    break
                
                # 1. Main Balance PID
                error = self.target_angle - current_pitch
                balance_out = self.pid.compute(error, Loop_Time)
                
                # 2. Precession (Yaw) Correction
                # If we are spinning (yawing) but didn't ask to, correct it.
                # Simple P-controller on Yaw Rate to keep it 0
                self.yaw_correction = -yaw_rate * 0.5 
                
                # 3. Vibration Detection (Adaptive Gain)
                self.history.append(error)
                if len(self.history) > 20: self.history.pop(0)
                
                # Calculate variance
                if len(self.history) == 20:
                    variance = sum((x - (sum(self.history)/20)) ** 2 for x in self.history) / 20
                    if variance > 5.0: # Threshold for "jittery"
                        self.vibration_counter += 1
                        if self.vibration_counter > 50: # Sustained vibration
                            self.pid.relax()
                            self.vibration_counter = 0
                            self.history = [] # Reset history

                self.drive_motors(balance_out, self.yaw_correction)

            # Maintain Loop Frequency
            elapsed = time.time() - loop_start
            if elapsed < Loop_Time:
                time.sleep(Loop_Time - elapsed)

if __name__ == "__main__":
    try:
        bot = BalanceBot()
        while bot.running:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping...")
        pz.stop()
        sys.exit()
