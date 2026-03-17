import time
import sys
import math
import smbus2

# SHIM: Trick the mpu6050 library into using smbus2 instead of the missing smbus
sys.modules['smbus'] = smbus2

from mpu6050 import mpu6050 # noqa: E402

APP_START_TIME = time.time()

# Hardware I2C Addresses
PICONZERO_ADDR = 0x22
MPU6050_ADDR = 0x68
CMD_RESET = 20

def get_ms():
    """Returns milliseconds since the application started."""
    return int((time.time() - APP_START_TIME) * 1000)

def scan_for_devices():
    """Step 1: Figure out the bus channels for motors and gyro."""
    print(f"[{get_ms():05d} ms] --- Step 1: Scanning for I2C Devices ---")
    buses_to_check = [3, 2, 1, 0]
    motor_bus = None
    gyro_bus = None

    for bus_num in buses_to_check:
        try:
            with smbus2.SMBus(bus_num) as bus:
                # Check for PiconZero
                try:
                    bus.read_byte(PICONZERO_ADDR)
                    motor_bus = bus_num
                except Exception:
                    pass

                # Check for MPU6050
                try:
                    bus.read_byte(MPU6050_ADDR)
                    gyro_bus = bus_num
                except Exception:
                    pass
        except Exception:
            pass

    print(f"[{get_ms():05d} ms] Results: Motor Bus = {motor_bus}, Gyro Bus = {gyro_bus}")
    return motor_bus, gyro_bus

def monitor_imu(sensor, duration=1.0):
    """Helper to monitor IMU for a given duration, print readings, and return averages."""
    start_time = time.time()
    gyro_readings = []
    accel_readings = []

    while time.time() - start_time < duration:
        ts_ms = get_ms()
        try:
            gyro_data = sensor.get_gyro_data()
            accel_data = sensor.get_accel_data()

            gyro_readings.append(gyro_data)
            accel_readings.append(accel_data)

            magnitude = (accel_data['x']**2 + accel_data['y']**2 + accel_data['z']**2) ** 0.5

            print(f"[{ts_ms:05d} ms] Gyro: X={gyro_data['x']:>6.2f}, Y={gyro_data['y']:>6.2f}, Z={gyro_data['z']:>6.2f} "
                  f"| Accel: X={accel_data['x']:>6.2f}, Y={accel_data['y']:>6.2f}, Z={accel_data['z']:>6.2f} | Tot G: {magnitude:.2f}")

        except Exception as err:
            print(f"[{ts_ms:05d} ms] ERROR reading IMU: {type(err).__name__} - {err}")

        time.sleep(0.05) # Sample at ~20Hz

    if not gyro_readings:
        print(f"[{get_ms():05d} ms] WARNING: No valid readings captured during this period.")
        empty = {"x": 0.0, "y": 0.0, "z": 0.0}
        return empty, empty

    avg_gx = sum(r['x'] for r in gyro_readings) / len(gyro_readings)
    avg_gy = sum(r['y'] for r in gyro_readings) / len(gyro_readings)
    avg_gz = sum(r['z'] for r in gyro_readings) / len(gyro_readings)

    avg_ax = sum(r['x'] for r in accel_readings) / len(accel_readings)
    avg_ay = sum(r['y'] for r in accel_readings) / len(accel_readings)
    avg_az = sum(r['z'] for r in accel_readings) / len(accel_readings)

    return {"x": avg_gx, "y": avg_gy, "z": avg_gz}, {"x": avg_ax, "y": avg_ay, "z": avg_az}

def main():
    motor_bus_num, gyro_bus_num = scan_for_devices()

    if motor_bus_num is None or gyro_bus_num is None:
        print(f"[{get_ms():05d} ms] CRITICAL: Could not find both devices. Please check wiring.")
        return

    try:
        gyro_sensor = mpu6050(MPU6050_ADDR, bus=gyro_bus_num)
        motor_bus = smbus2.SMBus(motor_bus_num)

        motor_bus.write_byte_data(PICONZERO_ADDR, CMD_RESET, 0)
        time.sleep(0.1)
    except Exception as err:
        print(f"[{get_ms():05d} ms] CRITICAL: Failed to initialize hardware: {err}")
        return

    def set_motors(val_0, val_1):
        try:
            motor_bus.write_byte_data(PICONZERO_ADDR, 0, val_0 & 0xFF)
            motor_bus.write_byte_data(PICONZERO_ADDR, 1, val_1 & 0xFF)
        except Exception as err:
            print(f"[{get_ms():05d} ms] ERROR setting motors: {err}")

    try:
        print(f"\n[{get_ms():05d} ms] --- Step 2: Sitting Still (Monitoring Static Gravity) ---")
        set_motors(0, 0)
        drift_gyro, static_accel = monitor_imu(gyro_sensor, duration=1.0)
        print(f"[{get_ms():05d} ms] ---> Accel Averages: X={static_accel['x']:.2f}, Y={static_accel['y']:.2f}, Z={static_accel['z']:.2f}")

        print(f"\n[{get_ms():05d} ms] --- Step 3: Motors at +30 for 1 second ---")
        set_motors(30, 30)
        fwd_cmd_gyro, _ = monitor_imu(gyro_sensor, duration=1.0)
        set_motors(0, 0)
        time.sleep(0.5)

        print(f"\n[{get_ms():05d} ms] --- Step 4: Motors at -30 for 1 second ---")
        set_motors(-30, -30)
        rev_cmd_gyro, _ = monitor_imu(gyro_sensor, duration=1.0)
        set_motors(0, 0)
        time.sleep(0.5)

        print(f"\n[{get_ms():05d} ms] --- Step 5: Motors opposite (+30 / -30) for 1 second ---")
        set_motors(30, -30)
        opp_cmd_gyro, _ = monitor_imu(gyro_sensor, duration=1.0)
        set_motors(0, 0)

    finally:
        print(f"\n[{get_ms():05d} ms] Cleaning up and stopping motors...")
        set_motors(0, 0)
        motor_bus.close()

    # --- Data Analysis & Math ---
    print("\n==================================================")
    print("      DATA-DRIVEN HARDWARE CONFIGURATION")
    print("==================================================")

    # 1. Map Axes using Static Gravity
    # max gravity = vertical, mid gravity = forward lean, min gravity = lateral
    axes = ['x', 'y', 'z']
    sorted_axes = sorted(axes, key=lambda a: abs(static_accel[a]))

    lateral_axis = sorted_axes[0]
    forward_axis = sorted_axes[1]
    vertical_axis = sorted_axes[2]

    # 2. Trigonometry to find the rest angle (Pitch)
    fwd_g = abs(static_accel[forward_axis])
    vert_g = abs(static_accel[vertical_axis])
    rest_angle_deg = math.degrees(math.atan2(fwd_g, vert_g))

    # 3. Deduce Polarity using Yaw (Vertical Axis) Rotation
    # If giving both motors the same sign causes more yaw than opposing signs, they are physically mirrored.
    yaw_val_same_cmd = abs(fwd_cmd_gyro[vertical_axis])
    yaw_val_opp_cmd = abs(opp_cmd_gyro[vertical_axis])

    if yaw_val_same_cmd > yaw_val_opp_cmd:
        # Same command caused spinning, opposing command caused straight driving
        motor1_polarity = 1
        motor2_polarity = -1
    else:
        # Opposing command caused spinning, same command caused straight driving
        motor1_polarity = 1
        motor2_polarity = 1

    print("HardwareConfig(")
    print(f"    motor_i2c_bus={motor_bus_num},")
    print(f"    imu_i2c_bus={gyro_bus_num},\n")
    print(f"    motor1_polarity={motor1_polarity},")
    print(f"    motor2_polarity={motor2_polarity},\n")
    print("    # Accelerometer Mapping")
    print(f"    accel_vertical_axis=Axis.{vertical_axis.upper()},")
    print(f"    accel_forward_axis=Axis.{forward_axis.upper()},\n")
    print("    # Gyroscope Mapping (Derived from rigid Accel mapping)")
    print(f"    gyro_yaw_axis=Axis.{vertical_axis.upper()},    # Rotates around Vertical")
    print(f"    gyro_roll_axis=Axis.{forward_axis.upper()},    # Rotates around Forward")
    print(f"    gyro_pitch_axis=Axis.{lateral_axis.upper()}    # Rotates around Lateral")
    print(")\n")
    print(f"Calculated Rest Angle: {rest_angle_deg:.2f} degrees")
    print("==================================================")

if __name__ == "__main__":
    main()