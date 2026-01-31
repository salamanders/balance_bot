# balance_bot

## Getting Started (First Time Setup)

Follow these steps to get your robot up and running from a fresh Raspberry Pi installation.

### 1. Clone the Repository
```bash
git clone https://github.com/your-org/balance-bot.git
cd balance-bot
```

### 2. System Setup
Run the setup script to enable I2C and install necessary system libraries (like the Picon Zero driver).
```bash
chmod +x setup.sh
sudo ./setup.sh
```

### 3. Install `uv`
This project uses `uv` for Python dependency management.
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source $HOME/.cargo/env
```

### 4. Install Dependencies
Sync the project dependencies.
```bash
uv sync
```

### 5. Verify Hardware (Software Check)
Run the interactive wiring check to verify motor direction and gyro orientation.
```bash
uv run balance-bot --check-wiring
```
Follow the on-screen instructions.

### 6. Run the Robot
Start the balancing software.
```bash
uv run balance-bot
```

---

## Physical Wiring Check
*Note: This section describes the physical connections required before running the software.*

* **MPU-6050**: VCC to 3.3V, GND to GND, SDA to SDA, SCL to SCL.
* **Motors**: Plugged into Motor A and Motor B on the Picon Zero.

**Orientation**: This code assumes the MPU is mounted such that tilting forward/back rotates the X-axis gyroscope. If your robot acts crazy immediately, change data['gx'] to data['gy'] in the get_pitch method.

## Bootup Process

1. Run `uv run balance-bot`
2. Hold the robot perfectly upright and still. The console will say `[STEP 1] HOLD VERTICAL`. Wait 1 second.
3. The console will say `[STEP 2] LETTING IT WOBBLE`. Gently support it with your hands (don't grip it tight, just cage it). It will twitch back and forth.
4. The console will print `Tuned! Kp: ...` and it will take over. Let go!

## Codebase Assumptions

This code relies on several hardware, physical, and configuration assumptions. If your robot differs, you may need to adjust these values in the cited files.

### Hardware Configuration
*   **I2C Addresses**: Assumes MPU-6050 is at `0x68` and Picon Zero HAT is at `0x22`.
    *   *Citation*: [src/balance_bot/robot_hardware.py](src/balance_bot/robot_hardware.py) (MPU), [src/balance_bot/piconzero.py](src/balance_bot/piconzero.py) (PiconZero `I2C_ADDRESS`).
*   **Motor Channels**: Assumes Left Motor is Channel 0 and Right Motor is Channel 1.
    *   *Citation*: [src/balance_bot/config.py](src/balance_bot/config.py) (`motor_l`, `motor_r` in `RobotConfig`).
*   **Motor Input Range**: Assumes motor driver accepts values from -100 to 100.
    *   *Citation*: [src/balance_bot/robot_hardware.py](src/balance_bot/robot_hardware.py) (`MOTOR_MIN_OUTPUT`, `MOTOR_MAX_OUTPUT`).

### Physics & Mounting
*   **Mounting Orientation**: Default assumes the Pitch axis corresponds to the **X-axis** of the gyroscope.
    *   *Citation*: [src/balance_bot/config.py](src/balance_bot/config.py) (`gyro_pitch_axis` in `RobotConfig`).
*   **Fall Limit**: The robot considers itself "fallen" (and stops motors) if the pitch angle exceeds **45 degrees**.
    *   *Citation*: [src/balance_bot/config.py](src/balance_bot/config.py) (`fall_angle_limit` in `RobotConfig`).
*   **Gravity Vector**: Pitch calculation assumes the Z-axis accelerometer measures gravity when upright.
    *   *Citation*: [src/balance_bot/utils.py](src/balance_bot/utils.py) (`calculate_pitch`).

### Control System
*   **Loop Frequency**: The control loop is designed to run at **100 Hz** (10ms per loop).
    *   *Citation*: [src/balance_bot/config.py](src/balance_bot/config.py) (`loop_time` in `RobotConfig`).
*   **Motor Deadband**: Assumes motors require a minimum PWM of **20.0** to overcome static friction.
    *   *Citation*: [src/balance_bot/config.py](src/balance_bot/config.py) (`min_pwm` in `BatteryConfig`).
*   **PID Defaults**: Starts with `Kp=25.0`, `Ki=0.0`, `Kd=0.5`.
    *   *Citation*: [src/balance_bot/config.py](src/balance_bot/config.py) (`PIDParams`).

### System Environment
*   **Status LEDs**: Assumes availability of system LEDs at `/sys/class/leds/led0/brightness` or `/sys/class/leds/ACT/brightness`.
    *   *Citation*: [src/balance_bot/leds.py](src/balance_bot/leds.py) (`_find_led_path`).
