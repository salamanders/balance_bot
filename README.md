# balance_bot

## References

### Root Directory
* [.gitignore](.gitignore): Git configuration specifying which files to ignore.
* [.python-version](.python-version): Specifies the Python version used by the project/tools.
* [AGENTS.md](AGENTS.md): Instructions/Context for AI agents working on this codebase.
* [AUTO_RUN.md](AUTO_RUN.md): Instructions for setting up the robot to run automatically on boot via systemd.
* [ITERATE.md](ITERATE.md): Guidelines and prompts for AI-driven code refactoring and maintenance.
* [LICENSE](LICENSE): Project license file.
* [Makefile](Makefile): Shortcut commands for installation, linting, formatting, and running.
* [README.md](README.md): The main entry point documentation.
* [environment.sh](environment.sh): Script to set up the development environment (system dependencies, uv).
* [pid_config.json](pid_config.json): Persisted storage for the tuned PID parameters.
* [pyproject.toml](pyproject.toml): Python project configuration, dependencies, and tool settings.
* [setup.sh](setup.sh): Initial system setup script (I2C enablement, system libraries).
* [uv.lock](uv.lock): Dependency lock file to ensure reproducible installs.

### Source Code (`src/balance_bot`)
* [src/balance_bot/__init__.py](src/balance_bot/__init__.py): Package initialization.
* [src/balance_bot/battery.py](src/balance_bot/battery.py): Logic for monitoring battery voltage and estimating charge.
* [src/balance_bot/config.py](src/balance_bot/config.py): Centralized configuration (constants, dataclasses) for the robot.
* [src/balance_bot/diagnostics.py](src/balance_bot/diagnostics.py): Tools for checking system health (I2C, imports).
* [src/balance_bot/leds.py](src/balance_bot/leds.py): Controls the Raspberry Pi status LEDs for feedback.
* [src/balance_bot/main.py](src/balance_bot/main.py): Main application entry point and state machine controller.
* [src/balance_bot/mocks.py](src/balance_bot/mocks.py): Mock hardware implementations used when `MOCK_HARDWARE` env var is set.
* [src/balance_bot/piconzero.py](src/balance_bot/piconzero.py): Authoritative driver for the Picon Zero Motor HAT (modernized version of the 4tronix driver).
* [src/balance_bot/pid.py](src/balance_bot/pid.py): PID controller implementation for balance logic.
* [src/balance_bot/robot_hardware.py](src/balance_bot/robot_hardware.py): Hardware Abstraction Layer (HAL) managing sensors and motors.
* [src/balance_bot/tuner.py](src/balance_bot/tuner.py): Automatic PID tuning logic (Ziegler-Nichols method).
* [src/balance_bot/utils.py](src/balance_bot/utils.py): Shared utility functions (math, timing, filtering).
* [src/balance_bot/wiring_check.py](src/balance_bot/wiring_check.py): Interactive tool for verifying motor/sensor wiring and orientation.

### Tests (`tests/`)
* [tests/test_battery.py](tests/test_battery.py): Unit tests for battery estimation logic.
* [tests/test_config_migration.py](tests/test_config_migration.py): Tests for configuration loading and migration.
* [tests/test_diagnostics.py](tests/test_diagnostics.py): Tests for diagnostic functions.
* [tests/test_i2c_config.py](tests/test_i2c_config.py): Tests for I2C bus configuration logic.
* [tests/test_imu_logic.py](tests/test_imu_logic.py): Tests for IMU math and pitch calculation.
* [tests/test_tuner.py](tests/test_tuner.py): Tests for the PID auto-tuning logic.
* [tests/test_utils.py](tests/test_utils.py): Unit tests for helper functions (clamp, filters, etc.).

### Printed Parts (`printed_parts/`)
* [printed_parts/dcmotor.scad](printed_parts/dcmotor.scad): OpenSCAD source file for motor mount.
* [printed_parts/self_balancing_wheel.3mf](printed_parts/self_balancing_wheel.3mf): 3D model project file (3MF) for the wheel.
* [printed_parts/self_balancing_wheel_A_0.15mm_FLEX_MK3S_1h22m.gcode](printed_parts/self_balancing_wheel_A_0.15mm_FLEX_MK3S_1h22m.gcode): G-code for printing Wheel Part A.
* [printed_parts/self_balancing_wheel_B_0.15mm_FLEX_MK3S_1h22m.gcode](printed_parts/self_balancing_wheel_B_0.15mm_FLEX_MK3S_1h22m.gcode): G-code for printing Wheel Part B.
* [printed_parts/self_balancing_wheel_proj.3mf](printed_parts/self_balancing_wheel_proj.3mf): Alternative/Project 3MF file for the wheel.

## Getting Started (First Time Setup)

Follow these steps to get your robot up and running from a fresh Raspberry Pi installation.

### 1. Clone the Repository
```bash
git clone https://github.com/your-org/balance-bot.git
cd balance-bot
```

### 2. System Setup
Run the setup script to enable I2C and install necessary system libraries.
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
