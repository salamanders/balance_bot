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

**Core & Utilities**
* [src/balance_bot/__init__.py](src/balance_bot/__init__.py): Package initialization.
* [src/balance_bot/config.py](src/balance_bot/config.py): Centralized configuration (constants, dataclasses) for the robot.
* [src/balance_bot/diagnostics.py](src/balance_bot/diagnostics.py): Tools for checking system health (I2C, imports).
* [src/balance_bot/main.py](src/balance_bot/main.py): Main application entry point.
* [src/balance_bot/utils.py](src/balance_bot/utils.py): Shared utility functions (math, timing, filtering).
* [src/balance_bot/wiring_check.py](src/balance_bot/wiring_check.py): Interactive tool for verifying motor/sensor wiring and orientation.

**Tier 1: Reflex (Brainstem)**
* [src/balance_bot/reflex/balance_core.py](src/balance_bot/reflex/balance_core.py): The high-frequency (100Hz) balance loop. Pure physics/math.
* [src/balance_bot/reflex/pid.py](src/balance_bot/reflex/pid.py): PID controller implementation.

**Tier 2: Adaptation (Cerebellum)**
* [src/balance_bot/adaptation/recovery.py](src/balance_bot/adaptation/recovery.py): Soft-start logic for recovering from a crash.
* [src/balance_bot/adaptation/tuner.py](src/balance_bot/adaptation/tuner.py): Automatic PID tuning logic (Continuous Tuner) and Balance Point Finder.
* [src/balance_bot/adaptation/battery.py](src/balance_bot/adaptation/battery.py): Logic for monitoring battery voltage and estimating charge.

**Tier 3: Behavior (Cortex)**
* [src/balance_bot/behavior/agent.py](src/balance_bot/behavior/agent.py): The "Brain" that orchestrates the robot. Handles scheduling, state management, and high-level goals.
* [src/balance_bot/behavior/leds.py](src/balance_bot/behavior/leds.py): Controls the Raspberry Pi status LEDs for feedback.

**Hardware Abstraction**
* [src/balance_bot/hardware/robot_hardware.py](src/balance_bot/hardware/robot_hardware.py): Hardware Abstraction Layer (HAL) managing sensors and motors.
* [src/balance_bot/hardware/piconzero.py](src/balance_bot/hardware/piconzero.py): Driver for the Picon Zero Motor HAT.
* [src/balance_bot/hardware/mocks.py](src/balance_bot/hardware/mocks.py): Mock hardware implementations.

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

## Architecture: Multi-Mind (Subsumption)

The robot control is divided into three "Minds" running at different frequencies and responsibilities.

### 1. The Brainstem (Reflex) - Tier 1
*   **Goal**: Stay Vertical.
*   **Frequency**: 100Hz.
*   **Responsibility**: Reads IMU, runs PID, sets Motors.
*   **Characteristics**: Stateless, Deterministic, Safety-Critical.

### 2. The Cerebellum (Adaptation) - Tier 2
*   **Goal**: Optimize Mechanics.
*   **Frequency**: ~10Hz (Subsampled).
*   **Responsibility**: Analyzes performance (oscillation, drift) and tweaks PID gains or Balance Point. Handles "Soft Start" recovery.

### 3. The Cortex (Behavior) - Tier 3
*   **Goal**: Intent & Orchestration.
*   **Frequency**: Low Frequency / Event Driven.
*   **Responsibility**: Manages the Agent lifecycle, LEDs, Config saving, and future Navigation goals.

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

### 6. Confirm Movement (Square Test)
Before running the full balancing logic, verify that the robot can move correctly and the gyro axis is correct by running a simple square pattern.
**Note:** Ensure the robot is resting on its back training wheel on the floor.
```bash
uv run balance-bot --confirm-wiring
```

### 7. Run the Robot
Start the balancing software.
```bash
uv run balance-bot
```

## CLI Reference

The robot application supports several command-line arguments to assist with setup, debugging, and development.

*   **`--check-wiring`**
    Runs an interactive wizard to verify motor direction and gyro orientation. Essential for first-time setup.
    ```bash
    uv run balance-bot --check-wiring
    ```

*   **`--confirm-wiring`**
    Executes a predefined movement pattern (Square Test) to validate that hardware axes are correctly mapped before attempting to balance.
    ```bash
    uv run balance-bot --confirm-wiring
    ```

*   **`--diagnose`**
    Runs a suite of system diagnostics to check for common issues, including:
    *   Python package imports.
    *   System I2C configuration (config.txt overlays).
    *   I2C bus permissions and access (smbus2).
    *   Device connectivity (i2cdetect).
    ```bash
    uv run balance-bot --diagnose
    ```

*   **`--auto-fix`**
    Enables automated crash reporting. If the robot crashes, this flag captures the traceback, logs, configuration state, and library versions, then submits a report to Jules for analysis.
    ```bash
    uv run balance-bot --auto-fix
    ```

*   **`--allow-mocks`**
    Forces the use of Mock Hardware. This allows you to run the control loop and logic on a laptop or device without physical sensors/motors. Useful for development and testing.
    ```bash
    uv run balance-bot --allow-mocks
    ```

---

## Physical Wiring Check
*Note: This section describes the physical connections required before running the software.*

* **MPU-6050**: VCC to 3.3V, GND to GND, SDA to SDA, SCL to SCL.
* **Motors**: Plugged into Motor A and Motor B on the Picon Zero.

If you have a Picon Zero hat - it gets in the way!  But fear not, you can use add a "bus 3" that uses some of the reachable GPIO pins.

`sudo nano /boot/firmware/config.txt`

Scroll to the bottom and add this exact line:

```txt
dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=17,i2c_gpio_scl=27
```

**Orientation**: This code assumes the MPU is mounted such that tilting forward/back rotates the X-axis gyroscope. The check-wiring will confirm this.

## Bootup Process

1. **Place the robot on its back training wheel.**
2. Run `uv run balance-bot`.

### First Run (Calibration)
If this is the first time running (or no config exists):
1. The robot will perform a **Self-Calibration Sequence** ("The Flop").
   - It will flop forward, then backward to measure its physical limits.
   - It will calculate the mechanical balance point.
2. Once calibrated, it will perform a **Kick-Up** maneuver to stand up and balance.

### Normal Run
1. The robot will detect it is resting on the back wheel.
2. It will perform a **Kick-Up** maneuver using the saved configuration.
3. It will begin balancing.

*Note: You can also start the robot holding it upright, and it will balance immediately.*

## Codebase Assumptions

This code relies on several hardware, physical, and configuration assumptions. If your robot differs, you may need to adjust these values in the cited files.

### Hardware Configuration
*   **I2C Addresses**: Assumes MPU-6050 is at `0x68` and Picon Zero HAT is at `0x22`.
    *   *Citation*: [src/balance_bot/hardware/robot_hardware.py](src/balance_bot/hardware/robot_hardware.py) (MPU), [src/balance_bot/hardware/piconzero.py](src/balance_bot/hardware/piconzero.py) (PiconZero `I2C_ADDRESS`).
*   **Motor Channels**: Assumes Left Motor is Channel 0 and Right Motor is Channel 1.
    *   *Citation*: [src/balance_bot/config.py](src/balance_bot/config.py) (`motor_l`, `motor_r` in `RobotConfig`).
*   **Motor Input Range**: Assumes motor driver accepts values from -100 to 100.
    *   *Citation*: [src/balance_bot/hardware/robot_hardware.py](src/balance_bot/hardware/robot_hardware.py) (`MOTOR_MIN_OUTPUT`, `MOTOR_MAX_OUTPUT`).

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
    *   *Citation*: [src/balance_bot/behavior/leds.py](src/balance_bot/behavior/leds.py) (`_find_led_path`).
