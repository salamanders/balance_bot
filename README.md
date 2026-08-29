# balance_bot

Self-balancing homebrew Segway robot powered by a Raspberry Pi, 4tronix PiconZero HAT, MPU-6050 IMU, and 3D-printed/LEGO mechanical topology.

> **Note for Developers and Autonomous Agents:**
> * For engineering governance, LLM directives, Multi-Mind subsumption architecture, Tabula Rasa self-discovery, empirical hardware rules, and codebase assumptions, see [AGENTS.md](file:///Users/benhill/Desktop/hobbies/balance_bot/AGENTS.md).
> * For the chronological log of physical experiments, hardware audits, bug investigations, and empirical lessons learned, see [JOURNAL.md](file:///Users/benhill/Desktop/hobbies/balance_bot/JOURNAL.md).

---

## References

### Root Directory

* [.gitignore](file:///Users/benhill/Desktop/hobbies/balance_bot/.gitignore): Git configuration specifying which files to ignore.
* [.python-version](file:///Users/benhill/Desktop/hobbies/balance_bot/.python-version): Specifies the Python version used by the project/tools.
* [AGENTS.md](file:///Users/benhill/Desktop/hobbies/balance_bot/AGENTS.md): Comprehensive engineering manual, LLM governance rules, system architecture, systemd auto-run instructions, and blind spot documentation.
* [JOURNAL.md](file:///Users/benhill/Desktop/hobbies/balance_bot/JOURNAL.md): Chronological engineering log of physical experiments, empirical hardware audits, bug investigations, and learnings.
* [LICENSE](file:///Users/benhill/Desktop/hobbies/balance_bot/LICENSE): Project license file.
* [Makefile](file:///Users/benhill/Desktop/hobbies/balance_bot/Makefile): Shortcut commands for installation, linting, formatting, and running.
* [README.md](file:///Users/benhill/Desktop/hobbies/balance_bot/README.md): The main entry point documentation.
* [environment.sh](file:///Users/benhill/Desktop/hobbies/balance_bot/environment.sh): Script to set up the development environment (system dependencies, uv).
* `pid_config.json`: Persisted storage for the tuned PID parameters.
* [pyproject.toml](file:///Users/benhill/Desktop/hobbies/balance_bot/pyproject.toml): Python project configuration, dependencies, and tool settings.
* [setup.sh](file:///Users/benhill/Desktop/hobbies/balance_bot/setup.sh): Initial system setup script (I2C enablement, system libraries).
* [uv.lock](file:///Users/benhill/Desktop/hobbies/balance_bot/uv.lock): Dependency lock file to ensure reproducible installs.

### Source Code (`src/balance_bot`)

**Core & Utilities**
* [src/balance_bot/__init__.py](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/__init__.py): Package initialization.
* [src/balance_bot/configuration.py](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/configuration.py): Centralized configuration (constants, dataclasses) for the robot.
* [src/balance_bot/enums.py](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/enums.py): Enumerations for direction and orientation.
* [src/balance_bot/jules_client.py](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/jules_client.py): Client for reporting crashes to Jules.
* [src/balance_bot/main.py](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/main.py): Main application entry point.
* [src/balance_bot/utils.py](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/utils.py): Shared utility functions (math, timing, filtering).
* [src/balance_bot/wiring_check.py](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/wiring_check.py): Interactive tool for verifying motor/sensor wiring and orientation.

**Tier 1: Reflex (Brainstem)**
* [src/balance_bot/reflex/balance_core.py](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/reflex/balance_core.py): The high-frequency (100Hz) balance loop. Pure physics/math.
* [src/balance_bot/reflex/pid.py](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/reflex/pid.py): PID controller implementation.

**Tier 2: Adaptation (Cerebellum)**
* [src/balance_bot/adaptation/recovery.py](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/adaptation/recovery.py): Soft-start logic for recovering from a crash.
* [src/balance_bot/adaptation/tuner.py](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/adaptation/tuner.py): Automatic PID tuning logic (Continuous Tuner) and Balance Point Finder.
* [src/balance_bot/adaptation/battery.py](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/adaptation/battery.py): Logic for monitoring battery voltage and estimating charge.

**Tier 3: Behavior (Cortex)**
* [src/balance_bot/behavior/agent.py](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/behavior/agent.py): The "Brain" that orchestrates the robot. Handles scheduling, state management, and high-level goals.
* [src/balance_bot/behavior/leds.py](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/behavior/leds.py): Controls the Raspberry Pi status LEDs for feedback.

**Hardware Abstraction**
* [src/balance_bot/hardware/robot_hardware.py](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/hardware/robot_hardware.py): Hardware Abstraction Layer (HAL) managing sensors and motors.
* [src/balance_bot/hardware/piconzero.py](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/hardware/piconzero.py): Driver for the Picon Zero Motor HAT.
* [src/balance_bot/hardware/piconzero_adapter.py](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/hardware/piconzero_adapter.py): Adapter for Picon Zero to manage I2C retries and bus switching.
* [src/balance_bot/hardware/mocks.py](file:///Users/benhill/Desktop/hobbies/balance_bot/src/balance_bot/hardware/mocks.py): Mock hardware implementations.

### Tests (`tests/`)
* [tests/benchmarks/](file:///Users/benhill/Desktop/hobbies/balance_bot/tests/benchmarks): Benchmark tests for performance critical components.
* [tests/test_balance_core_params.py](file:///Users/benhill/Desktop/hobbies/balance_bot/tests/test_balance_core_params.py): Tests for balance core parameters.
* [tests/test_balance_finder.py](file:///Users/benhill/Desktop/hobbies/balance_bot/tests/test_balance_finder.py): Tests for the Balance Point Finder logic.
* [tests/test_battery.py](file:///Users/benhill/Desktop/hobbies/balance_bot/tests/test_battery.py): Unit tests for battery estimation logic.
* [tests/test_i2c_config.py](file:///Users/benhill/Desktop/hobbies/balance_bot/tests/test_i2c_config.py): Tests for I2C bus configuration logic.
* [tests/test_imu_logic.py](file:///Users/benhill/Desktop/hobbies/balance_bot/tests/test_imu_logic.py): Tests for IMU math and pitch calculation.
* [tests/test_imu_resilience.py](file:///Users/benhill/Desktop/hobbies/balance_bot/tests/test_imu_resilience.py): Tests for IMU error handling and resilience.
* [tests/test_imu_yaw_roll.py](file:///Users/benhill/Desktop/hobbies/balance_bot/tests/test_imu_yaw_roll.py): Tests for yaw and roll calculations.
* [tests/test_kickup_config.py](file:///Users/benhill/Desktop/hobbies/balance_bot/tests/test_kickup_config.py): Tests for kick-up configuration and logic.
* [tests/test_piconzero_driver.py](file:///Users/benhill/Desktop/hobbies/balance_bot/tests/test_piconzero_driver.py): Tests for the Picon Zero driver.
* [tests/test_startup_logic.py](file:///Users/benhill/Desktop/hobbies/balance_bot/tests/test_startup_logic.py): Tests for the robot startup sequence.
* [tests/test_tuner.py](file:///Users/benhill/Desktop/hobbies/balance_bot/tests/test_tuner.py): Tests for the PID auto-tuning logic.
* [tests/test_utils.py](file:///Users/benhill/Desktop/hobbies/balance_bot/tests/test_utils.py): Unit tests for helper functions (clamp, filters, etc.).
* [tests/test_wiring_check_bus.py](file:///Users/benhill/Desktop/hobbies/balance_bot/tests/test_wiring_check_bus.py): Tests for the wiring check tool's bus detection.

### Printed Parts (`printed_parts/`)
* [printed_parts/dcmotor.scad](file:///Users/benhill/Desktop/hobbies/balance_bot/printed_parts/dcmotor.scad): OpenSCAD source file for motor mount.
* [printed_parts/tpu_wheel.scad](file:///Users/benhill/Desktop/hobbies/balance_bot/printed_parts/tpu_wheel.scad): OpenSCAD source file for TPU wheel.
* [printed_parts/wheel_v3.3mf](file:///Users/benhill/Desktop/hobbies/balance_bot/printed_parts/wheel_v3.3mf): 3D model project file (3MF) for the wheel (v3).
* [printed_parts/wheel_v3.scad](file:///Users/benhill/Desktop/hobbies/balance_bot/printed_parts/wheel_v3.scad): OpenSCAD source file for wheel v3.
* [printed_parts/wheel_v3_project.3mf](file:///Users/benhill/Desktop/hobbies/balance_bot/printed_parts/wheel_v3_project.3mf): Alternative/Project 3MF file for the wheel (v3).

---

## Getting Started (First Time Setup)

Follow these steps to get your robot up and running from a fresh Raspberry Pi installation.

### 0. Physical Wiring Check

* **MPU-6050**: VCC to 3.3V, GND to GND, SDA to SDA, SCL to SCL.
* **Motors**: Plugged into Motor A and Motor B on the Picon Zero.

If you have a Picon Zero HAT, it blocks standard I2C pins. You can add a software-defined "bus 3" using reachable GPIO pins:
1. Edit boot firmware configuration:
   ```bash
   sudo nano /boot/firmware/config.txt
   ```
2. Scroll to the bottom and add this exact line:
   ```txt
   dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=17,i2c_gpio_scl=27,i2c_gpio_delay_us=5
   ```
   *(Note: `i2c_gpio_delay_us=5` sets the bus speed to approximately 100kHz. Increase to `10` if you still encounter I2C errors.)*

### 1. Clone the Repository
```bash
git clone https://github.com/salamanders/balance_bot.git
cd balance-bot
```

### 2. Install `uv`
This project uses `uv` for Python dependency management:
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### 3. System Setup
Run the setup script to enable I2C and install necessary system libraries:
```bash
chmod +x setup.sh
sudo ./setup.sh
```

### 4. Install Dependencies
```bash
uv sync
```

### 5. Verify & Run (Zero-Knowledge Self-Discovery)
Run the **Zero-Knowledge Self-Discovery Protocol**. This state-machine wizard automatically discovers your robot's unique motor wiring, polarity, MPU orientation, and friction thresholds on first boot:
```bash
uv run balance-bot
```
* The wizard is **pessimistic**: it verifies every discovery empirically before saving to `pid_config.json`.
* Once self-discovery completes, the main control loop wakes up and begins balancing automatically.
* For detailed documentation of the 7 self-discovery phases, see [AGENTS.md Section 5](file:///Users/benhill/Desktop/hobbies/balance_bot/AGENTS.md#5-the-tabula-rasa-protocol-zero-knowledge-self-discovery).

---

## CLI Execution & Flags

* `uv run balance-bot` — Runs standard self-discovery wizard or active balance control loop.
* `uv run balance-bot --deadman` — Enables the HTTP Deadman's Switch web interface (`http://<robot-ip>:8090`) for living room safety and remote data fetching.
* `uv run balance-bot --experiment-duration 4.0` — Enforces a global experiment timeout budget (e.g., 4.0s) across all phases.
* `uv run balance-bot --allow-mocks` — Runs control loop using Mock Hardware (required for CI / laptop testing without physical sensors/motors).
* `uv run balance-bot --reset-brain` — Wipes learned `pid_config.json` and forces re-running Tabula Rasa discovery.
* `uv run balance-bot --auto-fix` — Enables automated crash reporting and traceback submission to Jules.
* `uv run balance-bot --repl` — Launches interactive REPL for manual motor control, IMU reads, and gyro tests.

### Remote HTTP Data API (Port 8090)
When `--deadman` is active, the robot serves the following data endpoints over HTTP:
* `GET /flight_data.csv` (or `/data/flight`) — Raw 100Hz flight telemetry CSV.
* `GET /discovery_data.csv` (or `/data/discovery`) — Raw Tabula Rasa discovery calibration CSV.
* `GET /learning_state.json` (or `/data/learning_state`) — Current learned controller parameters and bumper angles.
* `GET /hardware_config.json` (or `/data/hardware_config`) — Physical hardware axis/bus mappings.
* `GET /data/analysis` — Automated blackbox forensic analysis JSON (kickup session breakdown, pulse metrics, failure classification).
* `GET /status` — Real-time pitch, posture, and deadman arming status.


---

## System Architecture, Governance & Empirical Rules

To eliminate duplication across documentation files, all system architecture details, multi-mind subsumption topology, AI agent governance directives, empirical hardware rules, systemd daemon deployment, and codebase assumptions are consolidated in [AGENTS.md](file:///Users/benhill/Desktop/hobbies/balance_bot/AGENTS.md):

* **[Project Philosophy & Hardware Reality](file:///Users/benhill/Desktop/hobbies/balance_bot/AGENTS.md#1-project-philosophy--hardware-reality)** — Bumpers, resting state vs. crash angle, kick-up maneuvers, and persistence.
* **[Mandatory LLM Directives & Code Standards](file:///Users/benhill/Desktop/hobbies/balance_bot/AGENTS.md#2-mandatory-llm-directives--code-standards)** — Pre-flight checks, type safety, SDD/RLVR, and anti-patterns.
* **[Mandatory Journaling Protocol](file:///Users/benhill/Desktop/hobbies/balance_bot/AGENTS.md#3-mandatory-journaling-protocol-journalmd)** — Requirements and formatting for logging experiments in `JOURNAL.md`.
* **[System Architecture & Multi-Mind Topology](file:///Users/benhill/Desktop/hobbies/balance_bot/AGENTS.md#4-system-architecture--multi-mind-topology)** — Tier 1 Brainstem (`100Hz`), Tier 2 Cerebellum (`~10Hz`), Tier 3 Cortex.
* **[Tabula Rasa Zero-Knowledge Bootstrapping](file:///Users/benhill/Desktop/hobbies/balance_bot/AGENTS.md#5-the-tabula-rasa-protocol-zero-knowledge-self-discovery)** — 7-phase self-discovery sequence and future Toddler Engine.
* **[Critical Cyber-Physical Blind Spots](file:///Users/benhill/Desktop/hobbies/balance_bot/AGENTS.md#6-critical-cyber-physical-blind-spots)** — Comprehensive comparison table of LLM biases vs. physical constraints.
* **[Immutable Empirical Hardware Rules](file:///Users/benhill/Desktop/hobbies/balance_bot/AGENTS.md#7-immutable-empirical-hardware-rules)** — PiconZero I2C register retention trap, LEGO mechanical backlash vs. `Kp`, tilt-chasing resonance in soft recovery, and the global 4-second chaos cutoff rule.
* **[Systemd Deployment & Automatic Startup](file:///Users/benhill/Desktop/hobbies/balance_bot/AGENTS.md#8-operations-systemd-deployment--manual-repl)** — Configuring `/etc/systemd/system/balance-bot.service` for auto-run on boot.
* **[Codebase Assumptions & Reference Citations](file:///Users/benhill/Desktop/hobbies/balance_bot/AGENTS.md#9-codebase-assumptions--reference-citations)** — I2C addresses, motor channels A/B, loop frequency, fall limit, default PID gains, and LED paths.
