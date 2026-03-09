# Actual Blind Spots

Based on the explicit architectural warnings in `BLIND_SPOTS.md`, the following classes and functions suffer from severe physical and temporal vulnerabilities that an LLM might misinterpret as refactoring opportunities.

## 1. Zero-Knowledge Bootstrapping
**Vulnerability:** Calibration thresholds depend heavily on floor friction. Environmental edge cases (uneven terrain, wheel slip) are unhandled. Do not bypass or hardcode these discovery steps.
**Files:** `src/balance_bot/discovery/steps.py`, `src/balance_bot/discovery/pipeline.py`
* `FrictionThresholdStep.run`
* `DeriveKinematicsStep.run`
* `DeriveKinematicsStep._pulse_and_measure`
* `MotorTrimStep.run`
* `MechanicalBacklashStep.run`
* `KickupDynamicsStep.run`
* `KickupDynamicsStep._force_posture`
* `KickupDynamicsStep._attempt_kick`
* `KickupDynamicsStep._run_kickup_test`
* `SelfDiscoveryPipeline.run`

## 2. Temporal Determinism (The Brainstem)
**Vulnerability:** The 100Hz reflex loop lacks strict performance benchmarks or jitter tracking. Any added latency, garbage collection, or async overhead will cause physical instability.
**Files:** `src/balance_bot/reflex/balance_core.py`, `src/balance_bot/reflex/pid.py`
* `BalanceCore.update`
* `PIDController.update`

## 3. Hardware Communication Nuances
**Vulnerability:** I2C error handling relies on naive retries and lacks advanced recovery like physical bus power resets. Mock implementations fail to simulate real-world erratic latency or Byte Write restrictions.
**Files:** `src/balance_bot/hardware/piconzero.py`, `src/balance_bot/hardware/robot_hardware.py`
* `PiconZero._retry`
* `PiconZero.init`
* `PiconZero.cleanup`
* `PiconZero.set_motor`
* `PiconZero.set_motors`
* `RobotHardware.read_imu_raw`
* `RobotHardware.read_imu_converted`
* `RobotHardware.set_motors`
* `RobotHardware.initialize_drivers`

## 4. Physical Tuning (The Unverified Zone)
**Vulnerability:** The Tuner operates entirely open-loop without absolute ground truth from a secondary sensor. There are no explicit fallbacks to prevent runaway oscillation loops caused by bad state estimation.
**Files:** `src/balance_bot/adaptation/tuner.py`, `src/balance_bot/adaptation/recovery.py`
* `ContinuousTuner.update`
* `ContinuousTuner._count_zero_crossings`
* `BalancePointFinder.update`
* `RecoveryManager.update`
