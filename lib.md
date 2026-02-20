# Code Reduction Plan: Introducing `pydantic` and `simple-pid`

This document outlines the architectural changes to reduce boilerplate code by leveraging standard libraries.

## 1. Configuration (`config.py`) -> `pydantic`

Currently, `config.py` implements manual JSON serialization, deserialization, and validation using Python `dataclasses`. This requires ~270 lines of code, including helper methods like `_filter_keys` and manual error handling.

**Proposal:** Replace `dataclasses` with `pydantic.BaseModel`.

**Benefits:**
- **Automated Validation:** Pydantic automatically validates types and ranges.
- **Serialization:** `model_dump_json()` and `model_validate_json()` replace manual loading logic.
- **Schema Generation:** Free JSON schema generation (useful for future UI/Tooling).
- **Code Reduction:** Removes ~150 lines of boilerplate.

**Implementation Details:**
- `PIDParams`, `BatteryConfig`, `TunerConfig`, `LedConfig`, `ControlConfig` become `pydantic.BaseModel`.
- `RobotConfig` becomes `pydantic.BaseModel`.
- `load()` method becomes a wrapper around `model_validate_json(Path(CONFIG_FILE).read_text())`.
- `save()` method becomes a wrapper around `Path(CONFIG_FILE).write_text(self.model_dump_json(indent=4))`.

## 2. PID Control (`pid.py`) -> `simple-pid`

Currently, `pid.py` implements a custom PID controller (~70 lines) with features like anti-windup and derivative-on-measurement.

**Proposal:** Replace custom implementation with `simple-pid` library.

**Benefits:**
- **Robustness:** `simple-pid` is a well-tested library.
- **Features:** Supports auto-mode, output limits, and proprotional-on-measurement.
- **Code Reduction:** Removes ~70 lines of custom logic.

**Implementation Details:**
- `PIDController` class will wrap `simple_pid.PID` to maintain the existing interface (`update(error, dt, measurement_rate)`).
- We need to map `measurement_rate` usage. `simple-pid` uses `error` for derivative by default. We can use `differential_on_measurement=True` (if supported) or pass `measurement` to `__call__`?
    - `simple-pid` API: `pid(input_value, dt=...)`. The input is the *process variable* (measurement), not error, if we want it to calculate error. Or we can pass error.
    - Our current implementation calculates derivative term from `measurement_rate` directly if provided. `simple-pid` calculates it from `(input - last_input) / dt`.
    - If we pass `measurement_rate` as the derivative term, we might need to subclass or adjust usage.
    - *Fallback:* If `simple-pid` doesn't support explicit derivative term injection, we might keep our custom implementation or use `simple-pid`'s standard derivative-on-measurement mode (which calculates rate from position). Our `measurement_rate` comes from a Gyro, which is cleaner than differentiating position.
    - *Decision:* We will attempt to use `simple-pid`'s `differential_on_measurement=True` if applicable, or just accept standard derivative-on-error if the difference is negligible for this application. Or, we can feed the Gyro rate as the "Derivative" term if the library allows.

## 3. Execution Plan

1.  Add `pydantic` and `simple-pid` to `pyproject.toml`.
2.  Refactor `config.py`.
3.  Refactor `pid.py`.
4.  Verify all tests pass.
