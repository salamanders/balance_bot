# AGENTS.md

1. This is for a self-balancing robot homebrew toy.
2. Always run a lint check before a commit to avoid unused variables and other annoying issues.
3. The hardware is bottom-tier: The motors may be uneven, the wheels are 3d printed, and the whole thing is held together with LEGOs. Excellent code is the only way it can be successful.
4. Keep the code concise.  Make it modern.  Use the latest python functions to make it more readable.
5. The src/balance_bot/hardware/piconzero.py file has extra functions (lights, sensors, etc). That is fine. Leave the extra code alone, only worry about the motor controlling code.
6. The emulator can't know what a real motor would do, so don't try to optimize calls to hardware that are untestable. Feel free to optimize other things, but the final I/O with the hardware has to stay constant. 
7. **Vector3**: Always use `balance_bot.utils.Vector3` for 3D spatial data. It is an immutable `NamedTuple` with full arithmetic support. Do not use dictionaries or raw tuples for vectors.

## The Physical Reality

**Important:** The robot is a two-wheeled self-balancing robot with a Segway-like topology (wheels side-by-side).

It has **Training Wheels / Bumpers** that extend to the front and back.
*   **Geometry:** These bumpers stick out at approximately 20 degrees from vertical (this varies but is strictly < 50 degrees).
*   **Resting State:** When powered off or "relaxed", the robot rests stably on either the front or back bumper. This is a **valid, safe state**, not a crash.
*   **Operating States:**
    1.  **Resting:** Powered off or idle, leaning on a bumper. Stable.
    2.  **Tricycle Mode:** Driving around while leaning on a bumper (front or back). Valid for non-balancing movement.
    3.  **Balancing:** The goal state. Upright, wheels under the Center of Mass, not touching bumpers.
    4.  **Crashed:** Face-planted BEYOND the bumpers (e.g., > 50 degrees). This is the only "failure" state requiring human intervention (or extreme recovery).

**Implication for Code:**
*   Do not treat a 20-degree lean as a "crash". It is a "rest".
*   The robot can transition from Resting to Balancing via a "Kick-Up" maneuver.

## Features

1. - [x] Implement a pyproject.toml with a make, lint, run. I think we should use "uv".
2. - [x] Don't assume I have a console open.  There will need to be other ways to communicate the setup with the user - like blinking the rpi zero light to indicate the setup phase.
3. - [x] It **will** fall over.  There should be an easy way to prop it back upright and have it resume.
4. - [x] It should **improve over time** as it gets more experience.
5. - [x] It should remember "last known good" PID variables and start from there.  Eventually we might be able to skip the calibration step?  Which also means a way to force a new calibration.
6. - [x] The battery levels will change over time, so that will be hard to account for.
7. - [x] Make a version of https://github.com/salamanders/mecanum/blob/main/AUTO_RUN.md that doesn't wait for WiFi.
8. - [x] Make a version of https://github.com/salamanders/mecanum/blob/main/wiring_check.py that handles both wheels, and then the gyro directions.

## Testing and Mocks
- If you need to test the logic without hardware (e.g., in a CI environment or on your laptop), use `uv run balance-bot --allow-mocks`.
- The code is configured to crash loudly if hardware is missing, unless this flag is passed.

## Code Quality Standards

### Role

You are a Senior Python Robotics Engineer specializing in code maintainability and stability.

### Objective

Refactor the existing codebase to strictly align with modern Python 3.14+ standards and specific
maintainability rules.

### Constraint

The "Do No Harm" Rule:

* You are processing this code in a stateless vacuum.
* Do not make stylistic changes just for the sake of novelty.
* Only apply a change if it objectively moves the code closer to the "Target State" defined below.
* If a piece of code already meets these standards, leave it exactly as is.
* **Do not alter the runtime logic or physics constants of the robot.**

### Target State Instructions

#### Type Hinting

Ensure all function signatures have standard Python type hints. If a complex type is used (like a
dictionary of sensor data), use typing or dataclasses to define it explicitly.

#### Modernize

* Replace "magic number" limits and hacky logic with min(), max(), or clamp functions.
* Use f-strings over concatenation.
* Be careful with blocking calls that might slow down the control loop.

#### Config Extraction

If you see physics constants (PID values, motor limits, wait times) hardcoded in logic functions,
move them to a CONSTANTS section at the top of the file or a dedicated configuration object.

#### Descriptive Naming

Rename single-letter variables (e.g., x, t, v) to descriptive terms relevant to self-balancing
robotics (e.g., tilt_angle, loop_delta_time, velocity_target) unless they are standard loop iterators like i.

#### Docstrings

Ensure every function has a docstring explaining inputs, outputs, and side effects (especially important for
hardware GPIO calls). Docs should clearly show "why this code has clean separation of concerns from other code"

### Output

Provide the refactored code files. If no significant changes are needed for a file based on these strict
criteria, output the file unchanged.

Leave yourself notes on **why** you made a change in the code. e.g. "Consolidated starting constants to persist across
reboots" is a great way to explain why something was refactored.
