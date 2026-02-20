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
