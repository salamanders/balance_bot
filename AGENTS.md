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

# The "Tabula Rasa" Protocol: Zero-Knowledge Self-Discovery

## Philosophy & Context

This robot is a homebrew project built with bottom-tier hardware. The motors may be mismatched, the 3D-printed wheels
might be uneven, and the structural integrity relies on zip-ties, duct tape, chewing gum, and a can-do attitude. The IMU
might be mounted at a completely bizarre angle.

**Crucially, the robot is equipped with Front and Back Training Wheels (Bumpers).** These allow it to rest safely at an
angle (approx. 20°) without crashing. This defines the robot's existence: it is either **Resting** (safe, on a bumper),
**Balancing** (precarious, upright), or **Crashed** (rare failure, face-planted beyond the bumpers).

Because the physical form is unpredictable and the robot must operate autonomously, we cannot rely on hardcoded
constants, and **human intervention is a failure of logic.**

The robot wakes up with **Tabula Rasa** (a Blank Slate). It must deduce its entire physical configuration through
strict, pessimistic physics experiments.

**For Jules (the AI implementing this):** Treat this as a strictly sequential state-machine. Every step follows a logic
grid:

1. **What do I know?** (Established facts/axioms)
2. **What am I uncertain of?** (The current mystery)
3. **Action:** (The physical experiment to run)
4. **Deduction:** (The inescapable mathematical/physical conclusion)

If an experiment fails or yields ambiguous data, the state machine must safely stop, reverse the experiment's polarity
or increase the threshold, and try again. If it applies a fix (like inverting a motor), it must re-run the experiment to
verify the fix worked before proceeding.

---

## Phase 1: The Spark of Life (Hardware Presence)

* **What do I know?** I am executing code on a Raspberry Pi.
* **What am I uncertain of?** Do I have physical sensors and motor drivers attached? What I2C buses are they on?
* **Action:** Scan all available I2C buses (including software-defined `bus=3` on GPIOs). Look for the MPU-6050 (IMU at
  `0x68`) and Picon Zero (Motor Driver at `0x22`). Read the `WHO_AM_I` register on the IMU.
* **Deduction:** If successful, I lock in the I2C bus IDs and establish communication. If I fail, I flash an error LED
  and halt—I cannot solve a physical puzzle without a body.

> **Implementation Status:** Implemented.
> **Code Correlation:** `src/balance_bot/wiring_check.py` in `discover_buses` method.

---

## Phase 2: The Sense of Down (Gravity)

* **What do I know?** My sensors work and I can read them. I am resting statically on a physical bumper (front or back).
* **What am I uncertain of?** How is my IMU mounted? Which way is Down?
* **Action:** Ensure motors are strictly OFF. Take 100 readings from the 3-axis Accelerometer over 1 second and average
  the vectors.
* **Deduction:** Gravity dominates static acceleration. The normalized unit vector of this average reading is exactly
  the **Down Vector ($\vec{D}$)** in my arbitrary internal 3D coordinate space. Its inverse is the **Up
  Vector ($\vec{U}$)**.
    * *Note: The robot is TILTED in this state. This is normal.*

> **Implementation Status:** Implemented.
> **Code Correlation:** `src/balance_bot/wiring_check.py` in `calibrate_static_orientation` method.

---

## Phase 3: The Will to Move (Stiction & Phasing)

* **What do I know?** I know $\vec{D}$ (Down). I have two motor ports: Port A and Port B.
* **What am I uncertain of?** How much power is required to overcome the mechanical friction of my cheap motors? Are
  Port A and Port B wired in phase, or are they fighting each other?
* **Action:**
    1. **Stiction:** Slowly ramp PWM from 0% upwards simultaneously on both ports until the Gyro detects a sudden spike
       in rotation or vibration. Record this as `min_pwm`.
    2. **Phasing:** Apply a short pulse of `min_pwm + 10%` to *both* Port A and Port B in the same mathematical
       direction (+). Monitor the Gyro rotation around the Up Vector ($\vec{U}$).
* **Deduction:**
    * If the robot spins rapidly around the Up Vector (high Yaw rate), the wheels are spinning in opposite directions.
      Port B is out of phase with Port A.
        * **Fix:** Logically invert Port B in software. **Re-run the phasing test** to verify the Yaw rate is now near
          zero.
    * *New Knowledge: I know my deadband (`min_pwm`), and my motors now definitively push together.*

> **Implementation Status:** Implemented.
> **Code Correlation:** `src/balance_bot/wiring_check.py` in `find_min_power` and `align_motors_phase` methods.

---

## Phase 4: The Sense of Forward (Pitch Axis & Polarity)

* **What do I know?** $\vec{D}$ (Down), `min_pwm`, and that my motors push together. I am leaning on a bumper.
* **What am I uncertain of?** Which motor polarity means "Forward" (driving *under* the center of mass to stand up)?
  Which IMU axis measures my Pitch?
* **Action:**
    1. **Orientation:** Lean the robot **FORWARD** (rest on its front bumper). This is critical: Positive Power is defined
       as the direction that stands the robot up *from a forward lean*.
    2. **Pulse:** Apply a brief, strong pulse (+50%) to *both* motors in the Positive direction. Observe the Accelerometer
       and Gyroscope.
* **Deduction:**
    * The Gyro rotation vector during this lurch is definitively the **Pitch Axis ($\vec{P}$)**.
    * Look at the angle between the Up vector and the accelerometer.
    * If the pitch angle *decreases* (the chassis moves toward vertical from the front bumper), the wheels drove *under* the
      leaning center of mass. This is exactly what "Forward" means for a balancing robot. Positive = Forward.
    * If the pitch angle *increases* (the chassis slams harder into the bumper), the wheels drove *away* from the center
      of mass (Backward).
        * **Fix:** Invert *both* global motor polarities in software. **Re-run the test** to verify positive PWM causes
          the robot to pitch up.
    * *New Knowledge: I have my absolute 3D orientation ($\vec{D}$ and $\vec{P}$) and I know how to drive Forward.*

> **Implementation Status:** Implemented.
> **Code Correlation:** `src/balance_bot/wiring_check.py` in `determine_motor_direction` method.

---

## Phase 5: Absolute Identity (Left vs. Right via Right-Hand Rule)

* **What do I know?** My 3D orientation ($\vec{D}$ and $\vec{P}$), how to drive Forward, and that my motors are in
  phase.
* **What am I uncertain of?** Which motor is physically on my Left, and which is on my Right? (We must deduce this
  without human eyes).
* **Action:** Command Port A to drive **Forward** (+PWM) and Port B to drive **Backward** (-PWM). Observe the resulting
  Gyro rotation vector ($\vec{\omega}$).
* **Deduction:**
    * Because Port A is driving Forward and Port B is Backward, the robot will spin in place (Yaw).
    * The IMU is a silicon chip that obeys the mathematical Right-Hand Rule (RHR).
    * Point your right thumb in the direction of the **Up Vector ($\vec{U}$)**. Your fingers curl Counter-Clockwise (a
      Left turn).
    * Therefore, if the Gyro vector $\vec{\omega}$ points *Up* (positive dot product with $\vec{U}$), the robot is
      spinning Counter-Clockwise (Left).
    * If driving Port A Forward causes a **Left spin**, then Port A must be pushing the Right side of the robot forward.
      Therefore, **Port A is the Right Motor**, and Port B is the Left Motor.
    * If $\vec{\omega}$ points *Down*, the robot is spinning Right. **Port A is the Left Motor**, and Port B is the
      Right Motor.
    * *New Knowledge: I have mathematically mapped my left and right motor channels with ZERO human intervention.*

> **Implementation Status:** Implemented.
> **Code Correlation:** `src/balance_bot/wiring_check.py` in `deduce_left_right_autonomous` method.

---

## Phase 6: The Stride (Motor Trimming)

* **What do I know?** I know exactly how to navigate physical space correctly.
* **What am I uncertain of?** Are my 3D-printed wheels exactly the same size? Do my cheap DC motors have perfectly
  matched Kv ratings? (Hint: They don't).
* **Action:** Drive both motors **Forward** at identical PWM for 0.5 seconds in an open-loop burst.
* **Deduction:**
    * Integrate the Gyro rotation around the Up vector ($\vec{U}$) to measure Yaw drift.
    * If the robot naturally drifted Left, the Right motor is physically stronger or has a larger wheel.
    * **Fix:** Calculate a fractional scaling multiplier (e.g., `Right_Scale = 0.92`) and apply it permanently to the
      stronger motor in the software mixer. **Re-run the test** until the robot drives perfectly straight.
    * *New Knowledge: Perfect linear physical output mapping.*

> **Implementation Status:** Implemented.
> **Code Correlation:** `src/balance_bot/wiring_check.py` in `calibrate_motor_trim` method.

---

## Phase 7: The Balance Point (Center of Mass)

* **What do I know?** I am perfectly physically mapped and trimmed.
* **What am I uncertain of?** Because of battery placement and zip-ties, my physical Center of Mass (CoM) probably isn't
  perfectly aligned with the wheel axles. If I try to balance at an angle of 0.0°, I might constantly drift.
* **Action:** Engage the 100Hz PID balance loop and execute a "Kick-Up" maneuver to transition from Resting (on bumper)
  to Balancing. Attempt to hover in place with a `target_angle = 0.0°`.
* **Deduction:**
    * While balancing, monitor the Integral (I) term of the PID, or the average motor PWM over a 3-second rolling
      window.
    * If I have to constantly drive Forward at 15% power just to not fall over, I am "nose heavy".
    * **Fix:** Slowly shift the PID `target_angle` backward (e.g., to -2.0°) until the average motor power required to
      hover in place is exactly `0%`.
    * *New Knowledge: I have discovered my true mechanical Balance Point Offset.*

> **Implementation Status:** Implemented (Continuous Learning).
> **Code Correlation:** `src/balance_bot/wiring_check.py` handles Kick-Up (`find_flop_thresholds`), while `src/balance_bot/adaptation/tuner.py` handles continuous balance point tuning via `BalancePointFinder`.

---

## Output & Persistence

Upon successful completion of Phase 7, the robot has achieved total self-awareness. It flashes the Pi's status LED
rapidly in a success pattern and writes a `pid_config.json` file to disk containing:

1. Validated `i2c_bus` IDs and `sensor_addresses`.
2. `axis_map`: The vector matrices defining the Up/Down and Pitch axes.
3. `motor_map`: Left/Right assignments and Polarity Inversions.
4. `hardware_limits`: The discovered `min_pwm` deadbands and `motor_trim` ratio.
5. `dynamics`: The `balance_point_offset`.

**On all future bootups:** The robot detects `pid_config.json`, bypasses the Tabula Rasa protocol entirely, performs a
Kick-Up maneuver based on established facts, and balances flawlessly.

The configuration is managed using `pydantic` models for robust validation and serialization.

> **Implementation Status:** Implemented.
> **Code Correlation:** `src/balance_bot/config.py` in `RobotConfig.save`.

---

# Appendix A: Finding the True Balance Point (Deep Dive)

## Problem Statement

The robot initially determines its target angle ("Zero") using static calibration or geometric midpoint. However, the **Dynamic Balance Point** depends on the Center of Mass (CM), which may not align with the geometric center due to battery placement, wiring, or other asymmetries.

If the target angle is incorrect, the PID controller must constantly exert force to maintain that angle, resulting in a **Constant Steady-State Error** (Integral Windup) and reduced battery life.

## Solution: Effort Minimization

The robot autonomously finds its true balance point by minimizing the **Control Effort** required to stay stationary. This logic is implemented in `src/balance_bot/adaptation/tuner.py` via the `BalancePointFinder` class.

*   **Principle**: At the true balance point, the average motor output required to maintain that angle should be zero (ignoring friction).
*   **Observation**:
    *   **Forward Drift**: If the robot constantly drives **Forward** (Positive Motor Output) to stay upright, it is leaning **Forward** relative to its true balance point.
    *   **Backward Drift**: If the robot constantly drives **Backward** (Negative Motor Output), it is leaning **Backward**.

### The Algorithm: "Drift and Correct"

The `BalancePointFinder` runs in the background and slowly adjusts the `target_angle` based on the average motor output.

#### 1. Sampling Phase
The robot collects motor output samples only when it is in a **Stable State**:
*   **Condition**: The robot is balancing (not crashed).
*   **Condition**: `Abs(Pitch_Rate) < Threshold` (Not wobbling).
*   **Condition**: User is not requesting movement (Velocity target is 0).

#### 2. Analysis & Update
Once enough samples are collected (defined by `balance_check_interval`), the average motor output is calculated.

*   If **Average Motor Output > Threshold** (Positive/Forward):
    *   **Action**: *Decrease* Target Angle (Lean Back).
    *   *Effect*: Shifts the CM backward over the wheels.
*   If **Average Motor Output < -Threshold** (Negative/Backward):
    *   **Action**: *Increase* Target Angle (Lean Forward).
    *   *Effect*: Shifts the CM forward over the wheels.

**Formula**:
`New_Target = Current_Target +/- Learning_Rate`

#### 3. Safety Mechanisms
*   **Max Deviation Clamp**: The target angle is clamped to a safe range (e.g., +/- 10 degrees) to prevent dangerous tilts.
*   **Slow Learning**: The learning rate is small to ensure stability and avoid oscillation.
*   **Persistence**: The calibrated `target_angle` is saved to `pid_config.json` via the `Agent`'s configuration saving mechanism, allowing the robot to "learn" its balance point over multiple sessions.

---

# Appendix B: Future Architecture (The "Proprioceptive Toddler" Protocol)

This section outlines a design philosophy for a more "agentic" version of the wiring check, treating the robot as an autonomous agent building a **Knowledge Graph**.

## 1. Philosophy: "Motor Babbling" & Graph Traversal

A toddler does not follow a checklist to learn to walk. They lie on the floor, twitch a muscle, feel a sensation, and associate the two. They build a mental model of their body through **Motor Babbling** (random/semi-random actuation to observe sensor response).

The proposed architecture is a **Dependency-Driven Discovery Engine**.

* **Knowledge Atoms:** Discrete units of fact (e.g., `IMU_BUS_ID`, `GRAVITY_VECTOR`, `MOTOR_DEADBAND`).
* **Experiments:** Small, atomic actions that require specific *Input Atoms* and produce *Output Atoms*.
* **The Loop:** The robot continuously looks at what it knows, finds an experiment whose prerequisites are met, runs it, and adds the result to its knowledge base.

## 2. The Knowledge Ontology (The "Atoms")

These are the specific facts the robot can "know." The state is considered "Complete" when all Critical Atoms are populated.

| Knowledge Atom | Type | Description | Dependency |
| --- | --- | --- | --- |
| `HardwareBus` | `int` | The I2C bus ID for motors/IMU. | *None* |
| `GravityVector` | `Vector3` | Which raw axis points "Down"? | `HardwareBus` |
| `StaticStability` | `bool` | Is the robot sitting still? | `HardwareBus` |
| `MotorPresence` | `bool` | Do I have motors connected? | `HardwareBus` |
| `FrictionThreshold` | `float` | PWM required to cause *any* vibration (Stiction). | `MotorPresence` |
| `PitchAxis` | `Axis` | Which Gyro axis changes when I lurch? | `GravityVector`, `FrictionThreshold` |
| `MotorPhasing` | `bool` | Do motors 0 and 1 spin together or fight? | `PitchAxis` |
| `MotorPolarity` | `int` | Does +PWM make me pitch *up* (stand) or *down* (faceplant)? | `PitchAxis`, `MotorPhasing` |
| `ChassisHandedness` | `Map` | Which motor channel is Left vs Right? | `MotorPolarity`, `GravityVector` |
| `TrimCalibration` | `float` | Factor to make straight drive actually straight. | `ChassisHandedness` |

## 3. Handling "Trauma" (Failure Modes)

In a linear script, a failure is an exit. In a Discovery engine, a failure is **Data**.

* **Ambiguity:** If "The Crunch" results in equal vibration on X and Y axes, the robot doesn't crash. It retries with higher power.
* **Contradiction:** If "The Pirouette" says Motor 0 is Left, but a later validation check implies Motor 0 is Right, the robot invalidates the `ChassisHandedness` node and re-runs the experiment.
* **The "Fall" Event:** If at any point the robot tips past `CRASH_ANGLE` (e.g., during "The Attempt"), the logic pauses. It waits for the human to reset it (Proprioception: "I am crashed"). Once upright, it resumes exactly where it left off, effectively "remembering" that the previous attempt knocked it over.

---

# Appendix C: Architectural Simplifications

To ensure code quality and reduce boilerplate, the project utilizes:

1.  **Pydantic:** Used for `RobotConfig` and all sub-configs (`PIDParams`, `BatteryConfig`, etc.). This replaces manual JSON parsing and validation with robust, typed models.
2.  **Simple-PID:** The `pid.py` module wraps the industry-standard `simple-pid` library. It adds domain-specific logic like Gyro-based Derivative terms and Integral Clamping, while delegating the core math to the library.

# Appendix D: Detailed Discovery Protocol (Legacy Reference)

*Note: The implementation has been consolidated into simpler, more robust steps (e.g., `DeriveKinematicsStep` now handles Phasing, Orientation, and L/R Identity in one pass).*

## The Order of Discovery (The Knowledge Graph)

The sequence is strictly ordered. The robot cannot deduce $N$ without first proving $N-1$.

1. **I2C Bus Assignments:** Where are my brain stem and inner ear?
2. **Hardware Initialization:** Connect the nerves.
3. **Motor Candidates:** Assume I have two legs (Ch0, Ch1).
4. **Friction Threshold:** How hard do I have to push to move my body?
5. **Motor Phasing:** Do my legs move together, or am I spinning in a circle?
6. **Spatial Orientation (The Blind Flop):** Which way is up, forward, and tilt?
7. **Motor Direction:** Which way is "Forward"?
8. **Left/Right Identity:** Which leg is left and which is right? (Also fixes Gyro Yaw).
9. **Motor Trim:** One of my legs is stronger. Let's fix that so I drive straight.
10. **Mechanical Backlash:** How loose are my joints (gear slop)?
11. **Kick-Up Dynamics:** How much momentum do I need to stand myself up?

## Step-by-Step Execution Playbook

### Step 1: Discover Buses
* **What it does:** Scans I2C buses (1, 3, 0, 2) for standard addresses `0x22` (PiconZero) and `0x68` (MPU6050).
* **Traps:** If I2C wires are completely disconnected, it will die here.

### Step 4: Friction Threshold
* **What it does:** Pulses the motors with incrementally higher power until it detects physical movement.
* **How it checks:** Calculates the raw gyroscope magnitude.
* **Traps/Retries:** If the bot is held in the air, it will think friction is very low. It expects to be on the floor.

### Step 5-8: Derive Kinematics (Consolidated)
* **What it does:**
    1.  **Phasing:** Tests if motors are wired in phase.
    2.  **Orientation:** Measures Gravity and Gyro response to determine Up, Pitch, and Forward axes.
    3.  **L/R Identity:** Uses the Right-Hand Rule (driving one motor forward, one backward) to physically identify Left vs Right channels.
* **Traps:** Requires the robot to be on the floor and able to move slightly.

### Step 9: Motor Trim
* **What it does:** Drives straight for 1 second. Measures Yaw drift and calculates a correction factor.

### Step 10: Mechanical Backlash
* **What it does:** Engages gears forward, then slams them in reverse and times the delay until the IMU registers a chassis pitch rate spike.

### Step 11: Kick-Up Dynamics
* **What it does:** Tests different PWM values to find the exact momentum required to kick the robot up from the ground to a standing position from both the BACK and the FRONT.
