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
rapidly in a success pattern and writes a `config.json` file to disk containing:

1. Validated `i2c_bus` IDs and `sensor_addresses`.
2. `axis_map`: The vector matrices defining the Up/Down and Pitch axes.
3. `motor_map`: Left/Right assignments and Polarity Inversions.
4. `hardware_limits`: The discovered `min_pwm` deadbands and `motor_trim` ratio.
5. `dynamics`: The `balance_point_offset`.

**On all future bootups:** The robot detects `config.json`, bypasses the Tabula Rasa protocol entirely, performs a
Kick-Up maneuver based on established facts, and balances flawlessly.

> **Implementation Status:** Implemented.
> **Code Correlation:** `src/balance_bot/config.py` in `RobotConfig.save`.



## Appendix: Finding the True Balance Point (Deep Dive)

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
