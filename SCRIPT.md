# SCRIPT.md: Balance Bot First-Run Bootstrap Protocol ("The Spider-Verse Run")

*"Alright, people, let's do this one last time!"* This document details the exact, step-by-step execution of the `WiringCheck` state machine from `wiring_check.py`. It maps out how the robot wakes up with zero knowledge of its own body and builds a complete physical understanding of itself.

---

## 1. The Order of Discovery (The Knowledge Graph)

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

---

## 2. Step-by-Step Execution Playbook

### Step 1: `discover_buses()`
* **What it does:** Scans I2C buses (1, 3, 0, 2) for standard addresses `0x22` (PiconZero) and `0x68` (MPU6050).
* **Likely Values:** `motor_i2c_bus = 1`, `imu_i2c_bus = 1`.
* **Traps:** If I2C wires are completely disconnected, it will die here via `scan_i2c_or_die()`.

### Step 2 & 3: `init_hw()` & `_set_default_motors()`
* **What it does:** Boots the `RobotHardware` abstraction layer. Assumes Motor L is Ch0 and Motor R is Ch1 for now. 

### Step 4: `find_min_power()`
* **What it does:** Pulses the motors with incrementally higher power (starting at 10, stepping by 5) until it detects physical movement.
* **How it checks:** Calculates the raw gyroscope magnitude `sqrt(x^2 + y^2 + z^2) > 15.0 deg/s`.
* **Likely Values:** Depending on the carpet/floor, usually around `PWM = 20` to `35`.
* **Traps/Retries:** If the bot is held in the air, it will think friction is very low (`PWM = 10`). It expects to be on the floor. 

### Step 5: `align_motors_phase()`
* **What it does:** Tests if the motors are wired in the same phase (driving) or opposite phase (spinning in place). Drives at `min_power + 10`.
* **How it checks:** Takes the dot product of the Gravity vector (accelerometer) and the Rotation vector (gyroscope). 
  * If spinning: Rotation aligns with gravity (vertical axis). `cos_theta > 0.6`.
  * If driving/pitching: Rotation is perpendicular to gravity. 
* **Action:** If it detects a spin, it silently inverts `motor_r_invert`.
* **Retries:** Uses `verify_with_retries`. If it didn't move enough to tell, it tries again.

### Step 6: `calibrate_static_orientation()`
* **What it does:** The "Blind Flop". It measures gravity (P1), then pulses motors hard (`min_power + 30`) to flop over, and measures gravity again (P2). 
* **How it deduces axes:** * Pitch Axis = `cross_product(P1, P2)`
  * Vertical Axis = The dominant gravity component.
  * Forward Axis = The remaining axis.
* **Likely Values:** `gyro_pitch_axis = Y`, `accel_vertical_axis = Z`, `accel_forward_axis = X`. 
* **Traps/Retries:** If P1 to P2 angle change is `< 45 degrees`, it assumes it pushed into the floor instead of flopping over. It reverses power and tries again to hit P3. Trap: If it's shoved against a wall, the flop fails.

### Step 7: `determine_motor_direction()`
* **What it does:** Ensures positive motor power makes the robot "stand up" (reduce its pitch angle). 
* **How it checks:** Forces the bot to the FRONT bumper (pitch > 10). Pulses positive power. If pitch drops (moves towards 0) or forward acceleration spikes, direction is correct.
* **Action:** If it leans *further* forward, it inverts both `motor_l_invert` and `motor_r_invert`.
* **Traps:** Needs to successfully flop to the FRONT. If it gets stuck balancing perfectly upright (rare), it might misread the start state.

### Step 8: `deduce_left_right_autonomous()`
* **What it does:** "The Arc Maneuver". Drives Ch0 at `High Power` and Ch1 at `Low Power`. Uses the Right-Hand Rule of physics.
* **How it checks:** Takes dot product of `Up_Vector` (-Gravity) and `Avg_Gyro`. 
  * If $> 0$: Counter-Clockwise turn. Meaning the fast motor (Ch0) is on the RIGHT side.
  * If $< 0$: Clockwise turn. Fast motor (Ch0) is on the LEFT side.
* **Action:** Corrects `motor_l` and `motor_r` mappings. Also calibrates `gyro_yaw_invert` based on the turn direction.
* **Traps:** Absolutely brilliant deduction, but requires a slip-free floor. If wheels slip, the arc won't generate enough yaw. If spin rate is `< 10.0`, it retries.

### Step 9: `calibrate_motor_trim()`
* **What it does:** Drives straight for 1 second. Measures `avg_yaw_rate`. 
* **How it corrects:** Applies a proportional correction to `learning_state.motor_trim`. 
* **Likely Values:** `Trim = 0.05` to `0.15` (5-15% variance between cheap DC motors is common).
* **Retries/Traps:** Loops up to 15 times until drift is `< 2.0 deg/s`. If it can't perfect it, it saves the "best effort" to avoid an endless loop.

### Step 10: `measure_backlash()`
* **What it does:** Engages gears forward, then slams them in reverse and times the delay until the IMU registers a chassis pitch rate spike.
* **Likely Values:** `backlash_pulse_time = 0.03` to `0.08` seconds.
* **Traps:** Safety timeout of 1.0s. If it never detects movement, it assumes max slop (0.2s) and moves on.

### Step 11: `find_flop_thresholds()`
* **What it does:** Dynamic "Roll & Slam". Tests different PWM values to find the exact momentum required to kick the robot up from the ground to a standing position from both the BACK and the FRONT.
* **Traps/Cycles:** Because kicking up rapidly can cause the robot to drift sideways if trim isn't perfect, it monitors alignment. If it detects drift (`avg_yaw > 15.0`), it pauses, calls `calibrate_motor_trim()` again, and resumes!

### Final Verification: `verify_final_configuration()`
* **What it does:** A pessimistic double-check. Drives straight (expects no spin). Turns right (expects negative yaw). 
* **Traps:** If this fails, it throws a `sys.exit(1)` fatal error. The configuration is irreparably confused.

---

## 3. Architecture Critique: "Sequential Logic" vs "Toddler Flopping"

**Question:** *Should it do things differently? E.g., flop around like a toddler more randomly to figure out how limbs work?*

**Answer:** No. This order of operations is practically optimal for a zero-knowledge constrained system. 

If the robot were a neural network in a simulation (like OpenAI's walking agents), random toddler flopping works because millions of iterations allow a gradient descent algorithm to eventually map inputs to IMU vectors. 

However, in the real physical world:
1. **Battery Life & Wear:** Random flopping burns battery and strips plastic gears.
2. **The "Chicken and Egg" Data Problem:** You cannot make sense of random IMU noise if you don't know which axis is up. You cannot find which axis is up without predictably rolling the chassis. You cannot predictably roll the chassis if your motors are fighting each other (Phase mismatch). 

The `WiringCheck` script solves this by isolating variables perfectly:
* **Step 5 (Phase)** isolates wheel rotation from chassis rotation.
* **Step 6 (Axes)** relies on Step 5's phase-lock to guarantee a pure pitch-axis rotation.
* **Step 8 (Left/Right)** relies on Step 7's direction-lock so the robot actually drives forward in an arc, rather than spinning wildly backward.

### Potential Real-World Gaps Identified:
* **The "Against the Wall" Trap:** In Steps 6, 7, and 11, the robot relies on being able to flop. If a user places it against a wall or table leg, the `wait_for_stability` will pass, but the physical maneuvers will fail to achieve the required angles. The robot will assume it needs more power, ramp up to `PWM=100`, and still fail. 
* *Mitigation:* The `SurvivalWatchdog` catches infinite loops, but a human *must* provide clearance.
* **The "Carpet Grab" Trap:** In Step 8 (The Arc), high friction carpet might cause the robot to pitch forward and faceplant instead of executing a smooth yawing arc. 
* *Mitigation:* The arc uses `setup_p` and `kick_p` logic later on, but for the arc itself, it just powers the wheels. If it faceplants, the dot-product yaw threshold (`abs(dot_prod) < 10.0`) will fail safely and force a retry, preventing a false Left/Right mapping.

**Conclusion:** The script is incredibly resilient. It avoids the endless cycles by using strict thresholds, maximum attempt counters (e.g., `15` for trim, `5` for posture forcing), and a watchdog timer. Let it run its sequence.
