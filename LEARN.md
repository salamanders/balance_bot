Here is the complete architectural specification for the new `wiring_check.py`.

---

# Technical Specification: Zero-Knowledge "Self-Discovery" Wiring Check

## 1. Overview & Philosophy

**Current State:** The robot runs a rigid, linear script (Step 1 → Step 2 → Step 3) that requires constant human
verification ("Did I move forward?").
**New State:** The robot acts like a state machine. It wakes up knowing nothing. It loops, asking: *"What is the most
basic thing I don't know yet?"* It runs a specific experiment to learn that fact, saves it to the config, and repeats
the loop.

**Goal:** Reduce human interaction from **~10 prompts** to **1 prompt**.

---

## 2. The Core Loop (Main Entry Point)

The `run()` method should no longer be a list of function calls. It is now a **Knowledge Dependency Loop**.

Every one of these steps MUST be allowed to fail.  If you can't find the right bus?  Fail.  If once you find the right bus you can't read a gyro reading?  Fail.    If you can't pusle a motor for 0.1 second with "1" minium power?  Fail.
If when you move nothing in the gyros change?  Fail.
e.g. don't assume something worked.  Each step should be "attempt to discover the thing.  Discover the thing.  VERIFY THE THING.  Save new discovery.  Next iteration".


### Pseudocode

```python
def run(self):
    print("Beginning Self-Discovery Protocol...")

    while True:
        # Reload config from disk (or keep in memory) to see what we know
        c = self.config

        # --- Tier 1: Hardware Connectivity ---
        if c.motor_i2c_bus is None or c.imu_i2c_bus is None:
            self.discover_buses()
            self.config.save()
            continue

        # --- Tier 2: The Physical World (Sensors) ---
        # We need to know Up, Front, and the Pivot Axis before we can move.
        if c.accel_vertical_axis is None:
            self.calibrate_static_orientation()
            self.config.save()
            continue

        # --- Tier 3: Action/Reaction (Motors) ---
        # 3a. Friction Threshold
        if c.min_power_visible == 0:
            self.find_min_power()
            self.config.save()
            continue

        # 3b. Phasing (Are motors spinning together or fighting?)
        # We use a specific flag in config, e.g., 'motor_phasing_verified'
        if not c.motor_phasing_verified:
            self.align_motors_phase()
            self.config.save()
            continue

        # 3c. Direction (Does +Power make me Stand Up or Dig In?)
        # We use a flag, e.g., 'motor_direction_verified'
        if not c.motor_direction_verified:
            self.determine_motor_direction()  # The "Kick Up" Test
            self.config.save()
            continue

        # --- Tier 4: The Human Anchor ---
        # The ONLY thing we can't solve with physics is "Left vs Right"
        if not c.motor_channels_verified:
            self.ask_human_left_right()
            self.config.save()
            continue

        # --- Tier 5: Dynamics (Optional/Advanced) ---
        if c.control.kickup_power_forward == 0:
            self.find_flop_thresholds()
            self.config.save()
            continue

        print(" [SUCCESS] Self-Discovery Complete. I know everything.")
        break

```

---

## 3. The Solvers (Detailed Logic)

### Tier 1: `discover_buses()`

* **Goal:** Find I2C addresses `0x22` (Motors) and `0x68` (IMU).
* **Method:** Scan buses `[0, 1, 2, 3]`.
* **Human Input:** None.
* **Output:** Set `motor_i2c_bus` and `imu_i2c_bus`.

### Tier 2: `calibrate_static_orientation()` (Physics Deduction)

* **Goal:** Map Physical Axes (X, Y, Z) to Logical Axes (Vertical, Forward, Pitch).
* **Pre-condition:** Robot is sitting on the floor, motors off, resting on *either* front or back training wheels.
* **Logic:**

1. **Read Gravity:** The axis with the largest absolute value (~1.0g) is **Vertical**.
2. **Read Lean:** The axis with the *next* largest value is **Forward** (because the robot is leaning, gravity "leaks"
   into this axis).
3. **Read Pivot:** The axis with the *smallest* value (~0.0g) is **Pitch**. (Gravity does not pull "sideways" along the
   axle).


* **Action:**
* Set `accel_vertical_axis` = Max Axis. Set `accel_vertical_invert` based on sign.
* Set `accel_forward_axis` = Mid Axis.
* Set `gyro_pitch_axis` = Min Axis.


* **Human Input:** None.

### Tier 3a: `find_min_power()`

* **Goal:** Find the minimum PWM needed to overcome friction.
* **Method:**

1. Start PWM at 10.
2. Pulse both motors: `set_motors(PWM, PWM)`.
3. Check IMU: Did Gyro or Accel standard deviation spike above noise floor?
4. If no, PWM += 5. Repeat.


* **Human Input:** None.
* **Output:** Set `min_power_visible`.

### Tier 3b: `align_motors_phase()` (The Logic Puzzle)

* **Goal:** Ensure motors spin *together* (Straight), not *opposite* (Spin), regardless of direction.
* **Logic:**
* If motors are wired identically: `set_motors(50, 50)` -> Robot drives Straight (High Accel, Low Yaw).
* If motors are wired oppositely: `set_motors(50, 50)` -> Robot Spins (Low Accel, High Yaw).


* **Method:**

1. Drive `set_motors(min_power+10, min_power+10)`.
2. Measure `yaw_rate` vs `forward_accel_delta`.
3. **If Yaw is dominant:** Invert logic of **Channel 1 only**. Retest.
4. Now Accel should be dominant.


* **Human Input:** None.
* **Output:** Set internal flag `motor_phasing_verified = True`. (We don't know if we are going Forward or Backward yet,
  just that we are going *Straight*).

### Tier 3c: `determine_motor_direction()` (The "Kick Up" Test)

* **Goal:** Ensure Positive Power = "Stand Up" (Reduce Lean).
* **Method:**

1. Measure `start_pitch_angle`. (e.g., 30 degrees lean).
2. Pulse motors Positive: `set_motors(+Power, +Power)`.
3. Measure `end_pitch_angle`.
4. **Physics Check:**

* If `abs(end_pitch)` < `abs(start_pitch)`: The robot moved toward vertical (Stood Up). **Good.**
* If `abs(end_pitch)` > `abs(start_pitch)`: The robot dug into the floor (Increased Lean). **Bad.**


5. **Action:**

* If Bad: Invert **BOTH** Motor L and Motor R.


6. **Gyro Check:**

* Did the Gyro sign match the Angle change? If not, invert `gyro_pitch_invert`.


* **Human Input:** None.
* **Output:** Set `motor_l/r_invert`, `gyro_pitch_invert`, `accel_forward_invert` (deduced from motion).

### Tier 4: `ask_human_left_right()` (The Anchor)

* **Goal:** Map Channel 0/1 to Left/Right.
* **Context:** We know how to drive Straight, and we know which way is Up. We just don't know L/R.
* **Method:**

1. Robot says: *"I am going to spin. Watch me."*
2. Robot drives **Channel 0 Forward** and **Channel 1 Backward**.
3. Prompt: *"Did I spin CLOCKWISE (Right) or COUNTER-CLOCKWISE (Left)?"*
4. **Logic:**

* User says "Clockwise": Ch0 is **Left**, Ch1 is **Right**.
* User says "Counter-Clockwise": Ch0 is **Right**, Ch1 is **Left**.


* **Human Input:** Yes (One keystroke).
* **Output:** Set `motor_l` and `motor_r` indices.

---

## 4. Implementation Notes for Developer

1. **Safety First:** In every function that moves motors, wrap the logic in a `try...finally` block that ensures
   `self.hw.stop()` is called if the script crashes or is interrupted.
2. **Config Flags:** You may need to add temporary boolean flags to `RobotConfig` (e.g., `motor_phasing_verified`) to
   track progress through the loop, or infer them from existing data (e.g., if `min_power_visible > 0`, that step is
   done).
3. **Thresholds:**

* Gravity detection: `> 0.8g`.
* Motion detection: `Gyro Rate > 5 deg/s` or `Accel StdDev > 0.05`.

Human interaction should never be a "race". Don't make me do things within X seconds.  It would be much better to stop and ask "which way did it rotate?"
This (of course) would be after you verified that yes, the gyros said it DID ROTATE so you don't need to ask me "did it rotate at all, and if so which way".  REmember: minimizing the load on the person.

4. **Clean Up:** Remove the old numbered step functions (`step_1_bus`, `step_2_motors`) entirely. Replace with semantic
   names (`discover_buses`, `calibrate_orientation`).