This design document reimagines the `check_wiring.py` script. Instead of a linear script ("Tier 1" then "Tier 2"), we will treat the robot as an autonomous agent building a **Knowledge Graph**.

The system loop changes from `Step 1 -> Step 2 -> Step 3` to `Evaluate Knowns -> Select Possible Experiment -> Execute -> Update Beliefs`.

---

# Design Doc: The "Proprioceptive Toddler" Protocol

**Project:** Balance Bot (Salamanders)

**Target File:** `src/balance_bot/discovery/brain.py` (New)

**Replaces:** `src/balance_bot/wiring_check.py`

## 1. Philosophy: "Motor Babbling" & Graph Traversal

A toddler does not follow a checklist to learn to walk. They lie on the floor, twitch a muscle, feel a sensation, and associate the two. They build a mental model of their body through **Motor Babbling** (random/semi-random actuation to observe sensor response).

The new architecture is a **Dependency-Driven Discovery Engine**.

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

## 3. The Experiment Registry

The "Brain" iterates through this registry. If an experiment's **Prerequisites** are met but its **Result** is unknown, it becomes a candidate for execution.

### A. The Pulse (Hardware Scan)

* **Prereq:** None.
* **Action:** Scan I2C buses for `0x68` (IMU) and `0x22` (Motors).
* **Learns:** `HardwareBus`.
* **Failure:** Panic (No brain/body).

### B. The Meditation (Gravity Discovery)

* **Prereq:** `HardwareBus`.
* **Action:** Read IMU for 1 second while requesting `0` motor output.
* **Observation:** The dominant accelerometer vector is Gravity.
* **Learns:** `GravityVector` (Down), `StaticStability`.

### C. The Twitch (Stiction & Presence)

* **Prereq:** `HardwareBus`.
* **Action:** "Motor Babbling." Send short 50ms pulses of increasing PWM (10% -> 100%) to random channels.
* **Observation:** Monitor Gyro/Accel magnitude (total vibration).
* **Learns:** `FrictionThreshold` (The PWM where the robot starts shaking), `MotorPresence`.

### D. The Crunch (Pitch Axis Identification)

* **Prereq:** `GravityVector`, `FrictionThreshold`.
* **Action:** Send a "Lurch" command (surge both motors +30% for 200ms).
* **Observation:** Which Gyro axis had the highest rotational velocity?
* **Learns:** `PitchAxis` (The axis of rotation), `PitchPolarity` (tentative).
* **Note:** This works even if motors are fighting (spinning); the chassis will still tilt slightly or vibrate along the pitch axis.

### E. The Wiggle (Phasing)

* **Prereq:** `PitchAxis`.
* **Action:** Drive Motor A `+Power`, Motor B `+Power`.
* **Observation:** Compare Gyro Yaw rate (rotation around Gravity) vs Pitch rate.
* High Yaw, Low Pitch = Motors are fighting (Spinning).
* Low Yaw, High Pitch = Motors are helping (Driving).


* **Learns:** `MotorPhasing` (Invert one motor if necessary).

### F. The Attempt (Directionality)

* **Prereq:** `MotorPhasing`, `PitchAxis`.
* **Action:** Measure current Tilt (Angle of Gravity relative to Pitch Axis). Apply `+Power` (assumed Forward).
* **Observation:** Did the Tilt angle *decrease* (move toward vertical) or *increase* (fall over)?
* **Toddler Logic:** "I want to get up. I tried pushing my legs *this* way. Did my head go up?"


* **Learns:** `MotorPolarity` (Invert global direction if it made the fall worse).

### G. The Pirouette (Left/Right ID)

* **Prereq:** `MotorPolarity`.
* **Action:** Drive Motor 0 Forward, Motor 1 Backward.
* **Observation:** Use the Right-Hand Rule on the Gyro Vector.
* Did I spin CW or CCW relative to Gravity?


* **Learns:** `ChassisHandedness` (If I spun Left, Motor 0 is Right).

## 4. The Runtime Architecture

Instead of a `while True` loop with `if/else`, we implement a **Solver**.

```python
class DiscoveryBrain:
    def __init__(self):
        self.knowledge = {} # The Graph
        self.experiments = [ExpPulse, ExpMeditation, ExpTwitch, ...]

    def think(self):
        while not self.is_fully_calibrated():
            # 1. State Estimation
            current_state = self.knowledge.keys()

            # 2. Candidate Selection
            candidates = []
            for exp in self.experiments:
                if exp.can_run(current_state) and not exp.has_result(self.knowledge):
                    candidates.append(exp)

            if not candidates:
                raise StuckException("I don't know enough to learn anything new.")

            # 3. Prioritization (Cheapest/Safest first)
            best_exp = self.prioritize(candidates)

            # 4. Execution
            print(f"I am curious about... {best_exp.name}")
            result = best_exp.run()

            # 5. Integration
            if result.success:
                self.knowledge.update(result.data)
                self.save_checkpoint()
            else:
                self.handle_trauma(result) # Back off, retry, or fail

```

## 5. Handling "Trauma" (Failure Modes)

In a linear script, a failure is an exit. In a Discovery engine, a failure is **Data**.

* **Ambiguity:** If "The Crunch" results in equal vibration on X and Y axes, the robot doesn't crash. It retries with higher power.
* **Contradiction:** If "The Pirouette" says Motor 0 is Left, but a later validation check implies Motor 0 is Right, the robot invalidates the `ChassisHandedness` node and re-runs the experiment.
* **The "Fall" Event:** If at any point the robot tips past `CRASH_ANGLE` (e.g., during "The Attempt"), the logic pauses. It waits for the human to reset it (Proprioception: "I am crashed"). Once upright, it resumes exactly where it left off, effectively "remembering" that the previous attempt knocked it over.

## 6. Implementation Steps

1. **Refactor `RobotHardware`:** Ensure it can be initialized "Partially". It should accept `None` for config values it doesn't know yet (e.g., `accel_axis=None`) and return raw dicts instead of processed objects in those cases.
2. **Create `DiscoveryContext`:** A class that persists the `knowledge` dictionary to disk (`discovery_state.json`) instantly after every learned atom. This allows the robot to be rebooted mid-process and resume.
3. **Implement Experiments:** Port the logic from `wiring_check.py` into discrete `Experiment` classes.
4. **The "Jules" Persona:** Add log messages that reflect this exploratory nature.
* *Old:* `[INFO] Checking Phasing...`
* *New:* `[THOUGHT] I felt a vibration, but I didn't move forward. My legs might be fighting each other. Attempting to synchronize...`



## 7. Success State

The process ends when the **Knowledge Graph** satisfies the requirements to build a full `RobotConfig` object. At that point, the `DiscoveryBrain` saves `config.json` and hands control over to the `BalanceCore` (The "Adult" brain).
