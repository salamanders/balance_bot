import re
import os

def update_types():
    with open('src/balance_bot/hardware/types.py', 'r') as f:
        content = f.read()
    new_imureading = """@dataclass(slots=True)
class IMUReading:
    \"\"\"
    Data structure for converted IMU readings.
    \"\"\"
    pitch_angle: float
    pitch_rate: float
    yaw_rate: float
    roll_angle: float
    roll_rate: float
    error_count: int = 0
    accel_raw: glm.vec3 | None = None
    gyro_raw: glm.vec3 | None = None
"""
    content = re.sub(r'class IMUReading:.*?def __eq__[^:]*:\n.*?gyro_raw == other.gyro_raw\)', new_imureading, content, flags=re.DOTALL)
    with open('src/balance_bot/hardware/types.py', 'w') as f:
        f.write(content)

def update_balance_core():
    with open('src/balance_bot/reflex/balance_core.py', 'r') as f:
        content = f.read()

    content = re.sub(r'class MotionRequest:\n    """\n    Tier 3 -> Tier 1 Command Interface.\n    """\n    __slots__ = \[\'velocity\', \'turn_rate\', \'enable_control\'\]\n\n    def __init__\(self, velocity: float = 0\.0, turn_rate: float = 0\.0, enable_control: bool = True\):\n        self\.velocity = velocity\n        self\.turn_rate = turn_rate\n        self\.enable_control = enable_control\n',
    r'''@dataclass(slots=True)
class MotionRequest:
    """
    Tier 3 -> Tier 1 Command Interface.
    """
    velocity: float = 0.0
    turn_rate: float = 0.0
    enable_control: bool = True
''', content, flags=re.DOTALL)

    content = re.sub(r'class BalanceTelemetry:\n    """\n    Tier 1 -> Tier 2/3 Data Interface.\n\n    NOTE: This is intentionally implemented as a standard class with __slots__\n    rather than a @dataclass\(frozen=True\) to avoid significant instantiation\n    overhead inside the high-frequency 100Hz reflex loop.\n    """\n    __slots__ = \[\n        \'pitch_angle\', \'pitch_rate\', \'yaw_rate\', \'error_count\',\n        \'motor_output\', \'crashed\', \'left_pwm\', \'right_pwm\', \'target_angle\'\n    \]\n\n    def __init__\(self, pitch_angle: float, pitch_rate: float, yaw_rate: float, error_count: int, motor_output: float, crashed: bool, left_pwm: float, right_pwm: float, target_angle: float\):\n        self\.pitch_angle = pitch_angle\n        self\.pitch_rate = pitch_rate\n        self\.yaw_rate = yaw_rate\n        self\.error_count = error_count\n        self\.motor_output = motor_output\n        self\.crashed = crashed\n        self\.left_pwm = left_pwm\n        self\.right_pwm = right_pwm\n        self\.target_angle = target_angle\n',
    r'''@dataclass(slots=True)
class BalanceTelemetry:
    """
    Tier 1 -> Tier 2/3 Data Interface.
    """
    pitch_angle: float
    pitch_rate: float
    yaw_rate: float
    error_count: int
    motor_output: float
    crashed: bool
    left_pwm: float
    right_pwm: float
    target_angle: float
''', content, flags=re.DOTALL)

    content = re.sub(r'class TuningParams:\n    """\n    Tier 2 -> Tier 1 Adaptation Interface.\n    Allows dynamic adjustment of PID and Balance Point.\n    """\n    __slots__ = \[\'kp\', \'ki\', \'kd\', \'target_angle_offset\'\]\n\n    def __init__\(self, kp: float, ki: float, kd: float, target_angle_offset: float\):\n        self\.kp = kp\n        self\.ki = ki\n        self\.kd = kd\n        self\.target_angle_offset = target_angle_offset\n',
    r'''@dataclass(slots=True)
class TuningParams:
    """
    Tier 2 -> Tier 1 Adaptation Interface.
    Allows dynamic adjustment of PID and Balance Point.
    """
    kp: float
    ki: float
    kd: float
    target_angle_offset: float
''', content, flags=re.DOTALL)

    new_init = """    def __init__(self, hw_config: HardwareConfig, learning_state: LearningState, watchdog: SurvivalWatchdog | None = None):
        self.hw_config = hw_config
        self.learning_state = learning_state

        self.hw = RobotHardware(self.hw_config, self.learning_state, watchdog=watchdog)
        self.hw.init()

        # Control
        self.pid = PIDController(learning_state.pid)
        self.filter = ComplementaryFilter(hw_config.complementary_alpha)

        # State
        self.pitch = 0.0

        # Pre-allocate telemetry object
        self.current_telemetry = BalanceTelemetry(
            pitch_angle=0.0,
            pitch_rate=0.0,
            yaw_rate=0.0,
            error_count=0,
            motor_output=0.0,
            crashed=False,
            left_pwm=0.0,
            right_pwm=0.0,
            target_angle=0.0
        )
"""
    content = re.sub(r'    def __init__\(self, hw_config: HardwareConfig, learning_state: LearningState, watchdog: SurvivalWatchdog \| None = None\):\n        self\.hw_config = hw_config\n        self\.learning_state = learning_state\n\n        self\.hw = RobotHardware\(self\.hw_config, self\.learning_state, watchdog=watchdog\)\n        self\.hw\.init\(\)\n\n        # Control\n        self\.pid = PIDController\(learning_state\.pid\)\n        self\.filter = ComplementaryFilter\(hw_config\.complementary_alpha\)\n\n        # State\n        self\.pitch = 0\.0\n', new_init, content, flags=re.DOTALL)

    content = re.sub(
    r'            return BalanceTelemetry\(\n                pitch_angle=self\.pitch,\n                pitch_rate=reading\.pitch_rate,\n                yaw_rate=reading\.yaw_rate,\n                error_count=reading\.error_count,\n                motor_output=0\.0,\n                crashed=False,\n                left_pwm=0\.0,\n                right_pwm=0\.0,\n                target_angle=self\.learning_state\.pid\.target_angle\n            \)',
    r'''            self.current_telemetry.pitch_angle = self.pitch
            self.current_telemetry.pitch_rate = reading.pitch_rate
            self.current_telemetry.yaw_rate = reading.yaw_rate
            self.current_telemetry.error_count = reading.error_count
            self.current_telemetry.motor_output = 0.0
            self.current_telemetry.crashed = False
            self.current_telemetry.left_pwm = 0.0
            self.current_telemetry.right_pwm = 0.0
            self.current_telemetry.target_angle = self.learning_state.pid.target_angle
            return self.current_telemetry''', content)

    content = re.sub(
    r'            return BalanceTelemetry\(\n                pitch_angle=self\.pitch,\n                pitch_rate=reading\.pitch_rate,\n                yaw_rate=reading\.yaw_rate,\n                error_count=reading\.error_count,\n                motor_output=0\.0,\n                crashed=True,\n                left_pwm=0\.0,\n                right_pwm=0\.0,\n                target_angle=target_angle\n            \)',
    r'''            self.current_telemetry.pitch_angle = self.pitch
            self.current_telemetry.pitch_rate = reading.pitch_rate
            self.current_telemetry.yaw_rate = reading.yaw_rate
            self.current_telemetry.error_count = reading.error_count
            self.current_telemetry.motor_output = 0.0
            self.current_telemetry.crashed = True
            self.current_telemetry.left_pwm = 0.0
            self.current_telemetry.right_pwm = 0.0
            self.current_telemetry.target_angle = target_angle
            return self.current_telemetry''', content)

    content = re.sub(
    r'        return BalanceTelemetry\(\n            pitch_angle=self\.pitch,\n            pitch_rate=reading\.pitch_rate,\n            yaw_rate=reading\.yaw_rate,\n            error_count=reading\.error_count,\n            motor_output=pid_output, # Raw PID output \(useful for battery estimation\)\n            crashed=False,\n            left_pwm=left_motor,\n            right_pwm=right_motor,\n            target_angle=target_angle\n        \)',
    r'''        self.current_telemetry.pitch_angle = self.pitch
        self.current_telemetry.pitch_rate = reading.pitch_rate
        self.current_telemetry.yaw_rate = reading.yaw_rate
        self.current_telemetry.error_count = reading.error_count
        self.current_telemetry.motor_output = pid_output
        self.current_telemetry.crashed = False
        self.current_telemetry.left_pwm = left_motor
        self.current_telemetry.right_pwm = right_motor
        self.current_telemetry.target_angle = target_angle
        return self.current_telemetry''', content)

    with open('src/balance_bot/reflex/balance_core.py', 'w') as f:
        f.write(content)

def update_robot_hardware():
    with open('src/balance_bot/hardware/robot_hardware.py', 'r') as f:
        content = f.read()

    if "import threading" not in content:
        content = content.replace("import logging", "import logging\nimport threading")

    new_init = """    def __init__(self, hw_config: HardwareConfig, learning_state: LearningState, watchdog: SurvivalWatchdog | None = None):
        \"\"\"
        Initialize the robot hardware abstraction.
        :param hw_config: The immutable HardwareConfig object.
        :param learning_state: The mutable LearningState object.
        :param watchdog: Optional SurvivalWatchdog for heartbeat pulses.
        \"\"\"
        self.hw_config = hw_config
        self.learning_state = learning_state
        self.watchdog = watchdog
        self._imu_consecutive_errors = 0

        # Store the "last known good" value
        self._last_accel = glm.vec3(0.0)
        self._last_gyro = glm.vec3(0.0)

        # Threading for IMU Sensor
        self._sensor_thread = None
        self._sensor_running = False
        self._sensor_lock = threading.Lock()

        self.pz: MotorDriver | None = None
        self.sensor: IMUDriver | None = None

        self.initialize_drivers()
        self.start_sensor_thread()
"""
    content = re.sub(r'    def __init__\(self, hw_config: HardwareConfig, learning_state: LearningState, watchdog: SurvivalWatchdog \| None = None\):.*?        self\.initialize_drivers\(\)\n', new_init, content, flags=re.DOTALL)

    thread_methods = """    def start_sensor_thread(self) -> None:
        if self.sensor is None:
            return

        self._sensor_running = True
        self._sensor_thread = threading.Thread(target=self._sensor_worker, daemon=True)
        self._sensor_thread.start()
        logger.info("IMU SensorThread started.")

    def _sensor_worker(self) -> None:
        while self._sensor_running:
            try:
                accel = self.sensor.get_accel_data()
                gyro = self.sensor.get_gyro_data()

                # Apply Bias Calibration
                bias_vec = glm.vec3(
                    self.learning_state.gyro_bias_x,
                    self.learning_state.gyro_bias_y,
                    self.learning_state.gyro_bias_z
                )
                gyro = gyro - bias_vec

                with self._sensor_lock:
                    self._last_accel = accel
                    self._last_gyro = gyro
                    self._imu_consecutive_errors = 0

            except OSError:
                with self._sensor_lock:
                    self._imu_consecutive_errors += 1

            # Give a very tiny sleep to avoid absolute 100% core starvation
            time.sleep(0.001)

    def stop_sensor_thread(self) -> None:
        self._sensor_running = False
        if self._sensor_thread and self._sensor_thread.is_alive():
            self._sensor_thread.join(timeout=1.0)
"""
    content = content.replace("    def apply_config(self, new_config: HardwareConfig) -> None:", thread_methods + "\n    def apply_config(self, new_config: HardwareConfig) -> None:")

    new_cleanup = """    def cleanup(self) -> None:
        \"\"\"Cleanup hardware.\"\"\"
        self.stop_sensor_thread()
        self.stop()
        if self.pz:
            self.pz.cleanup()
"""
    content = re.sub(r'    def cleanup\(self\) -> None:\n        """Cleanup hardware\."""\n        self\.stop\(\)\n        if self\.pz:\n            self\.pz\.cleanup\(\)\n', new_cleanup, content)


    new_read_imu_raw = """    def read_imu_raw(self) -> tuple[glm.vec3, glm.vec3]:
        \"\"\"
        Returns raw accelerometer and gyro data instantly from the background thread buffer.
        :return: Tuple of (accel_dict, gyro_dict).
        \"\"\"
        if self.sensor is None:
            raise RuntimeError("IMU Sensor not initialized (Bus Unknown?)")

        with self._sensor_lock:
            accel = self._last_accel
            gyro = self._last_gyro
            errors = self._imu_consecutive_errors

        if errors > self.hw_config.imu_max_retries:
            logger.error(f"IMU Failed {errors} times in a row. Returning cached data indefinitely to avoid fatal crash.")
        elif errors > 0:
            logger.debug(f"IMU Glitch ({errors}/{self.hw_config.imu_max_retries}). Using cached data.")

        return accel, gyro"""
    content = re.sub(r'    def read_imu_raw\(self\) -> tuple\[glm\.vec3, glm\.vec3\]:.*?            return self\._last_accel, self\._last_gyro', new_read_imu_raw, content, flags=re.DOTALL)

    with open('src/balance_bot/hardware/robot_hardware.py', 'w') as f:
        f.write(content)


def update_enums():
    with open('src/balance_bot/enums.py', 'r') as f:
        enum_content = f.read()

    enum_content = re.sub(r'class BotState\(Enum\):\n    IDLE = auto\(\)\n    KICKUP = auto\(\)\n    BALANCING = auto\(\)\n    CRASHED = auto\(\)\n    FATAL_ERROR = auto\(\)\n', '', enum_content)

    with open('src/balance_bot/enums.py', 'w') as f:
        f.write(enum_content)

def create_states():
    with open('src/balance_bot/behavior/states.py', 'w') as f:
        f.write("""from typing import Any
from dataclasses import dataclass
import time
import logging

from ..configuration import HardwareConfig, LearningState
from ..reflex.balance_core import BalanceCore, MotionRequest, TuningParams, BalanceTelemetry
from ..adaptation.battery import BatteryEstimator
from ..behavior.leds import LedController
from ..adaptation.recovery import RecoveryManager
from ..adaptation.tuner import PIDTuner
from ..enums import Direction, Orientation
from ..utils import RateLimiter
from ..watchdog import SurvivalWatchdog

logger = logging.getLogger(__name__)

@dataclass
class AgentContext:
    core: BalanceCore
    config: HardwareConfig
    learning_state: LearningState
    led: LedController
    battery: BatteryEstimator
    recovery: RecoveryManager
    tuner: PIDTuner
    watchdog: SurvivalWatchdog | None

class BotState:
    def enter(self, context: AgentContext) -> None:
        pass

    def update(self, context: AgentContext, dt: float, motion_req: MotionRequest, tuning_params: TuningParams, last_telemetry: BalanceTelemetry | None, ticks: int) -> 'BotState':
        return self

    def exit(self, context: AgentContext) -> None:
        pass

class IdleState(BotState):
    def __init__(self, kickup_attempts: int = 0):
        self.kickup_attempts = kickup_attempts

    def update(self, context: AgentContext, dt: float, motion_req: MotionRequest, tuning_params: TuningParams, last_telemetry: BalanceTelemetry | None, ticks: int) -> BotState:
        pitch = context.core.pitch

        if abs(pitch) < 10.0:
            logger.info(f"-> Detected Upright ({pitch:.1f}). Transition to BALANCING.")
            return BalancingState()
        elif abs(pitch) > 10.0:
            if self.kickup_attempts < 3:
                logger.info(f"-> Resting ({pitch:.1f}). Transition to KICKUP (Attempt {self.kickup_attempts + 1}/3).")
                return KickupState(attempts=self.kickup_attempts)
            else:
                if ticks % 500 == 0:
                    logger.warning("-> Max Kick-Up attempts reached. Waiting for manual reset.")
        return self

class KickupState(BotState):
    def __init__(self, attempts: int = 0):
        self.attempts = attempts
        self._zero_motion_enabled = MotionRequest(velocity=0.0, turn_rate=0.0, enable_control=False)
        self._zero_tuning = TuningParams(kp=0.0, ki=0.0, kd=0.0, target_angle_offset=0.0)

    def _wait_for_settle(self, context: AgentContext, duration: float = 1.0, rate_threshold: float = 10.0) -> None:
        end_time = time.perf_counter() + duration
        rate = RateLimiter(1.0 / context.config.loop_time)
        dt = context.config.loop_time
        while True:
            if context.watchdog:
                context.watchdog.heartbeat()
            telemetry = context.core.update(self._zero_motion_enabled, self._zero_tuning, dt)

            if time.perf_counter() > end_time:
                if abs(telemetry.pitch_rate) < rate_threshold:
                    break
                else:
                    end_time = time.perf_counter() + 0.5
            dt = rate.sleep()

    def _sleep_with_update(self, context: AgentContext, duration: float) -> None:
        end_time = time.perf_counter() + duration
        rate = RateLimiter(1.0 / context.config.loop_time)
        dt = context.config.loop_time
        while time.perf_counter() < end_time:
            if context.watchdog:
                context.watchdog.heartbeat()
            context.core.update(self._zero_motion_enabled, self._zero_tuning, dt)
            dt = rate.sleep()

    def _check_and_fix_position(self, context: AgentContext, kick_direction: Direction, start_label: str) -> bool:
        wrong_position = False
        if kick_direction == Direction.BACKWARD and context.core.pitch > -10:
            wrong_position = True
        elif kick_direction == Direction.FORWARD and context.core.pitch < 10:
            wrong_position = True

        if not wrong_position:
            return True

        logger.warning(f"-> Not at {start_label} Limit? Repositioning...")
        fix_success = False
        base_fix_power = context.learning_state.min_power_visible + 15

        for p_try in range(int(base_fix_power), 101, 10):
            fix_power = float(p_try) * float(-kick_direction.value)
            context.core.hw.set_motors(fix_power, fix_power)
            self._sleep_with_update(context, 0.5)
            context.core.hw.stop()
            self._wait_for_settle(context)

            if (kick_direction == Direction.BACKWARD and context.core.pitch < -10) or (
                kick_direction == Direction.FORWARD and context.core.pitch > 10
            ):
                fix_success = True
                break

        if not fix_success:
            logger.warning("-> Reposition failed or confused. Aborting kickup.")
            return False
        return True

    def _attempt_catch(self, context: AgentContext, target_angle: float) -> bool:
        logger.info("-> Attempting Catch...")
        catch_start = time.perf_counter()

        catch_params = TuningParams(
            kp=context.learning_state.pid.kp * 1.5,
            ki=context.learning_state.pid.ki,
            kd=context.learning_state.pid.kd * 2.0,
            target_angle_offset=0.0,
        )

        rate = RateLimiter(1.0 / context.config.loop_time)
        dt = context.config.loop_time
        while time.perf_counter() - catch_start < 2.5:
            if context.watchdog:
                context.watchdog.heartbeat()
            telem = context.core.update(self._zero_motion_enabled, catch_params, dt)

            error = abs(telem.pitch_angle - target_angle)
            if error < 5.0 and abs(telem.pitch_rate) < 30.0:
                pass
            if abs(telem.pitch_angle - target_angle) > 40.0:
                pass
            dt = rate.sleep()

        final_error = abs(context.core.pitch - target_angle)
        if final_error < 10.0:
            logger.info("-> Catch Success!")
            return True
        return False

    def _incremental_kickup(self, context: AgentContext, target_angle: float, start_power: float) -> bool:
        power = start_power
        step = 5.0
        max_power = 100.0

        start_pitch = context.core.pitch
        kick_direction = Direction.BACKWARD if start_pitch < 0 else Direction.FORWARD

        start_label = (
            Orientation.BACK.upper()
            if kick_direction == Direction.BACKWARD
            else Orientation.FRONT.upper()
        )

        logger.info(f"-> Starting Incremental Kick-Up from {start_label}. Target: {target_angle:.2f}")

        try:
            while power <= max_power:
                if context.watchdog:
                    context.watchdog.heartbeat()
                self._wait_for_settle(context)

                if not self._check_and_fix_position(context, kick_direction, start_label):
                    return False

                logger.info(f"-> Kick-Up Attempt: Power {power:.1f} Direction {kick_direction}")

                drive_val = float(power) * float(kick_direction.value)
                context.core.hw.set_motors(drive_val, drive_val)
                self._sleep_with_update(context, 0.25)

                if self._attempt_catch(context, target_angle):
                    return True

                context.core.hw.stop()
                logger.info("-> Catch Failed. Retrying...")
                power += step

        except Exception as e:
            logger.error(f"Kick-Up Exception: {e}")
            context.core.hw.stop()
            return False

        logger.error("-> Failed to Kick-Up (Max Power Reached).")
        return False

    def update(self, context: AgentContext, dt: float, motion_req: MotionRequest, tuning_params: TuningParams, last_telemetry: BalanceTelemetry | None, ticks: int) -> BotState:
        pwr = (
            context.learning_state.control.kickup_power_forward
            if context.core.pitch < 0
            else context.learning_state.control.kickup_power_backward
        )
        success = self._incremental_kickup(context, context.learning_state.pid.target_angle, start_power=pwr)

        if success:
            logger.info("-> Kick-Up Successful! Transition to BALANCING.")
            return BalancingState()
        else:
            if context.core.pitch > 80.0: # Fallback FATAL check for completely unrecoverable orientation
                pass # FATAL checks handled in next state or here
            logger.warning("-> Kick-Up Failed. Transition to IDLE.")
            return IdleState(kickup_attempts=self.attempts + 1)

class BalancingState(BotState):
    def update(self, context: AgentContext, dt: float, motion_req: MotionRequest, tuning_params: TuningParams, last_telemetry: BalanceTelemetry | None, ticks: int) -> BotState:
        motion_req.enable_control = True
        current_pitch = last_telemetry.pitch_angle if last_telemetry else context.core.pitch

        if abs(current_pitch) > context.learning_state.crash_angle:
            logger.warning(f"-> Crash Detected ({current_pitch:.1f} > {context.learning_state.crash_angle}). Transition to CRASHED.")
            context.core.hw.stop()
            motion_req.enable_control = False
            return CrashedState()

        elif last_telemetry:
            # Reusing existing tuning_params instead of local agent logic to pass back modifications
            tune_kp = context.learning_state.pid.kp

            rec_target = context.recovery.update(False, last_telemetry.pitch_angle, tune_kp)
            if rec_target is not None:
                tuning_params.target_angle_offset = rec_target - context.learning_state.pid.target_angle

            curr_error = last_telemetry.pitch_angle - context.learning_state.pid.target_angle
            if rec_target is None:
                adj = context.tuner.update(curr_error)
                if adj.kp != 0 or adj.ki != 0 or adj.kd != 0:
                    context.learning_state.pid.kp = max(0.1, context.learning_state.pid.kp + adj.kp)
                    context.learning_state.pid.ki = max(0.0, context.learning_state.pid.ki + adj.ki)
                    context.learning_state.pid.kd = max(0.0, context.learning_state.pid.kd + adj.kd)
                    # We can't set config_dirty directly without agent, but config will be saved on interval
                    tuning_params.kp, tuning_params.ki, tuning_params.kd = context.learning_state.pid.kp, context.learning_state.pid.ki, context.learning_state.pid.kd

            if (rec_target is None and motion_req.velocity == 0.0 and motion_req.turn_rate == 0.0):
                aggression = 10.0 if not context.learning_state.balance_verified else 1.0
                effort = last_telemetry.motor_output / context.battery.compensation_factor
                if abs(curr_error) < 5.0 and abs(effort) > 5.0 and abs(last_telemetry.pitch_rate) < 20.0:
                    sign = 1 if effort > 0 else -1
                    context.learning_state.pid.target_angle += sign * (context.config.loop_time * aggression)

        return self

class CrashedState(BotState):
    def __init__(self):
        self.crash_time = time.monotonic()

    def update(self, context: AgentContext, dt: float, motion_req: MotionRequest, tuning_params: TuningParams, last_telemetry: BalanceTelemetry | None, ticks: int) -> BotState:
        motion_req.enable_control = False
        context.recovery.update(True, context.core.pitch, context.learning_state.pid.kp)

        if time.monotonic() - self.crash_time > 3.0:
            logger.info("-> Crash Timeout Expired. Transition to IDLE.")
            return IdleState()
        return self

class FatalErrorState(BotState):
    def update(self, context: AgentContext, dt: float, motion_req: MotionRequest, tuning_params: TuningParams, last_telemetry: BalanceTelemetry | None, ticks: int) -> BotState:
        motion_req.enable_control = False
        if ticks % 200 == 0:
            logger.critical("-> FATAL ERROR STATE. PLEASE MANUALLY RESET ROBOT.")
        return self
""")

def update_agent():
    with open('src/balance_bot/behavior/agent.py', 'r') as f:
        content = f.read()

    content = content.replace("from ..enums import Orientation, Direction, BotState", "from ..enums import Orientation, Direction\nfrom .states import AgentContext, BotState, IdleState, KickupState, BalancingState, CrashedState, FatalErrorState")

    content = content.replace("self.state = BotState.IDLE", "self.state = IdleState()")
    content = content.replace("self.state: BotState = BotState.IDLE", "self.state: BotState = IdleState()")

    new_loop_start = """                # Check background tasks (Saving)
                if self.ticks % 10 == 0:
                    self.led.update()
                    if self.config_dirty and (time.monotonic() - self.last_save_time > self.learning_state.timing.save_interval):
                        try:
                            config_snapshot = self.learning_state.model_dump()
                            self.io_executor.submit(self._save_config_worker, config_snapshot)
                            self.last_save_time = time.monotonic()
                            self.config_dirty = False
                        except Exception as e:
                            logger.error(f"Failed to initiate async config save: {e}")

                # Update Battery Logic (Always run to keep voltage filter updated)
                if last_telemetry:
                    ang_accel = (last_telemetry.pitch_rate - last_pitch_rate) / dt
                    last_pitch_rate = last_telemetry.pitch_rate
                    _comp_factor = self.battery.update(last_telemetry.motor_output, ang_accel)
                    if _comp_factor < self.learning_state.control.low_battery_log_threshold and self.battery_logger.should_log():
                        logger.warning(f"-> Low Battery? Compensating: {int(_comp_factor * 100)}%")
                else:
                    # Fallback if no telemetry (e.g. after Kickup)
                    pass

                # STATE PATTERN UPDATE
                context = AgentContext(
                    core=self.core,
                    config=self.hw_config,
                    learning_state=self.learning_state,
                    led=self.led,
                    battery=self.battery,
                    recovery=self.recovery,
                    tuner=self.tuner,
                    watchdog=self.watchdog
                )

                next_state = self.state.update(context, dt, motion_req, tuning_params, last_telemetry, self.ticks)
                if type(next_state) != type(self.state):
                    self.state.exit(context)
                    self.state = next_state
                    self.state.enter(context)
"""

    content = re.sub(
    r'                match self\.state:.*?# ---------------------------------------------------------\n                # EXECUTION\n                # ---------------------------------------------------------',
    new_loop_start, content, flags=re.DOTALL)

    content = re.sub(r'    def _wait_for_settle\(.*?\n        return False\n', '', content, flags=re.DOTALL)
    content = content.replace("self.state.name,", "type(self.state).__name__,")

    with open('src/balance_bot/behavior/agent.py', 'w') as f:
        f.write(content)

def update_main():
    with open('src/balance_bot/main.py', 'r') as f:
        content = f.read()

    if "import mmap" not in content:
        content = "import mmap\n" + content

    new_get_tail = """
def get_tail_telemetry(filepath: str, lines: int = 100) -> str:
    try:
        with open(filepath, "r+b") as f:
            if os.fstat(f.fileno()).st_size == 0:
                return "Telemetry file empty."
            mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)

            # Read header
            header_end = mm.find(b"\\n")
            if header_end == -1:
                return "Telemetry format invalid (no newline)."
            header = mm[:header_end + 1].decode("utf-8")

            # Find the last N lines efficiently without loading the whole file
            # Go to end
            pos = mm.size() - 1
            newline_count = 0

            while pos > header_end and newline_count <= lines:
                if mm[pos] == 10:  # b"\\n"
                    newline_count += 1
                pos -= 1

            if pos <= header_end:
                pos = header_end
            else:
                pos += 2 # Move past the newline

            tail_lines = mm[pos:].decode("utf-8")
            mm.close()
            return header + tail_lines
    except Exception as e:
        return f"Telemetry irrecoverable: {e}"
"""
    content = content.replace("def _reset_robot_memory() -> None:", new_get_tail + "\n\ndef _reset_robot_memory() -> None:")

    new_crash_catch = """    # 5. Capture Telemetry
    telemetry_data = "No telemetry data found."
    if os.path.exists("flight_data.csv"):
        telemetry_data = get_tail_telemetry("flight_data.csv", 100)"""

    content = re.sub(
    r'    # 5\. Capture Telemetry.*?    except Exception as read_err:\n        telemetry_data = f"Failed to read telemetry: \{read_err\}"',
    new_crash_catch, content, flags=re.DOTALL)

    with open('src/balance_bot/main.py', 'w') as f:
        f.write(content)


def update_utils():
    with open('src/balance_bot/utils.py', 'r') as f:
        content = f.read()

    content = content.replace("BUSY_WAIT_THRESHOLD = 0.0015  # 1.5ms", "BUSY_WAIT_THRESHOLD = 0.002  # 2.0ms")

    with open('src/balance_bot/utils.py', 'w') as f:
        f.write(content)

update_types()
update_balance_core()
update_robot_hardware()
update_enums()
create_states()
update_agent()
update_main()
update_utils()
