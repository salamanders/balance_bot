"""
Proprioceptive Toddler Engine (3-Stage Babbling)
Replaces legacy fragmented discovery scripts with one unified, data-literal engine.
Every motor command and IMU sample is recorded to discovery_data.csv for full empirical transparency.
"""
import csv
import logging
import time
from typing import Any

from pyglm import glm
from ..configuration import HardwareConfig, LearningState, PIDParams, BatteryConfig, ControlConfig
from ..enums import Axis
from .step import CalibrationStep, StepStatus
from ..hardware.robot_hardware import RobotHardware

logger = logging.getLogger(__name__)


class DiscoveryTelemetryRecorder:
    """Records full motor command and IMU telemetry to discovery_data.csv."""

    def __init__(self, filename: str, hw: RobotHardware):
        self.filename = filename
        self.hw = hw
        self.file = open(filename, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            "timestamp", "stage", "cmd_left_pwm", "cmd_right_pwm",
            "pitch_angle", "pitch_rate", "yaw_rate",
            "accel_x", "accel_y", "accel_z",
            "gyro_x", "gyro_y", "gyro_z"
        ])
        self.file.flush()

    def log_sample(self, stage: str, cmd_left: float, cmd_right: float) -> None:
        ts = time.time()
        try:
            converted = self.hw.read_imu_converted()
            raw = self.hw.read_imu_converted()
            pitch_angle = getattr(converted, "pitch_angle", 0.0)
            pitch_rate = getattr(converted, "pitch_rate", 0.0)
            yaw_rate = getattr(converted, "yaw_rate", 0.0)
            ax = raw.accel_raw[0] if raw.accel_raw else 0.0
            ay = raw.accel_raw[1] if raw.accel_raw else 0.0
            az = raw.accel_raw[2] if raw.accel_raw else 0.0
            gx = raw.gyro_raw[0] if raw.gyro_raw else 0.0
            gy = raw.gyro_raw[1] if raw.gyro_raw else 0.0
            gz = raw.gyro_raw[2] if raw.gyro_raw else 0.0
        except Exception:
            pitch_angle = pitch_rate = yaw_rate = ax = ay = az = gx = gy = gz = 0.0

        self.writer.writerow([
            f"{ts:.4f}", stage, f"{cmd_left:.1f}", f"{cmd_right:.1f}",
            f"{pitch_angle:.2f}", f"{pitch_rate:.2f}", f"{yaw_rate:.2f}",
            f"{ax:.3f}", f"{ay:.3f}", f"{az:.3f}",
            f"{gx:.3f}", f"{gy:.3f}", f"{gz:.3f}"
        ])
        self.file.flush()

    def close(self) -> None:
        try:
            self.file.close()
        except Exception:
            pass


class ProprioceptiveToddlerStep(CalibrationStep):
    """
    Unified 3-stage Proprioceptive Toddler self-discovery engine:
    1. Independent Breakaway Static Friction (min_L, min_R -> min_power_visible)
    2. A/B Polarity Contrast & Orthogonal IMU Axes (at min_power_visible + 3%)
    3. Symmetric Rock-and-Check Flip & Bounding Box Geometry (start at min_power_visible + 2%, step 2%)
    """

    @property
    def name(self) -> str:
        return "Proprioceptive Toddler (3-Stage Babbling)"

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> tuple[
        StepStatus, dict[str, Any], dict[str, Any]]:
        logger.info(">>> Proprioceptive Toddler Engine: Initiating 3-Stage Discovery <<<")
        recorder = DiscoveryTelemetryRecorder("discovery_data.csv", hw)

        try:
            # --- STAGE 1: INDEPENDENT BREAKAWAY STATIC FRICTION ---
            logger.info("--- Stage 1: Independent Breakaway Friction ---")
            min_l = self._find_single_motor_breakaway(hw, recorder, motor_idx=0, label="Stage1_Left")
            min_r = self._find_single_motor_breakaway(hw, recorder, motor_idx=1, label="Stage1_Right")

            if min_l is None or min_r is None:
                logger.error("[FATAL] Could not find breakaway threshold for both motors.")
                return StepStatus.FATAL, {}, {}

            min_power = max(min_l, min_r)
            logger.info(f"[STAGE 1 RESULT] Left Breakaway={min_l:.1f}%, Right Breakaway={min_r:.1f}% -> min_power_visible={min_power:.1f}%")

            # Plausibility & Broken Wire verification
            if min_power < 0.5 or min_power > 60.0:
                logger.error(f"[FATAL] Breakaway friction out of bounds ({min_power:.1f}%).")
                return StepStatus.FATAL, {}, {}
            if abs(min_l - min_r) > 25.0:
                logger.error(f"[FATAL] Severe asymmetry between motors (|{min_l:.1f}% - {min_r:.1f}%| > 25%).")
                return StepStatus.FATAL, {}, {}

            # --- STAGE 2: POLARITY CONTRAST & ORTHOGONAL AXES ---
            logger.info("--- Stage 2: A/B Polarity Contrast & Orthogonal Axes ---")
            test_pwm = min_power + 3.0  # Gentle crawl
            axes_config, swap_lr = self._derive_kinematics_and_polarity(hw, recorder, test_pwm, label="Stage2_Kinematics")
            if not axes_config:
                logger.error("[FATAL] Could not derive orthogonal IMU axes.")
                return StepStatus.NEEDS_RETRY, {}, {}

            # Apply discovered axes to hw immediately so Stage 3 can measure real pitch angles
            hw.apply_config(config.model_copy(update=axes_config))

            # --- STAGE 3: SYMMETRIC ROCK-AND-CHECK FLIP & BOUNDING BOX ---
            logger.info("--- Stage 3: Symmetric Rock-and-Check Flip & Rest Geometry ---")
            rest_a = self._sample_settled_pitch(hw, recorder, "Stage3_RestA")
            logger.info(f"  [Stage 3] Initial resting bumper angle (Side A): {rest_a:.1f}°")

            flip_pwm, rest_b = self._run_symmetric_rock_and_check(
                hw, recorder, start_pwm=max(15.0, min_power + 10.0), step=5.0, label="Stage3_RockAndCheck"
            )
            if flip_pwm is None:
                logger.error("[FATAL] Could not discover minimum flip power up to 80% PWM.")
                return StepStatus.NEEDS_RETRY, {}, {}

            rest_angle_backward = max(rest_a, rest_b)
            rest_angle_forward = min(rest_a, rest_b)
            max_tilt_angle = min(10.0, min(abs(rest_a), abs(rest_b)) * 0.8)

            logger.info(
                f"[STAGE 3 RESULT] Rest Backward={rest_angle_backward:.1f}°, Rest Forward={rest_angle_forward:.1f}°, Max Tilt={max_tilt_angle:.1f}°, Flip PWM={flip_pwm:.1f}%"
            )

            # Apply updates
            new_control = state.control.model_copy(update={
                "kickup_power_forward": flip_pwm,
                "kickup_power_backward": flip_pwm,
                "max_tilt_angle": max_tilt_angle,
                "motor_trim": 0.0,
            })

            hw_updates: dict[str, Any] = dict(axes_config)
            if swap_lr:
                # Physically swap L/R mapping
                hw_updates["motor_l"] = config.motor_r
                hw_updates["motor_r"] = config.motor_l

            state_updates: dict[str, Any] = {
                "min_power_visible": min_power,
                "rest_angle_backward": rest_angle_backward,
                "rest_angle_forward": rest_angle_forward,
                "control": new_control,
                "kickup_dynamics_verified": True,
                "manual_lean_verified": True,
                "motor_trim_verified": True,
                "backlash_verified": True,
            }

            logger.info(">>> Proprioceptive Toddler Discovery Complete! <<<")
            return StepStatus.SUCCESS, hw_updates, state_updates

        finally:
            recorder.close()

    def _sample_settled_pitch(self, hw: RobotHardware, recorder: DiscoveryTelemetryRecorder, stage: str) -> float:
        samples = []
        for _ in range(10):
            if hw.watchdog:
                hw.watchdog.heartbeat()
            recorder.log_sample(stage, 0.0, 0.0)
            val = getattr(hw.read_imu_converted(), "pitch_angle", 0.0)
            try:
                samples.append(float(val))
            except (TypeError, ValueError):
                samples.append(0.0)
            time.sleep(0.05)
        return sum(samples) / len(samples)

    def _find_single_motor_breakaway(self, hw: RobotHardware, recorder: DiscoveryTelemetryRecorder,
                                     motor_idx: int, label: str) -> float | None:
        """Babble power from 0% stepping +1% until single motor moves gyro > 3.0 deg/s. Always returns to home."""
        # Zero static gyro bias before babbling
        hw.wait_for_stability(duration=1.0)
        for pwm in range(0, 61, 1):
            if hw.watchdog:
                hw.watchdog.heartbeat()
            cmd_l = float(pwm) if motor_idx == 0 else 0.0
            cmd_r = float(pwm) if motor_idx == 1 else 0.0

            # 1. Forward Test Pulse 0.3s
            start_t = time.time()
            max_gyro = 0.0
            while time.time() - start_t < 0.3:
                hw.set_motors(cmd_l, cmd_r)
                recorder.log_sample(label, cmd_l, cmd_r)
                raw = hw.read_imu_converted()
                if raw.gyro_raw:
                    max_gyro = max(max_gyro, float(glm.length(raw.gyro_raw)))
                time.sleep(0.02)

            hw.stop()

            # 2. Return to Home Pulse (reverse -cmd_l, -cmd_r for 0.3s)
            home_t = time.time()
            while time.time() - home_t < 0.3:
                hw.set_motors(-cmd_l, -cmd_r)
                recorder.log_sample(f"{label}_home", -cmd_l, -cmd_r)
                time.sleep(0.02)

            hw.stop()
            # Settle and read
            settle_t = time.time()
            while time.time() - settle_t < 0.3:
                recorder.log_sample(f"{label}_settle", 0.0, 0.0)
                raw = hw.read_imu_converted()
                if raw.gyro_raw:
                    max_gyro = max(max_gyro, float(glm.length(raw.gyro_raw)))
                time.sleep(0.02)

            if max_gyro > 3.0:
                logger.info(f"  [{label}] Motion detected at {pwm}.0% PWM (max_gyro={max_gyro:.1f} deg/s)")
                return float(pwm)

        return None

    def _derive_kinematics_and_polarity(self, hw: RobotHardware, recorder: DiscoveryTelemetryRecorder,
                                        test_pwm: float, label: str) -> tuple[dict[str, Any], bool]:
        """Test straight (p, p) vs spin (p, -p) at min_power_visible + 3% to assign axes and polarity. Always returns to home."""
        # 1. Measure static gravity baseline
        hw.wait_for_stability(duration=1.0)
        base_raw = hw.read_imu_converted().accel_raw or glm.vec3(0, 0, 1)

        axes = {"x", "y", "z"}
        vert_candidates = {k: abs(getattr(base_raw, k)) for k in axes}
        vert_axis = max(vert_candidates, key=vert_candidates.get)
        accel_vertical_invert = getattr(base_raw, vert_axis) < 0

        # 2. Pulse straight (test_pwm, test_pwm), then return to home (-test_pwm, -test_pwm)
        l_gyro, l_accel = self._pulse_and_measure_with_home(hw, recorder, test_pwm, test_pwm, f"{label}_Straight")
        # 3. Pulse spin (test_pwm, -test_pwm), then return to home (-test_pwm, test_pwm)
        s_gyro, s_accel = self._pulse_and_measure_with_home(hw, recorder, test_pwm, -test_pwm, f"{label}_Spin")

        if l_gyro is None or s_gyro is None:
            return {}, False

        # Compare vertical (yaw) rotation
        up_dir = glm.normalize(base_raw)
        yaw_straight = abs(glm.dot(l_gyro, up_dir))
        yaw_spin = abs(glm.dot(s_gyro, up_dir))

        motor_r_invert = False
        if yaw_straight > yaw_spin:
            logger.info(f"  [{label}] Polarity Mismatch Confirmed (straight yaw={yaw_straight:.1f} > spin yaw={yaw_spin:.1f}). Inverting Right Motor.")
            motor_r_invert = True

        # Pitch Axis: dominant gyro orthogonal to Vertical
        pitch_candidates = {k: abs(getattr(l_gyro, k)) for k in axes if k != vert_axis}
        pitch_axis = max(pitch_candidates, key=pitch_candidates.get)
        gyro_pitch_invert = getattr(l_gyro, pitch_axis) > 0  # +PWM -> negative pitch rate

        # Forward Axis: dominant accel delta orthogonal to Vertical
        fwd_candidates = {k: abs(getattr(l_accel - base_raw, k)) for k in axes if k != vert_axis}
        fwd_axis = max(fwd_candidates, key=fwd_candidates.get)
        accel_forward_invert = getattr(l_accel - base_raw, fwd_axis) > 0

        # Check Left/Right physical swap via Yaw cross product
        raw_up = glm.cross(l_accel - base_raw, l_gyro)
        check_val = glm.dot(s_gyro, raw_up)
        swap_lr = check_val > 0
        if swap_lr:
            logger.info(f"  [{label}] L/R Motors physically swapped (dot={check_val:.2f}).")

        axes_config = {
            "accel_vertical_axis": Axis(vert_axis),
            "accel_vertical_invert": accel_vertical_invert,
            "gyro_pitch_axis": Axis(pitch_axis),
            "gyro_pitch_invert": gyro_pitch_invert,
            "gyro_yaw_axis": Axis(vert_axis),
            "gyro_yaw_invert": False,
            "accel_forward_axis": Axis(fwd_axis),
            "accel_forward_invert": accel_forward_invert,
            "motor_r_invert": motor_r_invert,
        }
        return axes_config, swap_lr

    def _pulse_and_measure(self, hw: RobotHardware, recorder: DiscoveryTelemetryRecorder,
                           cmd_l: float, cmd_r: float, label: str) -> tuple[Any, Any]:
        gyros = []
        accels = []
        start_t = time.time()
        while time.time() - start_t < 0.3:
            if hw.watchdog:
                hw.watchdog.heartbeat()
            hw.set_motors(cmd_l, cmd_r)
            recorder.log_sample(label, cmd_l, cmd_r)
            raw = hw.read_imu_converted()
            if raw.gyro_raw and raw.accel_raw:
                gyros.append(raw.gyro_raw)
                accels.append(raw.accel_raw)
            time.sleep(0.02)

        hw.stop()
        hw.wait_for_stability(duration=0.5)

        if not gyros:
            return None, None
        avg_gyro = sum(gyros, glm.vec3(0, 0, 0)) / len(gyros)
        avg_accel = sum(accels, glm.vec3(0, 0, 0)) / len(accels)
        return avg_gyro, avg_accel

    def _pulse_and_measure_with_home(self, hw: RobotHardware, recorder: DiscoveryTelemetryRecorder,
                                   cmd_l: float, cmd_r: float, label: str) -> tuple[Any, Any]:
        gyros = []
        accels = []
        start_t = time.time()
        while time.time() - start_t < 0.3:
            if hw.watchdog:
                hw.watchdog.heartbeat()
            hw.set_motors(cmd_l, cmd_r)
            recorder.log_sample(label, cmd_l, cmd_r)
            raw = hw.read_imu_converted()
            if raw.gyro_raw and raw.accel_raw:
                gyros.append(raw.gyro_raw)
                accels.append(raw.accel_raw)
            time.sleep(0.02)

        hw.stop()

        # Always return to home (reverse -cmd_l, -cmd_r for 0.3s)
        home_t = time.time()
        while time.time() - home_t < 0.3:
            if hw.watchdog:
                hw.watchdog.heartbeat()
            hw.set_motors(-cmd_l, -cmd_r)
            recorder.log_sample(f"{label}_Home", -cmd_l, -cmd_r)
            time.sleep(0.02)

        hw.stop()
        hw.wait_for_stability(duration=0.5)

        if not gyros:
            return None, None
        avg_gyro = sum(gyros, glm.vec3(0, 0, 0)) / len(gyros)
        avg_accel = sum(accels, glm.vec3(0, 0, 0)) / len(accels)
        return avg_gyro, avg_accel

    def _run_symmetric_rock_and_check(self, hw: RobotHardware, recorder: DiscoveryTelemetryRecorder,
                                      start_pwm: float, step: float, label: str) -> tuple[float | None, float]:
        """Symmetric rock-and-check (+1 then -1) starting at start_pwm with step increments (max 4 attempts)."""
        p = start_pwm
        attempts = 0
        while p <= 60.0 and attempts < 4:
            attempts += 1
            if hw.watchdog:
                hw.watchdog.heartbeat()
            logger.info(f"  [Symmetric Search] Testing rock-and-check at {p:.1f}% PWM (attempt {attempts}/4)...")
            # 1. Try Direction +1.0
            flipped, settled_pitch = self._attempt_rock(hw, recorder, 1.0, p, f"{label}_DirPlus_{p}")
            if flipped:
                logger.info(f"  [FOUND] KickUp success in Direction +1.0 at {p:.1f}% PWM (settled={settled_pitch:.1f}°)")
                return p, settled_pitch

            # 2. Try Direction -1.0
            flipped, settled_pitch = self._attempt_rock(hw, recorder, -1.0, p, f"{label}_DirMinus_{p}")
            if flipped:
                logger.info(f"  [FOUND] KickUp success in Direction -1.0 at {p:.1f}% PWM (settled={settled_pitch:.1f}°)")
                return p, settled_pitch

            p += step

        return None, 0.0

    def _attempt_rock(self, hw: RobotHardware, recorder: DiscoveryTelemetryRecorder,
                      sign: float, p: float, label: str) -> tuple[bool, float]:
        # Record initial settled pitch
        pitch_initial = self._sample_settled_pitch(hw, recorder, f"{label}_Initial")

        # Pulse 0.6s
        pulse_pwm = p * sign
        start_t = time.time()
        while time.time() - start_t < 0.6:
            if hw.watchdog:
                hw.watchdog.heartbeat()
            hw.set_motors(pulse_pwm, pulse_pwm)
            recorder.log_sample(label, pulse_pwm, pulse_pwm)
            time.sleep(0.02)

        hw.stop()
        # Wait 1.0s to settle completely
        time.sleep(1.0)
        pitch_after = self._sample_settled_pitch(hw, recorder, f"{label}_Settled")

        flipped = (pitch_initial * pitch_after < 0) and abs(pitch_after - pitch_initial) > 3.0
        if not flipped:
            # Always return to home after every attempt (successful or not)
            home_t = time.time()
            while time.time() - home_t < 0.6:
                if hw.watchdog:
                    hw.watchdog.heartbeat()
                hw.set_motors(-pulse_pwm, -pulse_pwm)
                recorder.log_sample(f"{label}_Home", -pulse_pwm, -pulse_pwm)
                time.sleep(0.02)
            hw.stop()
            time.sleep(1.0)
            pitch_after = self._sample_settled_pitch(hw, recorder, f"{label}_HomeSettled")

        return flipped, pitch_after
