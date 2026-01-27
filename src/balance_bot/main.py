import time

from .config import RobotConfig
from .robot_hardware import RobotHardware
from .pid import PIDController
from .leds import LedController
from .tuner import ContinuousTuner
from .battery import BatteryEstimator

# Constants
COMPLEMENTARY_ALPHA = 0.98
FALL_ANGLE_LIMIT = 45.0
VIBRATION_THRESHOLD = 10
SETUP_WAIT_SEC = 2.0
CALIBRATION_PAUSE_SEC = 1.0
SAVE_INTERVAL_SEC = 30.0


class RobotController:
    def __init__(self):
        self.config = RobotConfig.load()
        self.hw = RobotHardware(
            motor_l=self.config.motor_l,
            motor_r=self.config.motor_r,
            invert_l=self.config.motor_l_invert,
            invert_r=self.config.motor_r_invert,
            gyro_axis=self.config.gyro_pitch_axis,
            gyro_invert=self.config.gyro_pitch_invert,
        )
        self.led = LedController()
        self.pid = PIDController(self.config.pid)
        self.tuner = ContinuousTuner()
        self.battery = BatteryEstimator()

        self.running = True
        self.pitch = 0.0
        self.last_save_time = time.monotonic()
        self.config_dirty = False

        # State for tuning
        self.vibration_counter = 0

    def init(self) -> None:
        self.hw.init()
        print(">>> ROBOT ALIVE. Hold vertical for STEP 1.")

    def get_pitch(self, dt: float) -> tuple[float, float, float]:
        acc_angle, gyro_rate, yaw_rate = self.hw.read_imu_processed()

        # Complementary Filter
        self.pitch = (COMPLEMENTARY_ALPHA * (self.pitch + gyro_rate * dt)) + (
            (1.0 - COMPLEMENTARY_ALPHA) * acc_angle
        )

        return self.pitch, gyro_rate, yaw_rate

    def run(self) -> None:
        # Initial Setup Phase
        self.led.signal_setup()
        start_wait = time.monotonic()
        while time.monotonic() - start_wait < SETUP_WAIT_SEC:
            self.led.update()
            time.sleep(self.config.loop_time)

        try:
            # Mode dispatch
            self.run_calibrate()
            self.run_tune()

            # Main Balance Loop
            self.run_balance()

        except KeyboardInterrupt:
            pass
        finally:
            if self.config_dirty:
                self.config.save()
            self.hw.stop()
            self.led.signal_off()
            self.hw.cleanup()
            print("\nMotors Stopped.")

    def run_calibrate(self) -> None:
        print("-> Calibrating Vertical...")
        self.led.signal_setup()

        # Update pitch once to get initial reading
        self.get_pitch(self.config.loop_time)
        self.config.pid.target_angle = self.pitch

        print(f"-> Calibrated Vertical at: {self.config.pid.target_angle:.2f}")
        print("-> Let it wobble gently (Step 2)...")

        # Pause
        pause_start = time.monotonic()
        while time.monotonic() - pause_start < CALIBRATION_PAUSE_SEC:
            self.led.update()
            time.sleep(self.config.loop_time)

    def run_tune(self) -> None:
        print("-> Auto-Tuning...")
        self.led.signal_tuning()

        while self.running:
            loop_start = time.monotonic()

            pitch, _, _ = self.get_pitch(self.config.loop_time)
            self.led.update()

            error = self.config.pid.target_angle - pitch

            # Tune Logic: Increment Kp until vibration
            self.config.pid.kp += 0.05

            # Vibration detection
            if (error > 0 and self.pid.last_error < 0) or (
                error < 0 and self.pid.last_error > 0
            ):
                self.vibration_counter += 1

            if self.vibration_counter > VIBRATION_THRESHOLD:
                self.config.pid.kp *= 0.6
                self.config.pid.kd = self.config.pid.kp * 0.05
                self.config.pid.ki = self.config.pid.kp * 0.005
                print(
                    f"-> Tuned! Kp={self.config.pid.kp:.2f} Kd={self.config.pid.kd:.2f}"
                )
                self.config.save()
                self.vibration_counter = 0
                return  # Exit tuning, go to balance

            output = self.config.pid.kp * error
            # Simple P loop for tuning, no turn correction
            self.hw.set_motors(output, output)
            self.pid.last_error = error

            self._maintain_loop_timing(loop_start)

    def run_balance(self) -> None:
        print("-> Balancing...")
        self.led.signal_ready()

        last_pitch_rate = 0.0

        while self.running:
            loop_start = time.monotonic()

            pitch, pitch_rate, yaw_rate = self.get_pitch(self.config.loop_time)
            self.led.update()

            error = self.config.pid.target_angle - pitch

            # Fall detection
            if abs(error) > FALL_ANGLE_LIMIT:
                print("!!! FELL OVER !!!")
                self.hw.stop()
                if self.config_dirty:
                    self.config.save()
                    self.config_dirty = False

                self.run_recover()
                # Restore state after recover
                self.led.signal_ready()
                self.pid.reset()
                continue

            # PID Update
            output = self.pid.update(error, self.config.loop_time)

            # Turn Correction
            turn_correction = -yaw_rate * 0.5

            # Continuous Tuning
            kp_n, ki_n, kd_n = self.tuner.update(error)
            if kp_n != 0 or ki_n != 0 or kd_n != 0:
                self.config.pid.kp = max(0.1, self.config.pid.kp + kp_n)
                self.config.pid.ki = max(0.0, self.config.pid.ki + ki_n)
                self.config.pid.kd = max(0.0, self.config.pid.kd + kd_n)
                self.config_dirty = True
                print(
                    f"-> Tuned: P={self.config.pid.kp:.2f} I={self.config.pid.ki:.3f} D={self.config.pid.kd:.2f}"
                )

            # Battery Estimation
            ang_accel = (pitch_rate - last_pitch_rate) / self.config.loop_time
            last_pitch_rate = pitch_rate

            comp_factor = self.battery.update(output, ang_accel, self.config.loop_time)
            # Log low battery occasionally
            if comp_factor < 0.95 and (time.monotonic() * 10) % 50 < 1:
                print(f"-> Low Battery? Compensating: {int(comp_factor * 100)}%")

            # Drive
            final_drive = output / comp_factor
            self.hw.set_motors(
                final_drive + turn_correction, final_drive - turn_correction
            )

            # Periodic Save
            if self.config_dirty and (time.monotonic() - self.last_save_time > SAVE_INTERVAL_SEC):
                self.config.save()
                self.last_save_time = time.monotonic()
                self.config_dirty = False

            self._maintain_loop_timing(loop_start)

    def run_recover(self) -> None:
        self.led.signal_off()
        self.hw.stop()

        while self.running:
            loop_start = time.monotonic()
            pitch, _, _ = self.get_pitch(self.config.loop_time)
            self.led.update()

            error = self.config.pid.target_angle - pitch
            if abs(error) < 5.0:
                print(f"-> Upright detected! (Pitch: {pitch:.2f})")
                self.led.countdown()
                return  # Back to balance

            self._maintain_loop_timing(loop_start)

    def _maintain_loop_timing(self, loop_start: float) -> None:
        elapsed = time.monotonic() - loop_start
        if elapsed < self.config.loop_time:
            time.sleep(self.config.loop_time - elapsed)


def main() -> None:
    bot = RobotController()
    bot.init()
    bot.run()


if __name__ == "__main__":
    main()
