import time
import logging
from pathlib import Path

from .config import LedConfig

logger = logging.getLogger(__name__)


class LedController:
    """
    Controls the robot's status LED (usually the green ACT LED on a Raspberry Pi).

    Provides visual feedback for different robot states:
    - Setup/Calibrate: Fast Blink.
    - Tuning: Slow Blink.
    - Balanced: Solid On.
    - Off/Recover: Off.
    - Countdown: Specific blink sequence.
    """

    def __init__(self, config: LedConfig = LedConfig()):
        """
        Initialize the LED controller.

        :param config: Configuration object for LED timings and counts.
        """
        self.config = config

        self.led_path: Path | None = self._find_led_path()
        """Path to the system LED brightness control file."""

        self.mode = "OFF"
        """Current operating mode (OFF, SETUP, TUNING, ON)."""

        self.last_toggle = 0.0
        """Timestamp of the last blink toggle."""

        self.is_on = False
        """Current physical state of the LED."""

        self.blink_interval = 0.0
        """Target interval for the current blink mode (Seconds)."""

        # Turn off initially to ensure known state
        self.set_led(False)

    def _find_led_path(self) -> Path | None:
        """
        Find the system path for the status LED.

        Checks common locations for Raspberry Pi LEDs.
        :return: Path object if found, else None.
        """
        candidates = (
            Path("/sys/class/leds/led0/brightness"), # Pi Zero / Pi 3
            Path("/sys/class/leds/ACT/brightness"),  # Newer Kernels
        )
        return next((p for p in candidates if p.exists()), None)

    def set_led(self, on: bool) -> None:
        """
        Turn the LED hardware on or off.

        :param on: True for ON (Brightness 1/255), False for OFF (0).
        """
        self.is_on = on
        if not self.led_path:
            return

        val = "1" if on else "0"
        try:
            self.led_path.write_text(val)
        except (PermissionError, OSError):
            # PermissionError: Happens if not running as root or udev rules missing.
            # OSError: Hardware issue.
            # We silently ignore these to prevent crashing the main control loop.
            pass

    def signal_setup(self) -> None:
        """
        Set mode to SETUP (Fast Blink).
        Used during initialization and calibration.
        """
        if self.mode != "SETUP":
            self.mode = "SETUP"
            self.blink_interval = self.config.setup_blink_interval

    def signal_tuning(self) -> None:
        """
        Set mode to TUNING (Medium/Slow Blink).
        Used during the auto-tuning phase.
        """
        if self.mode != "TUNING":
            self.mode = "TUNING"
            self.blink_interval = self.config.tuning_blink_interval

    def signal_ready(self) -> None:
        """
        Set mode to ON (Solid).
        Used when the robot is actively balancing.
        """
        self.mode = "ON"
        self.set_led(True)

    def signal_off(self) -> None:
        """
        Set mode to OFF.
        Used when motors are stopped or recovering.
        """
        self.mode = "OFF"
        self.set_led(False)

    def update(self) -> None:
        """
        Update LED state based on current mode and time.

        Must be called periodically in the main loop to handle blinking.
        Non-blocking.
        """
        if self.mode in ["SETUP", "TUNING"]:
            now = time.monotonic()
            if now - self.last_toggle > self.blink_interval:
                self.set_led(not self.is_on)
                self.last_toggle = now
        elif self.mode == "ON":
            if not self.is_on:
                self.set_led(True)
        elif self.mode == "OFF":
            if self.is_on:
                self.set_led(False)

    def _blink(self, count: int, on_time: float, off_time: float) -> None:
        """
        Helper for blocking blink sequences.

        :param count: Number of blinks.
        :param on_time: Duration On (Seconds).
        :param off_time: Duration Off (Seconds).
        """
        for _ in range(count):
            self.set_led(True)
            time.sleep(on_time)
            self.set_led(False)
            time.sleep(off_time)

    def countdown(self) -> None:
        """
        Execute the blocking countdown sequence before balancing starts.

        Sequence:
        - 3 Blinks
        - Pause
        - 2 Blinks
        - Pause
        - 1 Blink
        """
        logger.info("Starting in...")

        self._blink(
            self.config.countdown_blink_count_3,
            self.config.countdown_blink_on_time,
            self.config.countdown_blink_off_time,
        )
        time.sleep(self.config.countdown_pause_time)

        self._blink(
            self.config.countdown_blink_count_2,
            self.config.countdown_blink_on_time,
            self.config.countdown_blink_off_time,
        )
        time.sleep(self.config.countdown_pause_time)

        self._blink(
            self.config.countdown_blink_count_1,
            self.config.countdown_blink_on_time,
            self.config.countdown_blink_off_time,
        )

        logger.info("GO!")
