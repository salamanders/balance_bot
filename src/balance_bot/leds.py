import time
import logging
from pathlib import Path

from .config import LedConfig

logger = logging.getLogger(__name__)


class LedController:
    """Controls the robot's LED for status indication."""

    def __init__(self, config: LedConfig = LedConfig()):
        """
        Initialize the LED controller.
        :param config: Configuration for LED timings.
        """
        self.config = config
        self.led_path: Path | None = self._find_led_path()
        self.mode = "OFF"
        self.last_toggle = 0.0
        self.is_on = False
        self.blink_interval = 0.0

        # Turn off initially
        self.set_led(False)

    def _find_led_path(self) -> Path | None:
        """Find the system path for the status LED."""
        candidates = (
            Path("/sys/class/leds/led0/brightness"),
            Path("/sys/class/leds/ACT/brightness"),
        )
        return next((p for p in candidates if p.exists()), None)

    def set_led(self, on: bool) -> None:
        """
        Turn the LED on or off.
        :param on: True for ON, False for OFF.
        """
        self.is_on = on
        if not self.led_path:
            return

        val = "1" if on else "0"
        try:
            self.led_path.write_text(val)
        except (PermissionError, OSError):
            # Often happens if permissions aren't set or we're not on Pi
            pass

    def signal_setup(self) -> None:
        """Fast blink for setup/waiting."""
        if self.mode != "SETUP":
            self.mode = "SETUP"
            self.blink_interval = self.config.setup_blink_interval

    def signal_tuning(self) -> None:
        """Slow blink for tuning."""
        if self.mode != "TUNING":
            self.mode = "TUNING"
            self.blink_interval = self.config.tuning_blink_interval

    def signal_ready(self) -> None:
        """Solid on for balancing."""
        self.mode = "ON"
        self.set_led(True)

    def signal_off(self) -> None:
        """Turn off LED."""
        self.mode = "OFF"
        self.set_led(False)

    def update(self) -> None:
        """Update LED state based on current mode and time."""
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
        """Helper for blocking blink sequences."""
        for _ in range(count):
            self.set_led(True)
            time.sleep(on_time)
            self.set_led(False)
            time.sleep(off_time)

    def countdown(self) -> None:
        """
        Blocking countdown sequence before balancing starts.
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
