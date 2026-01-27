import time
import logging
from pathlib import Path

logger = logging.getLogger(__name__)


class LedController:
    def __init__(self):
        self.led_path: Path | None = self._find_led_path()
        self.mode = "OFF"
        self.last_toggle = 0.0
        self.is_on = False
        self.blink_interval = 0.0

        # Turn off initially
        self.set_led(False)

    def _find_led_path(self) -> Path | None:
        candidates = (
            Path("/sys/class/leds/led0/brightness"),
            Path("/sys/class/leds/ACT/brightness"),
        )
        return next((p for p in candidates if p.exists()), None)

    def set_led(self, on: bool) -> None:
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
        """Fast blink for setup/waiting (10Hz)."""
        if self.mode != "SETUP":
            self.mode = "SETUP"
            self.blink_interval = 0.05

    def signal_tuning(self) -> None:
        """Slow blink for tuning (2Hz)."""
        if self.mode != "TUNING":
            self.mode = "TUNING"
            self.blink_interval = 0.25

    def signal_ready(self) -> None:
        """Solid on for balancing."""
        self.mode = "ON"
        self.set_led(True)

    def signal_off(self) -> None:
        self.mode = "OFF"
        self.set_led(False)

    def update(self) -> None:
        if self.mode in ["SETUP", "TUNING"]:
            now = time.time()
            if now - self.last_toggle > self.blink_interval:
                self.set_led(not self.is_on)
                self.last_toggle = now
        elif self.mode == "ON":
            if not self.is_on:
                self.set_led(True)
        elif self.mode == "OFF":
            if self.is_on:
                self.set_led(False)

    def countdown(self) -> None:
        """
        Blocking countdown sequence:
        3 blinks, pause, 2 blinks, pause, 1 blink, go.
        """
        logger.info("Starting in...")

        # 3 Blinks
        for _ in range(3):
            self.set_led(True)
            time.sleep(0.2)
            self.set_led(False)
            time.sleep(0.2)

        time.sleep(0.5)

        # 2 Blinks
        for _ in range(2):
            self.set_led(True)
            time.sleep(0.2)
            self.set_led(False)
            time.sleep(0.2)

        time.sleep(0.5)

        # 1 Blink
        self.set_led(True)
        time.sleep(0.2)
        self.set_led(False)
        time.sleep(0.2)

        logger.info("GO!")
