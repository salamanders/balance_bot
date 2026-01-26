import time
import os


class LedController:
    def __init__(self):
        self.led_path = self._find_led_path()
        self.mode = "OFF"
        self.last_toggle = 0.0
        self.is_on = False
        self.blink_interval = 0.0

        # Turn off initially
        self.set_led(False)

    def _find_led_path(self):
        paths = ["/sys/class/leds/led0/brightness", "/sys/class/leds/ACT/brightness"]
        for path in paths:
            if os.path.exists(path):
                return path
        return None

    def set_led(self, on):
        self.is_on = on
        if not self.led_path:
            return

        val = "1" if on else "0"
        try:
            with open(self.led_path, "w") as f:
                f.write(val)
        except (PermissionError, OSError):
            pass

    def signal_setup(self):
        """Fast blink for setup/waiting (10Hz)."""
        if self.mode != "SETUP":
            self.mode = "SETUP"
            self.blink_interval = 0.05

    def signal_tuning(self):
        """Slow blink for tuning (2Hz)."""
        if self.mode != "TUNING":
            self.mode = "TUNING"
            self.blink_interval = 0.25

    def signal_ready(self):
        """Solid on for balancing."""
        self.mode = "ON"
        self.set_led(True)

    def signal_off(self):
        self.mode = "OFF"
        self.set_led(False)

    def update(self):
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

    def countdown(self):
        """
        Blocking countdown sequence:
        3 blinks, pause, 2 blinks, pause, 1 blink, go.
        """
        print("\nStarting in...")

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

        print("\nGO!")
