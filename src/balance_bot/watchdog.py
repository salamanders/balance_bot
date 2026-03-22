import threading
import time
import _thread
import logging

class SurvivalWatchdog:
    """
    A high-priority background thread that monitors the main organism's "heartbeat".
    If the main thread fails to check in within the timeout (due to deadlock,
    infinite loop, or blocking I/O), this watchdog triggers a panic by raising
    a KeyboardInterrupt in the main thread.
    """
    def __init__(self, timeout: float = 15.0) -> None:
        self.timeout = timeout
        self.last_heartbeat = time.monotonic()
        self.running = True
        self.triggered = False
        self.thread = threading.Thread(target=self._watch, daemon=True)
        self.thread.start()

    def heartbeat(self) -> None:
        """The main loop calls this to say 'I am still alive and thinking.'"""
        self.last_heartbeat = time.monotonic()

    def stop(self) -> None:
        """Stop the watchdog thread gracefully."""
        self.running = False

    def _watch(self) -> None:
        while self.running:
            time.sleep(1.0)
            if time.monotonic() - self.last_heartbeat > self.timeout:
                logging.error("WATCHDOG TRIGGERED: Organism is paralyzed/stuck! Initiating panic response.")
                self.triggered = True

                # This injects a fatal exception directly into the main thread.
                # It shatters ANY while True loop the main thread is stuck in.
                _thread.interrupt_main()

                # The thread dies after doing its job
                break
