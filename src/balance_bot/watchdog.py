"""
# System Context
This module is part of the `balance_bot` application, designed to control a self-balancing
homebrew robot. It relies on a deterministic, high-frequency control loop and pessimistic hardware interactions.

# Business Rules
- Fail-fast initialization: The system must crash loudly if physical hardware is missing or unresponsive during boot.
- Fault-tolerant control loop: Once Tier 1 is running (e.g., `BalanceCore`), transient I/O errors must not collapse the system; use continuous data quality metrics instead of fatal exceptions.
- Physical pessimism: Never hardcode physical constants; rely on zero-knowledge self-discovery to deduce configuration.

# Dependency Maps
- Relies on internal configuration (`HardwareConfig`, `LearningState`).
- Interfaces with Tier 1 (`BalanceCore`), Tier 3 (`Agent`), and physical hardware abstraction (`RobotHardware`).
"""

import _thread
import logging
import threading
import time


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
