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
from typing import Any


class SurvivalWatchdog:
    """
    A high-priority background thread that monitors the main organism's "heartbeat",
    enforces global experiment duration limits, and connects to the Deadman's Switch.

    If the main thread fails to check in within the timeout (due to deadlock,
    infinite loop, or blocking I/O), or if the global experiment duration expires,
    or if the Deadman switch triggers an emergency stop, this watchdog raises
    a KeyboardInterrupt in the main thread.
    """

    def __init__(
        self,
        timeout: float = 15.0,
        experiment_duration: float | None = None,
        deadman_server: Any = None,
    ) -> None:
        self.timeout = timeout
        self.experiment_duration = experiment_duration
        self.deadman_server = deadman_server
        self.start_time = time.monotonic()
        self.last_heartbeat = self.start_time
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
            time.sleep(0.2)
            now = time.monotonic()

            # 1. Global Experiment Duration Limit (Rule 4: Global vs. Local Cutoffs)
            if self.experiment_duration is not None and (
                now - self.start_time > self.experiment_duration
            ):
                logging.info(
                    f"EXPERIMENT TIMEOUT: Global budget ({self.experiment_duration:.1f}s) expired. Halting cleanly."
                )
                self.triggered = True
                _thread.interrupt_main()
                break

            # 2. Deadman Switch E-Stop
            if self.deadman_server is not None and getattr(
                self.deadman_server, "estop_triggered", False
            ):
                logging.critical(
                    "WATCHDOG TRIGGERED: Deadman E-Stop activated! Initiating emergency halt."
                )
                self.triggered = True
                _thread.interrupt_main()
                break

            # 3. Main thread liveliness check
            if now - self.last_heartbeat > self.timeout:
                logging.error(
                    "WATCHDOG TRIGGERED: Organism is paralyzed/stuck! Initiating panic response."
                )
                self.triggered = True

                # This injects a fatal exception directly into the main thread.
                # It shatters ANY while True loop the main thread is stuck in.
                _thread.interrupt_main()

                # The thread dies after doing its job
                break
