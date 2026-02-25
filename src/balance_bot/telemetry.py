import time
import csv
from pathlib import Path
from queue import Queue
from threading import Thread
import logging

logger = logging.getLogger(__name__)

class TelemetryBlackbox:
    def __init__(self, filename="flight_data.csv"):
        self.filename = Path(filename)
        self.queue = Queue(maxsize=2000) # Buffer to prevent I/O blocking
        self.running = False
        self.worker = Thread(target=self._writer_thread, daemon=True)

        # Initialize CSV with headers
        with open(self.filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                "timestamp", "state", "pitch_angle", "pitch_rate",
                "yaw_rate", "left_pwm", "right_pwm", "pid_target"
            ])

    def start(self):
        self.running = True
        self.worker.start()
        logger.info(f"Telemetry Blackbox recording to {self.filename}")

    def log_tick(self, state_name, pitch, pitch_rate, yaw_rate, left_pwm, right_pwm, pid_target):
        if not self.running: return

        try:
            self.queue.put_nowait((
                time.time(), state_name, round(pitch, 2), round(pitch_rate, 2),
                round(yaw_rate, 2), left_pwm, right_pwm, round(pid_target, 2)
            ))
        except Queue.Full:
            pass # Drop frame rather than block the 100Hz control loop

    def _writer_thread(self):
        while self.running or not self.queue.empty():
            batch = []
            while not self.queue.empty() and len(batch) < 50:
                batch.append(self.queue.get())

            if batch:
                with open(self.filename, 'a', newline='') as f:
                    csv.writer(f).writerows(batch)
            else:
                time.sleep(0.1)

    def stop(self):
        self.running = False
        if self.worker.is_alive():
            self.worker.join(timeout=1.0)
