"""
# System Context
This module is part of the `balance_bot` application, designed to control a self-balancing
homebrew robot. It relies on a deterministic, high-frequency control loop and pessimistic hardware interactions.

# Business Rules
- Physical Safety: All physical experiments in living room environments must be time-bounded or protected by an active Deadman's Switch.
- Fail-safe disarm: If the operator releases the button, closes the tab, or network connection drops for >2.0s, actuators must be immediately disarmed.
- Zero Control Loop Contention: The HTTP server runs on a low-priority background daemon thread and never performs blocking I/O on Tier 1 (BalanceCore).

# Dependency Maps
- Interfaces with `SurvivalWatchdog`, `RobotHardware`, `Agent`, and `main.py`.
"""

import contextlib
import http.server
import json
import logging
import threading
import time
from typing import Any

logger = logging.getLogger(__name__)

DEADMAN_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>Balance Bot - Deadman's Switch</title>
    <style>
        * { box-sizing: border-box; user-select: none; -webkit-user-select: none; margin: 0; padding: 0; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
            background-color: #121214;
            color: #ececec;
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: space-between;
            min-height: 100vh;
            padding: 1.5rem;
            touch-action: manipulation;
        }
        header {
            text-align: center;
            margin-bottom: 1rem;
        }
        h1 { font-size: 1.5rem; font-weight: 700; color: #fff; }
        .subtitle { font-size: 0.85rem; color: #888; margin-top: 0.25rem; }

        .status-card {
            background: #1e1e24;
            border: 1px solid #2e2e38;
            border-radius: 12px;
            padding: 1rem 1.5rem;
            width: 100%;
            max-width: 400px;
            margin-bottom: 1.5rem;
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 0.75rem;
        }
        .status-item { display: flex; flex-direction: column; }
        .status-label { font-size: 0.75rem; color: #888; text-transform: uppercase; letter-spacing: 0.5px; }
        .status-value { font-size: 1.1rem; font-weight: 600; color: #fff; margin-top: 0.2rem; }
        .badge {
            display: inline-block;
            padding: 0.2rem 0.6rem;
            border-radius: 999px;
            font-size: 0.75rem;
            font-weight: 700;
        }
        .badge-disarmed { background: #3a1a1a; color: #ff6b6b; border: 1px solid #ff4444; }
        .badge-active { background: #1a3a22; color: #51cf66; border: 1px solid #2f9e44; }
        .badge-estop { background: #500; color: #fff; border: 1px solid #f00; animation: flash 0.8s infinite; }

        @keyframes flash {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }

        .controls-container {
            display: flex;
            flex-direction: column;
            align-items: center;
            width: 100%;
            max-width: 400px;
            flex: 1;
            justify-content: center;
            gap: 1.5rem;
        }

        .deadman-btn {
            width: 240px;
            height: 240px;
            border-radius: 50%;
            border: 6px solid #444;
            background: radial-gradient(circle at 35% 35%, #4a1515, #1f0707);
            color: #ff8787;
            font-size: 1.25rem;
            font-weight: 800;
            text-transform: uppercase;
            letter-spacing: 1px;
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            gap: 0.5rem;
            box-shadow: 0 10px 30px rgba(0,0,0,0.6), inset 0 2px 10px rgba(255,255,255,0.1);
            cursor: pointer;
            transition: all 0.15s ease;
            outline: none;
            -webkit-tap-highlight-color: transparent;
        }
        .deadman-btn .subtext { font-size: 0.7rem; font-weight: 500; color: #aaa; text-transform: none; }

        .deadman-btn.active {
            background: radial-gradient(circle at 35% 35%, #1b6329, #0d3314);
            border-color: #40c057;
            color: #b2f2bb;
            box-shadow: 0 0 40px rgba(64, 192, 87, 0.6), inset 0 2px 15px rgba(255,255,255,0.3);
            transform: scale(0.96);
        }
        .deadman-btn.active .subtext { color: #d3f9d8; }

        .estop-btn {
            background: #c92a2a;
            color: #fff;
            border: none;
            border-radius: 8px;
            padding: 0.85rem 2rem;
            font-size: 1rem;
            font-weight: 700;
            letter-spacing: 1px;
            cursor: pointer;
            width: 100%;
            max-width: 320px;
            text-transform: uppercase;
            box-shadow: 0 4px 15px rgba(201, 42, 42, 0.4);
            transition: background 0.15s ease;
        }
        .estop-btn:active { background: #a61e1e; transform: scale(0.98); }

        footer {
            font-size: 0.75rem;
            color: #666;
            text-align: center;
            margin-top: 1rem;
        }
    </style>
</head>
<body>
    <header>
        <h1>⚡ BALANCE BOT SAFETY</h1>
        <div class="subtitle">Living Room Deadman's Switch</div>
    </header>

    <div class="status-card">
        <div class="status-item">
            <span class="status-label">Switch State</span>
            <span id="switch-state" class="status-value"><span class="badge badge-disarmed">DISARMED</span></span>
        </div>
        <div class="status-item">
            <span class="status-label">Robot Posture</span>
            <span id="robot-state" class="status-value">--</span>
        </div>
        <div class="status-item">
            <span class="status-label">Pitch Angle</span>
            <span id="pitch-angle" class="status-value">0.0°</span>
        </div>
        <div class="status-item">
            <span class="status-label">Heartbeat Lag</span>
            <span id="hb-age" class="status-value">--</span>
        </div>
    </div>

    <div class="controls-container">
        <button id="deadman-btn" class="deadman-btn">
            <span>HOLD TO RUN</span>
            <span class="subtext">Press & hold to enable motors</span>
        </button>

        <button id="estop-btn" class="estop-btn">🛑 EMERGENCY STOP</button>
    </div>

    <footer>
        Releasing button halts motors within 2.0s.<br>
        Closing tab, locking screen, or lost Wi-Fi triggers fail-safe stop.
    </footer>

    <script>
        let isHolding = false;
        let heartbeatInterval = null;
        const deadmanBtn = document.getElementById('deadman-btn');
        const estopBtn = document.getElementById('estop-btn');
        const switchStateEl = document.getElementById('switch-state');
        const robotStateEl = document.getElementById('robot-state');
        const pitchAngleEl = document.getElementById('pitch-angle');
        const hbAgeEl = document.getElementById('hb-age');

        async function sendHeartbeat() {
            try {
                const res = await fetch('/heartbeat', { method: 'POST' });
                const data = await res.json();
                updateUI(data);
            } catch (err) {
                console.error('Heartbeat error:', err);
                switchStateEl.innerHTML = '<span class="badge badge-disarmed">DISCONNECTED</span>';
            }
        }

        async function sendRelease() {
            try {
                const res = await fetch('/release', { method: 'POST' });
                const data = await res.json();
                updateUI(data);
            } catch (err) {
                console.error('Release error:', err);
            }
        }

        async function sendEStop() {
            try {
                const res = await fetch('/estop', { method: 'POST' });
                const data = await res.json();
                updateUI(data);
            } catch (err) {
                console.error('E-Stop error:', err);
            }
        }

        async function fetchStatus() {
            try {
                const res = await fetch('/status');
                const data = await res.json();
                updateUI(data);
            } catch (err) {
                switchStateEl.innerHTML = '<span class="badge badge-disarmed">OFFLINE</span>';
            }
        }

        function updateUI(data) {
            if (!data) return;
            if (data.estop_triggered) {
                switchStateEl.innerHTML = '<span class="badge badge-estop">E-STOPPED</span>';
            } else if (data.is_alive) {
                switchStateEl.innerHTML = '<span class="badge badge-active">ACTIVE / ARMED</span>';
            } else {
                switchStateEl.innerHTML = '<span class="badge badge-disarmed">DISARMED</span>';
            }

            if (data.posture) robotStateEl.textContent = data.posture;
            if (data.pitch !== undefined) pitchAngleEl.textContent = `${data.pitch.toFixed(1)}°`;
            if (data.heartbeat_age !== undefined) {
                hbAgeEl.textContent = data.heartbeat_age < 99 ? `${data.heartbeat_age.toFixed(1)}s` : '--';
            }
        }

        function startHold(e) {
            if (e) e.preventDefault();
            if (isHolding) return;
            isHolding = true;
            deadmanBtn.classList.add('active');
            deadmanBtn.querySelector('span').textContent = 'RUNNING';
            deadmanBtn.querySelector('.subtext').textContent = 'Release to STOP';

            sendHeartbeat();
            if (heartbeatInterval) clearInterval(heartbeatInterval);
            heartbeatInterval = setInterval(sendHeartbeat, 500);
        }

        function endHold(e) {
            if (e) e.preventDefault();
            if (!isHolding) return;
            isHolding = false;
            deadmanBtn.classList.remove('active');
            deadmanBtn.querySelector('span').textContent = 'HOLD TO RUN';
            deadmanBtn.querySelector('.subtext').textContent = 'Press & hold to enable motors';

            if (heartbeatInterval) {
                clearInterval(heartbeatInterval);
                heartbeatInterval = null;
            }
            sendRelease();
        }

        // Pointer / Touch / Mouse Events
        deadmanBtn.addEventListener('mousedown', startHold);
        window.addEventListener('mouseup', endHold);
        deadmanBtn.addEventListener('touchstart', startHold, { passive: false });
        window.addEventListener('touchend', endHold, { passive: false });
        window.addEventListener('touchcancel', endHold, { passive: false });

        // Safety listeners for tab switch, screen lock, window blur
        window.addEventListener('blur', endHold);
        document.addEventListener('visibilitychange', () => {
            if (document.hidden) endHold();
        });
        window.addEventListener('beforeunload', sendRelease);

        estopBtn.addEventListener('click', (e) => {
            e.preventDefault();
            endHold();
            sendEStop();
        });

        // Status polling loop (2Hz) when not actively sending heartbeats
        setInterval(() => {
            if (!isHolding) {
                fetchStatus();
            }
        }, 1000);

        fetchStatus();
    </script>
</body>
</html>
"""


class DeadmanHandler(http.server.BaseHTTPRequestHandler):
    """Minimal HTTP Request Handler for the Deadman's Switch."""

    server: "DeadmanServer"  # Type annotation for reference to custom server instance

    def log_message(self, format: str, *args: Any) -> None:
        """Suppress default HTTP access logs to prevent console clutter."""
        pass

    def _send_json(self, status_code: int, payload: dict[str, Any]) -> None:
        content = json.dumps(payload).encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(content)

    def do_GET(self) -> None:
        if self.path in ("/", "/index.html"):
            content = DEADMAN_HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(content)))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == "/status":
            self._send_json(200, self.server.get_status_dict())
        else:
            self.send_error(404, "Not Found")

    def do_POST(self) -> None:
        if self.path == "/heartbeat":
            self.server.receive_heartbeat()
            self._send_json(200, self.server.get_status_dict())
        elif self.path == "/release":
            self.server.release()
            self._send_json(200, self.server.get_status_dict())
        elif self.path == "/estop":
            self.server.estop()
            self._send_json(200, self.server.get_status_dict())
        else:
            self.send_error(404, "Not Found")


class DeadmanServer(http.server.ThreadingHTTPServer):
    """
    HTTP Deadman Switch Server running on a background daemon thread.
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 8090, timeout: float = 2.0):
        super().__init__((host, port), DeadmanHandler)
        self.deadman_timeout = timeout
        self.last_heartbeat_time = 0.0
        self.is_armed = False
        self.estop_triggered = False
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self.posture_provider = None
        self.pitch_provider = None

    def start(self) -> None:
        """Start the HTTP server on a background daemon thread."""
        self._thread = threading.Thread(target=self.serve_forever, daemon=True)
        host = str(self.server_address[0])
        logger.info(
            f"⚡ HTTP Deadman's Switch Server listening on http://{host}:{self.server_port}"
        )

    def stop(self) -> None:
        """Shutdown the server cleanly."""
        self.shutdown()
        self.server_close()

    def receive_heartbeat(self) -> None:
        """Called on POST /heartbeat from operator's browser."""
        with self._lock:
            if not self.estop_triggered:
                self.last_heartbeat_time = time.monotonic()
                self.is_armed = True

    def release(self) -> None:
        """Called when operator explicitly releases the button."""
        with self._lock:
            self.is_armed = False
            self.last_heartbeat_time = 0.0

    def estop(self) -> None:
        """Called on emergency stop trigger."""
        with self._lock:
            self.estop_triggered = True
            self.is_armed = False
            self.last_heartbeat_time = 0.0
        logger.critical("🛑 EMERGENCY STOP TRIGGERED VIA DEADMAN WEB INTERFACE!")

    def reset_estop(self) -> None:
        """Reset emergency stop state."""
        with self._lock:
            self.estop_triggered = False
            self.is_armed = False
            self.last_heartbeat_time = 0.0

    def is_alive(self) -> bool:
        """
        Check if the deadman switch is actively held and within timeout limit.
        """
        with self._lock:
            if self.estop_triggered:
                return False
            if not self.is_armed:
                return False
            age = time.monotonic() - self.last_heartbeat_time
            return age <= self.deadman_timeout

    def get_status_dict(self) -> dict[str, Any]:
        """Return a snapshot of deadman and robot status."""
        with self._lock:
            alive = (
                not self.estop_triggered
                and self.is_armed
                and (time.monotonic() - self.last_heartbeat_time <= self.deadman_timeout)
            )
            age = (
                (time.monotonic() - self.last_heartbeat_time)
                if self.last_heartbeat_time > 0
                else 999.0
            )
            estop = self.estop_triggered

        posture = "UNKNOWN"
        pitch = 0.0
        if self.posture_provider and callable(self.posture_provider):
            with contextlib.suppress(Exception):
                posture = str(self.posture_provider())
        if self.pitch_provider and callable(self.pitch_provider):
            with contextlib.suppress(Exception):
                pitch = float(self.pitch_provider())

        return {
            "is_alive": alive,
            "is_armed": self.is_armed,
            "estop_triggered": estop,
            "heartbeat_age": round(age, 2),
            "timeout_threshold": self.deadman_timeout,
            "posture": posture,
            "pitch": pitch,
        }
