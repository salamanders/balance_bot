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

import json
import logging
import os
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any

logger = logging.getLogger(__name__)


@dataclass
class CrashReport:
    """Data object encapsulating all information needed for a crash report."""
    error_msg: str
    stack_trace: str
    logs: str
    state: dict[str, Any]
    libs: dict[str, str]
    telemetry: str = "No telemetry data found."


JULES_API_BASE = "https://jules.googleapis.com/v1alpha"
SOURCE_NAME = "sources/github/salamanders/balance_bot"
DEFAULT_TIMEOUT = 30.0


class JulesClient:
    """
    Client for the Jules API.
    Used to auto-report crashes and request fixes.
    """

    def __init__(self, api_key: str | None = None) -> None:
        self._api_key = api_key or os.environ.get("JULES_API_KEY")
        if not self._api_key:
            logger.warning("JulesClient initialized without API Key. Auto-fix will fail.")

    def __repr__(self) -> str:
        key_status = "***" if self._api_key else "None"
        return f"<{self.__class__.__name__} api_key={key_status}>"

    def _make_request(self, method: str, endpoint: str, data: dict[str, Any] | None = None) -> dict[str, Any]:
        if not self._api_key:
            raise ValueError("JULES_API_KEY not set.")

        url = f"{JULES_API_BASE}/{endpoint}"
        headers = {
            "Content-Type": "application/json",
            "X-Goog-Api-Key": self._api_key,
        }

        body = json.dumps(data).encode("utf-8") if data else None

        req = urllib.request.Request(url, data=body, headers=headers, method=method)

        try:
            with urllib.request.urlopen(req, timeout=DEFAULT_TIMEOUT) as response:
                if response.status >= 300:
                    raise RuntimeError(f"Jules API Error: {response.status} {response.reason}")
                return json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as e:
            error_body = e.read().decode("utf-8")
            logger.error(f"Jules API HTTP Error: {e.code} - {error_body}")
            raise
        except Exception as e:
            logger.error(f"Jules API Request Failed: {e}")
            raise

    def get_sources(self) -> list[dict[str, Any]]:
        """List available sources (verification step)."""
        resp = self._make_request("GET", "sources")
        return resp.get("sources", [])

    def create_session(self, prompt: str) -> dict[str, Any]:
        """Create a new Jules session for the hardcoded source."""
        payload = {
            "prompt": prompt,
            "sourceContext": {
                "source": SOURCE_NAME,
                "githubRepoContext": {
                    "startingBranch": "main"  # Assumption: fix on main or let Jules decide
                }
            },
            "automationMode": "AUTO_CREATE_PR",  # As per desire to "start working on a fix PR"
            "title": "Crash Auto-Fix"
        }
        return self._make_request("POST", "sessions", payload)

    def report_crash(
            self,
            report: CrashReport
    ) -> tuple[bool, str]:
        """
        Constructs the prompt and initiates the fix session.
        Returns (success, prompt).
        """
        # Construct a detailed prompt
        prompt = (
            f"The application crashed with the following error:\n\n"
            f"{report.error_msg}\n\n"
            f"Stack Trace:\n```\n{report.stack_trace}\n```\n\n"
            f"Recent Logs:\n```\n{report.logs}\n```\n\n"
            f"Recent Telemetry:\n```csv\n{report.telemetry}\n```\n\n"
            f"Current Runtime State:\n```json\n{json.dumps(report.state, indent=2, default=str)}\n```\n\n"
            f"Installed Libraries:\n```json\n{json.dumps(report.libs, indent=2)}\n```\n\n"
            f"Please analyze this crash, identify the root cause, and create a Pull Request with a fix."
        )

        if not self._api_key:
            logger.error("Cannot report crash: JULES_API_KEY not found.")
            return False, prompt

        try:
            logger.info("Contacting Jules API to report crash...")
            session = self.create_session(prompt)
            session_id = session.get("name", "unknown")
            logger.info(f"Successfully started Jules Session: {session_id}")
            logger.info("Jules is now working on a fix.")
            return True, prompt
        except Exception as e:
            logger.error(f"Failed to trigger Jules Auto-Fix: {e}")
            return False, prompt
