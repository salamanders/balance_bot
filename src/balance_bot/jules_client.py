import os
import json
import logging
import urllib.request
import urllib.error
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)

JULES_API_BASE = "https://jules.googleapis.com/v1alpha"
SOURCE_NAME = "sources/github/salamanders/balance_bot"

class JulesClient:
    """
    Client for the Jules API.
    Used to auto-report crashes and request fixes.
    """

    def __init__(self, api_key: Optional[str] = None):
        self.api_key = api_key or os.environ.get("JULES_API_KEY")
        if not self.api_key:
            logger.warning("JulesClient initialized without API Key. Auto-fix will fail.")

    def _make_request(self, method: str, endpoint: str, data: Optional[Dict] = None) -> Dict:
        if not self.api_key:
            raise ValueError("JULES_API_KEY not set.")

        url = f"{JULES_API_BASE}/{endpoint}"
        headers = {
            "Content-Type": "application/json",
            "X-Goog-Api-Key": self.api_key,
        }

        body = json.dumps(data).encode("utf-8") if data else None

        req = urllib.request.Request(url, data=body, headers=headers, method=method)

        try:
            with urllib.request.urlopen(req) as response:
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

    def get_sources(self) -> list:
        """List available sources (verification step)."""
        resp = self._make_request("GET", "sources")
        return resp.get("sources", [])

    def create_session(self, prompt: str) -> Dict:
        """Create a new Jules session for the hardcoded source."""
        payload = {
            "prompt": prompt,
            "sourceContext": {
                "source": SOURCE_NAME,
                "githubRepoContext": {
                    "startingBranch": "main"  # Assumption: fix on main or let Jules decide
                }
            },
            "automationMode": "AUTO_CREATE_PR", # As per desire to "start working on a fix PR"
            "title": "Crash Auto-Fix"
        }
        return self._make_request("POST", "sessions", payload)

    def report_crash(
        self,
        error_msg: str,
        stack_trace: str,
        logs: str,
        state: Dict[str, Any],
        libs: Dict[str, str]
    ) -> None:
        """
        Constructs the prompt and initiates the fix session.
        """
        if not self.api_key:
            logger.error("Cannot report crash: JULES_API_KEY not found.")
            return

        # Construct a detailed prompt
        prompt = (
            f"The application crashed with the following error:\n\n"
            f"{error_msg}\n\n"
            f"Stack Trace:\n```\n{stack_trace}\n```\n\n"
            f"Recent Logs:\n```\n{logs}\n```\n\n"
            f"Current Runtime State:\n```json\n{json.dumps(state, indent=2, default=str)}\n```\n\n"
            f"Installed Libraries:\n```json\n{json.dumps(libs, indent=2)}\n```\n\n"
            f"Please analyze this crash, identify the root cause, and create a Pull Request "
            f"with a fix."
        )

        try:
            logger.info("Contacting Jules API to report crash...")
            session = self.create_session(prompt)
            session_id = session.get("name", "unknown")
            logger.info(f"Successfully started Jules Session: {session_id}")
            logger.info("Jules is now working on a fix.")
        except Exception as e:
            logger.error(f"Failed to trigger Jules Auto-Fix: {e}")
