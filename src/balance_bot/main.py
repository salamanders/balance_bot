import os
import sys
import traceback
import importlib.metadata
from dataclasses import asdict
from .diagnostics import run_diagnostics
from .wiring_check import WiringCheck
from .movement_check import MovementCheck
from .behavior.agent import Agent
from .utils import setup_logging, get_captured_logs
from .jules_client import JulesClient


def main() -> None:
    """Entry point for the robot control application."""
    # Ensure logging is set up early to capture imports/startup
    setup_logging()

    # Handle Mock Fallback Flag
    if "--allow-mocks" in sys.argv:
        os.environ["ALLOW_MOCK_FALLBACK"] = "1"

    try:
        if "--diagnose" in sys.argv:
            run_diagnostics()
            return

        if "--check-wiring" in sys.argv:
            try:
                WiringCheck().run()
            except KeyboardInterrupt:
                pass
            return

        if "--confirm-wiring" in sys.argv:
            try:
                MovementCheck().run()
            except KeyboardInterrupt:
                pass
            return

        bot = Agent()
        bot.init()
        bot.run()

    except Exception as e:
        # Check if Auto-Fix is requested
        if "--auto-fix" in sys.argv:
            print("\n!!! CRASH DETECTED !!!")
            print("Auto-Fix enabled. Gathering data for Jules...")

            # 1. Capture Traceback
            tb = traceback.format_exc()

            # 2. Capture Logs
            logs = get_captured_logs()

            # 3. Capture State
            state = {"status": "Crashed before Agent init or in Utility"}
            # Access 'bot' from locals if it was initialized
            if "bot" in locals():
                try:
                    # 'bot' is an Agent instance
                    agent_inst = locals()["bot"]
                    state = asdict(agent_inst.config)
                    # Add runtime info
                    state["runtime_ticks"] = agent_inst.ticks
                except Exception:
                    state["serialization_error"] = "Could not serialize bot config"

            # 4. Capture Libs
            libs = {
                dist.metadata["Name"]: dist.version
                for dist in importlib.metadata.distributions()
            }

            # 5. Report
            client = JulesClient()
            client.report_crash(str(e), tb, logs, state, libs)
            print("Crash report submitted to Jules. Check your dashboard for the new session.")

        # Always re-raise to exit with error
        raise

if __name__ == "__main__":
    main()
