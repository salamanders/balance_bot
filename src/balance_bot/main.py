import os
import sys
import argparse
import traceback
import importlib.metadata
from dataclasses import asdict
from .wiring_check import WiringCheck
from .behavior.agent import Agent
from .utils import setup_logging, get_captured_logs
from .jules_client import JulesClient


def main() -> None:
    """Entry point for the robot control application."""
    # Ensure logging is set up early to capture imports/startup
    setup_logging()

    parser = argparse.ArgumentParser(description="Balance Bot Control")
    parser.add_argument("--discover", action="store_true", help="Run the toddler discovery process")
    parser.add_argument("--force", action="store_true", help="Force relearning even if knowledge exists")
    parser.add_argument("--reset-brain", action="store_true", help="Reset the knowledge graph")
    parser.add_argument("--check-wiring", action="store_true", help="Run wiring check utility")
    parser.add_argument("--allow-mocks", action="store_true", help="Allow fallback to mock hardware")
    parser.add_argument("--auto-fix", action="store_true", help="Report crashes to Jules")

    args = parser.parse_args()

    # Handle Mock Fallback Flag
    if args.allow_mocks:
        os.environ["ALLOW_MOCK_FALLBACK"] = "1"

    try:
        if args.reset_brain:
            from .discovery.knowledge_graph import DiscoveryContext
            DiscoveryContext().forget_all()
            if not args.discover:
                return

        if args.discover:
            from .discovery.knowledge_graph import DiscoveryContext
            from .discovery.baby_brain import DiscoveryBrain
            from .discovery.types import Atom

            try:
                ctx = DiscoveryContext()
                # Check if we already graduated
                if ctx.has_atom(Atom.TRIM_CALIBRATION) and not args.force:
                     print("I already know how to walk. Use --force to relearn.")
                     return

                brain = DiscoveryBrain(ctx)
                brain.think()
            except KeyboardInterrupt:
                pass
            return

        if args.check_wiring:
            try:
                WiringCheck().run()
            except KeyboardInterrupt:
                pass
            return

        try:
            bot = Agent()
            bot.init()
            bot.run()
        finally:
            # Emergency Stop / Cleanup if bot was initialized
            # This catches crashes that happen before bot.run() (e.g. init)
            # or if bot.run()'s internal cleanup failed.
            if "bot" in locals():
                try:
                    agent_inst = locals()["bot"]
                    if hasattr(agent_inst, "core"):
                        agent_inst.core.cleanup()
                except Exception:
                    # Do not let cleanup errors mask the original crash
                    pass

    except Exception as e:
        # Check if Auto-Fix is requested
        if args.auto_fix:
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
