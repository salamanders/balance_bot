import os
import argparse
import traceback
import importlib.metadata
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
        if args.reset_brain or args.force:
            from .config import RobotConfig
            print("Resetting Robot Memory...")
            cfg = RobotConfig.load()
            cfg.reset_hardware_map()
            cfg.save()
            print("Brain reset complete.")
            # If we are not discovering or checking wiring, we are done.
            if not args.discover and not args.check_wiring:
                return

        if args.discover or args.check_wiring:
            try:
                # WiringCheck handles incremental discovery automatically.
                # If reset_hardware_map was called, it starts from scratch.
                WiringCheck().run()
            except KeyboardInterrupt:
                pass
            return

        try:
            bot = Agent()
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
                    state = agent_inst.config.model_dump()
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
            success, prompt = client.report_crash(str(e), tb, logs, state, libs)

            if success:
                print("Crash report submitted to Jules. Check your dashboard for the new session.")
            else:
                try:
                    import datetime
                    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                    filename = f"exception_report_{ts}.md"
                    with open(filename, "w") as f:
                        f.write(prompt)
                    print(f"Crash report saved locally to {filename} (Jules upload failed).")
                except Exception as write_err:
                     print(f"Failed to write local crash report: {write_err}")

        # Always re-raise to exit with error
        raise

if __name__ == "__main__":
    main()
