import os
import argparse
import traceback
import importlib.metadata
from .wiring_check import WiringCheck
from .behavior.agent import Agent
from .utils import setup_logging, get_captured_logs
from .jules_client import JulesClient
from .watchdog import SurvivalWatchdog


def main() -> None:
    """Entry point for the robot control application."""
    # Ensure logging is set up early to capture imports/startup
    setup_logging()

    parser = argparse.ArgumentParser(description="Balance Bot Control")
    parser.add_argument("--reset-brain", action="store_true", help="Wipe all learned configuration")
    parser.add_argument("--allow-mocks", action="store_true", help="Allow fallback to mock hardware")
    parser.add_argument("--auto-fix", action="store_true", help="Report crashes to Jules")

    args = parser.parse_args()

    # Handle Mock Fallback Flag
    if args.allow_mocks:
        os.environ["ALLOW_MOCK_FALLBACK"] = "1"

    # Spawn the survival instinct with a 20-second frustration limit
    watchdog = SurvivalWatchdog(timeout=20.0)

    try:
        # 1. Wipe Brain if requested
        if args.reset_brain:
            print("Resetting Robot Memory...")
            from .configuration import HardwareConfig, LearningState
            HardwareConfig().save()
            LearningState().save()
            print("Brain reset complete. Entering Toddler Phase.")

        # 2. Check if we need to learn
        from .configuration import LearningState
        state = LearningState.load()

        # If the final discovery step hasn't been verified, it's a baby.
        # We also check if kickup power is 0.0, because WiringCheck includes KickUp after Backlash.
        needs_discovery = (not state.backlash_verified) or (state.control.kickup_power_forward == 0.0)

        # 3. Learn (Toddler Phase)
        if needs_discovery:
            print("Incomplete knowledge detected. Initiating Discovery...")
            WiringCheck(watchdog=watchdog).run()
            # WiringCheck finishes and cleans up the hardware locks safely.
            print("Discovery complete. Waking up Main Agent...")

        # 4. Run (Adult Phase)
        bot = Agent(watchdog=watchdog)
        bot.run()

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
            state_info = {"status": "Crashed before Agent init or in Utility"}
            # Access 'bot' from locals if it was initialized
            if "bot" in locals():
                try:
                    # 'bot' is an Agent instance
                    agent_inst = locals()["bot"]
                    state_info = {
                        "hardware": agent_inst.hw_config.model_dump(),
                        "learning": agent_inst.learning_state.model_dump(),
                        "runtime_ticks": agent_inst.ticks
                    }
                except Exception:
                    state_info["serialization_error"] = "Could not serialize bot config"

            # 4. Capture Libs
            libs = {
                dist.metadata["Name"]: dist.version
                for dist in importlib.metadata.distributions()
            }

            # 5. Report
            client = JulesClient()
            success, prompt = client.report_crash(str(e), tb, logs, state_info, libs)

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
    finally:
        watchdog.stop()

if __name__ == "__main__":
    main()
