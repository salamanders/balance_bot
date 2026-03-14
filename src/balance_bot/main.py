import os
import argparse
import traceback
import importlib.metadata
import datetime
from .behavior.agent import Agent
from .utils import setup_logging, get_captured_logs
from .jules_client import JulesClient
from .watchdog import SurvivalWatchdog

def parse_args() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description="Balance Bot Control")
    parser.add_argument("--reset-brain", action="store_true", help="Wipe all learned configuration")
    parser.add_argument("--allow-mocks", action="store_true", help="Allow fallback to mock hardware")
    parser.add_argument("--auto-fix", action="store_true", help="Report crashes to Jules")
    return parser.parse_args()

def _reset_robot_memory() -> None:
    """Wipe all learned configuration."""
    print("Resetting Robot Memory...")
    from .configuration import HardwareConfig, LearningState
    HardwareConfig().save()
    LearningState().save()
    print("Brain reset complete. Entering Toddler Phase.")

def _run_discovery(watchdog: SurvivalWatchdog) -> None:
    """Run the self-discovery pipeline (Toddler Phase)."""
    print("Incomplete knowledge detected. Initiating Discovery...")
    from .discovery.pipeline import SelfDiscoveryPipeline
    from .discovery import (
        DiscoverBusesStep, HardwareInitStep, ManualLeanCalibrationStep, BrokenWireCheckStep, FrictionThresholdStep,
        DeriveKinematicsStep, MotorTrimStep, MechanicalBacklashStep, KickupDynamicsStep
    )

    steps = [
        DiscoverBusesStep(), HardwareInitStep(), ManualLeanCalibrationStep(), BrokenWireCheckStep(), FrictionThresholdStep(),
        DeriveKinematicsStep(), MotorTrimStep(), MechanicalBacklashStep(), KickupDynamicsStep()
    ]
    pipeline = SelfDiscoveryPipeline(steps, watchdog)
    pipeline.run()
    # SelfDiscoveryPipeline finishes and cleans up the hardware locks safely.
    print("Discovery complete. Waking up Main Agent...")

def _handle_crash_reporting(e: Exception, bot_instance: Agent | None = None) -> None:
    """Gather state and report the crash to Jules."""
    print("\n!!! CRASH DETECTED !!!")
    print("Auto-Fix enabled. Gathering data for Jules...")

    # 1. Capture Traceback
    tb = traceback.format_exc()

    # 2. Capture Logs
    logs = get_captured_logs()

    # 3. Capture State
    state_info = {"status": "Crashed before Agent init or in Utility"}
    if bot_instance:
        try:
            state_info = {
                "hardware": bot_instance.hw_config.model_dump(),
                "learning": bot_instance.learning_state.model_dump(),
                "runtime_ticks": bot_instance.ticks
            }
        except Exception:
            state_info["serialization_error"] = "Could not serialize bot config"

    # 4. Capture Libs
    libs = {
        dist.metadata["Name"]: dist.version
        for dist in importlib.metadata.distributions()
    }

    # 5. Capture Telemetry
    telemetry_data = "No telemetry data found."
    try:
        if os.path.exists("flight_data.csv"):
            with open("flight_data.csv", "r") as f:
                # Grab the header and the last 100 lines
                lines = f.readlines()
                if len(lines) > 100:
                    telemetry_data = "".join([lines[0]] + lines[-100:])
                else:
                    telemetry_data = "".join(lines)
    except Exception as read_err:
        telemetry_data = f"Failed to read telemetry: {read_err}"

    # 6. Report
    client = JulesClient()
    success, prompt = client.report_crash(str(e), tb, logs, state_info, libs, telemetry_data)

    if success:
        print("Crash report submitted to Jules. Check your dashboard for the new session.")
    else:
        try:
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"exception_report_{ts}.md"
            with open(filename, "w") as f:
                f.write(prompt)
            print(f"Crash report saved locally to {filename} (Jules upload failed).")
        except Exception as write_err:
            print(f"Failed to write local crash report: {write_err}")

def main() -> None:
    """Entry point for the robot control application."""
    args = parse_args()

    # Ensure logging is set up early to capture imports/startup
    # Redirect stdout and stderr to the logger if --auto-fix is enabled
    setup_logging(capture_stdout=args.auto_fix)

    # Handle Mock Fallback Flag
    if args.allow_mocks:
        os.environ["ALLOW_MOCK_FALLBACK"] = "1"

    # Spawn the survival instinct with a 20-second frustration limit
    watchdog = SurvivalWatchdog(timeout=20.0)
    bot = None

    try:
        # 1. Wipe Brain if requested
        if args.reset_brain:
            _reset_robot_memory()

        # 2. Check if we need to learn
        from .configuration import LearningState
        state = LearningState.load()

        # If the final discovery step hasn't been verified, it's a baby.
        # We also check if kickup power is 0.0, because SelfDiscoveryPipeline includes KickUp after Backlash.
        needs_discovery = (not state.backlash_verified) or (state.control.kickup_power_forward == 0.0)

        # 3. Learn (Toddler Phase)
        if needs_discovery:
            _run_discovery(watchdog)

        # 4. Run (Adult Phase)
        bot = Agent(watchdog=watchdog)
        bot.run()

    except Exception as e:
        # Check if Auto-Fix is requested
        if args.auto_fix:
            _handle_crash_reporting(e, bot)

        # Always re-raise to exit with error
        raise
    finally:
        watchdog.stop()

if __name__ == "__main__":
    main()
