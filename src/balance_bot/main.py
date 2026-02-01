import sys
from .diagnostics import run_diagnostics
from .wiring_check import WiringCheck
from .behavior.agent import Agent

def main() -> None:
    """Entry point for the robot control application."""
    if "--diagnose" in sys.argv:
        run_diagnostics()
        return

    if "--check-wiring" in sys.argv:
        try:
            WiringCheck().run()
        except KeyboardInterrupt:
            pass
        return

    bot = Agent()
    bot.init()
    bot.run()

if __name__ == "__main__":
    main()
