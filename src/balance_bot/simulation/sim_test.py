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

import os
import sys
import time

# Ensure src is in the python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../src")))

from balance_bot.simulation.sim_env import BalanceBotEnv


def main() -> None:
    print("Starting PyBullet Visualization...")
    # Instantiate environment with human rendering
    env = BalanceBotEnv(render_mode="human")

    # Reset the environment
    _, _ = env.reset()

    # Run a dummy control loop infinitely
    try:
        while True:
            # Send 0.0 to both motors
            action = [0.0, 0.0]

            # Step the environment
            obs, _, terminated, truncated, _ = env.step(action)

            # Pitch is the first observation
            pitch = obs[0]

            # Reset if robot crashes/terminates
            if terminated:
                print(f"Robot terminated at pitch: {pitch:.2f} rad. Resetting...")
                _, _ = env.reset()

            # Sleep slightly to match physical time
            time.sleep(1.0 / 100.0)

    except KeyboardInterrupt:
        print("Visualization interrupted by user.")
    finally:
        env.close()
        print("Environment closed.")


if __name__ == "__main__":
    main()
