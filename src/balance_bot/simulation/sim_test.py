import time
import os
import sys

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
