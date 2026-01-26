1. This is for a self-balancing robot homebrew toy.
2. Always run a lint check before a commit to avoid unused variables and other annoying issues.
3. The hardware is bottom-tier: The motors may be uneven, the wheels are 3d printed, and the whole thing is held together with LEGOs. Excellent code is the only way it can be successful.
4. Keep the code concise.  Make it modern.  Use the latest python functions to make it more readable.

## Features

1. - [x] Implement a pyproject.toml with a make, lint, run. I think we should use "uv".
2. - [x] Don't assume I have a console open.  There will need to be other ways to communicate the setup with the user - like blinking the rpi zero light to indicate the setup phase.
3. - [x] It **will** fall over.  There should be an easy way to prop it back upright and have it resume.
4. - [x] It should **improve over time** as it gets more experience.
5. - [x] It should remember "last known good" PID variables and start from there.  Eventually we might be able to skip the calabration step?  Which also means a way to force a new calibration.
6. - [x] The battery levels will change over time, so that will be hard to account for.
7. - [x] Make a version of https://github.com/salamanders/mecanum/blob/main/AUTO_RUN.md that doesn't wait for WiFi.
8. - [ ] Make a version of https://github.com/salamanders/mecanum/blob/main/wiring_check.py that handles both wheels, and then the gyro directions.
