The "modern simple way" (and the most robust) is to create a **Systemd Service**.
It creates one text file that tells the Pi: *"Run this command, and restart it if it crashes."*

### Step 1: Get your details

You need to know exactly where your code lives.

1. SSH into your Pi.
2. Navigate to your robot folder: `cd balance-bot` (or whatever you named it).
3. Run `make install` to ensure all dependencies are installed.
4. Type `pwd` and copy the output. (e.g., `/home/pi/balance-bot`)
5. Type `whoami` to confirm your username. (e.g., `pi` or `admin`)
6. Type `which uv` to get the path to uv. (e.g. `/home/pi/.local/bin/uv`)

### Step 2: Create the Service File

Run this command to create the file:

```bash
sudo nano /etc/systemd/system/balance-bot.service
```

Paste this text into the file. **Important:** Replace `/home/pi/balance-bot` with the path you copied in Step 1, and `pi` with your username if it's different.

```ini
[Unit]
Description=Balance Bot Controller
# We do NOT wait for network, so it starts immediately on boot.

[Service]
# Update to your username from "whoami"
User=pi
# Replace /home/pi/balance-bot with the path you copied in Step 1 (where you cloned the github repo)
WorkingDirectory=/home/pi/balance-bot
# We use uv to run the app. Replace /home/pi/.local/bin/uv with the result of `which uv` from Step 1.
# Note: "balance-bot" is the command defined in pyproject.toml [project.scripts]
ExecStart=/home/pi/.local/bin/uv run balance-bot

# Auto-restart if it crashes
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

### Step 3: Enable it

Save the file (Ctrl+O, Enter) and exit (Ctrl+X). Then run these two commands:

```bash
# 1. Tell Linux to scan for the new file
sudo systemctl daemon-reload

# 2. Enable it so it runs on every boot
sudo systemctl enable balance-bot.service
```

### Step 4: Test it immediately

You don't have to reboot to test it. Just start it now:

```bash
sudo systemctl start balance-bot.service
```

Check if it is running:

```bash
sudo systemctl status balance-bot.service
```

* **Green Light:** If you see "Active: active (running)", it's working!
* **Red Light:** If it failed, read the error message at the bottom of that status screen. It usually means a typo in the path.

### How to update your code later

Since the service runs in the background, you can't just edit the file and run it again manually (you'll get "Address already in use" errors if you try to run another instance).

When you change your code:

1. Edit your files.
2. Run `sudo systemctl restart balance-bot` to load the changes.
