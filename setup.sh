# Enable I2C interface if you haven't yet
sudo raspi-config nonint do_i2c 0

sudo apt-get update
sudo apt-get install i2c-tools libi2c-dev

# Install dependencies with uv
uv sync
