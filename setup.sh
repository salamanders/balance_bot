# Enable I2C interface if you haven't yet
sudo raspi-config nonint do_i2c 0


sudo apt-get update
sudo apt-get install i2c-tools libi2c-dev
# Get the Picon Zero library
wget http://4tronix.co.uk/piconz.sh -O piconz.sh
bash piconz.sh

# Install dependencies with uv
uv sync
