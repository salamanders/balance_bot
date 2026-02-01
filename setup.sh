# Enable I2C interface if you haven't yet
sudo raspi-config nonint do_i2c 0


sudo apt-get update
sudo apt-get install python3-smbus i2c-tools libi2c-dev

# Install the MPU6050 library
pip3 install mpu6050-raspberrypi
