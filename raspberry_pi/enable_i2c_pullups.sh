#!/bin/bash
# Enable internal pull-ups on I2C pins (GPIO 2 and 3)
# This is a temporary workaround - external pull-ups recommended

echo "Enabling internal pull-ups on I2C pins..."

# GPIO 2 (SDA) - pull-up
sudo raspi-gpio set 2 pu

# GPIO 3 (SCL) - pull-up  
sudo raspi-gpio set 3 pu

echo "Done! Internal pull-ups enabled."
echo "Note: These are weak (~50kΩ) - external 4.7kΩ resistors recommended for reliable communication"
