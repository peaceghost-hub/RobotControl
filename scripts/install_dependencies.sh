#!/bin/bash
# Installation script for Robot Control System

echo "=========================================="
echo " Robot Control System - Install Script   "
echo "=========================================="
echo ""

# Check if running on Raspberry Pi
if [ -f /proc/device-tree/model ]; then
    MODEL=$(cat /proc/device-tree/model)
    echo "Detected: $MODEL"
else
    echo "Warning: Not running on Raspberry Pi"
fi

echo ""
echo "Installing system dependencies..."
sudo apt-get update
sudo apt-get install -y python3-pip python3-dev git
sudo apt-get install -y libatlas-base-dev libjasper-dev
sudo apt-get install -y libqtgui4 libqt4-test
sudo apt-get install -y i2c-tools

echo ""
echo "Installing Python dependencies..."
cd raspberry_pi
pip3 install -r requirements.txt

echo ""
echo "Enabling required interfaces..."
sudo raspi-config nonint do_camera 0
sudo raspi-config nonint do_spi 0
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_serial 2  # Disable login shell, enable hardware

echo ""
echo "Creating configuration file..."
if [ ! -f config.json ]; then
    cp config.json.example config.json
    echo "config.json created. Please edit with your settings."
else
    echo "config.json already exists."
fi

echo ""
echo "Installation complete!"
echo ""
echo "Next steps:"
echo "1. Edit raspberry_pi/config.json with your settings"
echo "2. Run: python3 raspberry_pi/main.py"
echo ""
