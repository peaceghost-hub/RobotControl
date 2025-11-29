#!/bin/bash
# Raspberry Pi Setup Script for Robot Controller
# Run this script on your Raspberry Pi to set up the robot controller

echo "Setting up Raspberry Pi Robot Controller..."

# Update system
echo "Updating system packages..."
sudo apt update && sudo apt upgrade -y

# Install required system packages
echo "Installing system dependencies..."
sudo apt install -y python3-pip python3-dev git

# Install Python packages
echo "Installing Python packages..."
pip3 install --user requests psutil

# For camera support (optional)
echo "Installing camera libraries..."
sudo apt install -y python3-picamera python3-opencv

# For sensor support (optional)
echo "Installing sensor libraries..."
pip3 install --user Adafruit-DHT Adafruit-ADS1x15 RPi.GPIO pyserial

# Create robot controller service (optional)
echo "Setting up robot controller as a service..."
sudo tee /etc/systemd/system/robot-controller.service > /dev/null <<EOF
[Unit]
Description=Raspberry Pi Robot Controller
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/robot_control
ExecStart=/usr/bin/python3 /home/pi/robot_control/robot_controller.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload

echo "Setup complete!"
echo ""
echo "Next steps:"
echo "1. Copy robot_controller.py to your Raspberry Pi"
echo "2. Update DASHBOARD_IP in the script to match your dashboard's IP"
echo "3. Run: python3 robot_controller.py"
echo "4. Or enable service: sudo systemctl enable robot-controller && sudo systemctl start robot-controller"