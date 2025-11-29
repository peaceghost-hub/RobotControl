#!/bin/bash
# Start the robot control system on Raspberry Pi

echo "Starting Robot Control System..."

cd "$(dirname "$0")/../raspberry_pi"

# Check if config exists
if [ ! -f "config.json" ]; then
    echo "Error: config.json not found!"
    echo "Please copy config.json.example to config.json and configure it."
    exit 1
fi

# Check if dependencies are installed
if ! python3 -c "import Adafruit_DHT" 2>/dev/null; then
    echo "Error: Dependencies not installed!"
    echo "Please run: scripts/install_dependencies.sh"
    exit 1
fi

# Start main control loop
echo "Starting robot control system..."
python3 main.py
