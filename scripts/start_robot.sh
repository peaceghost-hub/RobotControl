#!/bin/bash
# Start the robot control system on Raspberry Pi

echo "Starting Robot Control System..."

cd "$(dirname "$0")/../raspberry_pi"

# Prefer a project venv if present (recommended on the Pi)
PYTHON_BIN="python3"
if [ -x "../.venv/bin/python" ]; then
    PYTHON_BIN="../.venv/bin/python"
fi

# Check if config exists
if [ ! -f "config.json" ]; then
    echo "Error: config.json not found!"
    echo "Please copy config.json.example to config.json and configure it."
    exit 1
fi

# Check optional dependencies (warn but don't block boot)
if ! python3 -c "import Adafruit_DHT" 2>/dev/null; then
    echo "WARNING: Adafruit_DHT not installed — DHT sensor will be unavailable"
    echo "  To install: scripts/install_dependencies.sh"
fi

# ── Wait for network connectivity (QMI / WiFi / Ethernet) ────
# The systemd service already has After=qmi-network.service, but
# the modem may still be negotiating an IP.  Wait up to 60s for
# *any* route to the dashboard or internet.
DASHBOARD_HOST=$(python3 -c "
import json, urllib.parse, sys
try:
    cfg = json.load(open('config.json'))
    url = cfg.get('dashboard_api',{}).get('base_url','')
    host = urllib.parse.urlparse(url).hostname or '8.8.8.8'
    print(host)
except: print('8.8.8.8')
" 2>/dev/null)

echo "Waiting for network (target: ${DASHBOARD_HOST})..."
for i in $(seq 1 30); do
    if ping -c 1 -W 2 "$DASHBOARD_HOST" &>/dev/null; then
        echo "  ✓ Network reachable after $((i*2))s"
        break
    fi
    sleep 2
done
# Proceed even if network isn't ready — the QMI watchdog and Python
# health monitor will keep retrying in the background.

# Start main control loop
echo "Starting robot control system..."
"$PYTHON_BIN" main.py
