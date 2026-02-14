#!/usr/bin/env bash
# ============================================================
#  QMI Auto-Connect Service for SIM7600E
#
#  Creates a systemd service that:
#   - Starts QMI connection on boot
#   - Monitors connectivity and reconnects if dropped
#
#  Usage:
#    sudo ./setup_qmi_autoconnect.sh [APN]
# ============================================================

set -euo pipefail

APN="${1:-Econet}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

if [ "$EUID" -ne 0 ]; then
    echo "ERROR: Run with sudo"
    exit 1
fi

echo "Setting up QMI auto-connect service (APN: $APN)..."

# ---- Create the connection script ----
cat > /usr/local/bin/qmi-connect.sh << 'SCRIPT'
#!/usr/bin/env bash
# QMI connection manager for SIM7600E
# Called by systemd service — runs once to establish connection

APN="__APN__"
QMI_DEVICE=""
IFACE="wwan0"
MAX_RETRIES=5

log() { echo "$(date '+%Y-%m-%d %H:%M:%S') [qmi-connect] $*"; }

# Find QMI device
find_qmi() {
    for dev in /dev/cdc-wdm0 /dev/cdc-wdm1; do
        [ -e "$dev" ] && { QMI_DEVICE="$dev"; return 0; }
    done
    return 1
}

# Wait for QMI device to appear (USB may take time after boot)
log "Waiting for QMI device..."
for i in $(seq 1 30); do
    find_qmi && break
    sleep 2
done

if [ -z "$QMI_DEVICE" ]; then
    log "ERROR: No QMI device found after 60s. Is SIM7600E in QMI mode?"
    exit 1
fi
log "Found QMI device: $QMI_DEVICE"

# Kill PPP if running (can't share the modem)
poff -a 2>/dev/null || true
killall pppd 2>/dev/null || true

connect() {
    log "Connecting to $APN..."

    # Set raw_ip mode
    if [ -e "/sys/class/net/$IFACE/qmi/raw_ip" ]; then
        ip link set "$IFACE" down 2>/dev/null || true
        echo Y > "/sys/class/net/$IFACE/qmi/raw_ip"
    fi
    ip link set "$IFACE" up

    # Stop any stale connection
    qmicli -d "$QMI_DEVICE" --wds-stop-network=disable-autoconnect \
        --device-open-proxy 2>/dev/null || true
    sleep 1

    # Start connection
    if ! qmicli -d "$QMI_DEVICE" \
        --wds-start-network="apn=$APN,ip-type=4" \
        --client-no-release-cid \
        --device-open-proxy; then
        log "ERROR: wds-start-network failed"
        return 1
    fi

    sleep 2

    # Get IP via DHCP
    if command -v udhcpc &>/dev/null; then
        udhcpc -i "$IFACE" -q -n -t 10 2>/dev/null
    elif command -v dhclient &>/dev/null; then
        dhclient "$IFACE" 2>/dev/null
    fi

    sleep 2

    # Verify
    if ip addr show "$IFACE" 2>/dev/null | grep -q "inet "; then
        log "Connected! IP: $(ip -4 addr show "$IFACE" | grep inet | awk '{print $2}')"
        return 0
    else
        log "ERROR: No IP address assigned"
        return 1
    fi
}

# Try to connect with retries
for attempt in $(seq 1 $MAX_RETRIES); do
    if connect; then
        log "QMI connection established successfully"
        exit 0
    fi
    log "Attempt $attempt/$MAX_RETRIES failed, retrying in 10s..."
    sleep 10
done

log "ERROR: Failed to connect after $MAX_RETRIES attempts"
exit 1
SCRIPT

# Substitute actual APN
sed -i "s|__APN__|$APN|g" /usr/local/bin/qmi-connect.sh
chmod +x /usr/local/bin/qmi-connect.sh

# ---- Create the watchdog script ----
cat > /usr/local/bin/qmi-watchdog.sh << 'WATCHDOG'
#!/usr/bin/env bash
# QMI connection watchdog — checks connectivity every 60s, reconnects if down

IFACE="wwan0"
PING_TARGET="8.8.8.8"

log() { echo "$(date '+%Y-%m-%d %H:%M:%S') [qmi-watchdog] $*"; }

log "Watchdog started, monitoring $IFACE"

while true; do
    sleep 60

    # Check if interface has an IP
    if ! ip addr show "$IFACE" 2>/dev/null | grep -q "inet "; then
        log "No IP on $IFACE — reconnecting..."
        /usr/local/bin/qmi-connect.sh
        continue
    fi

    # Ping test
    if ! ping -I "$IFACE" -c 1 -W 5 "$PING_TARGET" &>/dev/null; then
        log "Ping failed — reconnecting..."
        /usr/local/bin/qmi-connect.sh
    fi
done
WATCHDOG

chmod +x /usr/local/bin/qmi-watchdog.sh

# ---- Create systemd service ----
cat > /etc/systemd/system/qmi-network.service << 'SERVICE'
[Unit]
Description=QMI Network Connection (SIM7600E)
After=network-pre.target
Wants=network-pre.target
Before=network.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/local/bin/qmi-connect.sh
ExecStop=/usr/bin/qmicli -d /dev/cdc-wdm0 --wds-stop-network=disable-autoconnect --device-open-proxy
Restart=on-failure
RestartSec=15

[Install]
WantedBy=multi-user.target
SERVICE

# ---- Create watchdog service ----
cat > /etc/systemd/system/qmi-watchdog.service << 'WATCHDOG_SVC'
[Unit]
Description=QMI Connection Watchdog
After=qmi-network.service
Requires=qmi-network.service

[Service]
Type=simple
ExecStart=/usr/local/bin/qmi-watchdog.sh
Restart=always
RestartSec=30

[Install]
WantedBy=multi-user.target
WATCHDOG_SVC

# ---- Disable old PPP service if exists ----
systemctl disable ppp@sim7600e 2>/dev/null || true
systemctl stop ppp@sim7600e 2>/dev/null || true

# ---- Enable and start ----
systemctl daemon-reload
systemctl enable qmi-network.service
systemctl enable qmi-watchdog.service

echo ""
echo "==========================================="
echo " QMI Auto-Connect Service Installed"
echo "==========================================="
echo ""
echo "Services created:"
echo "  qmi-network.service  — connects on boot"
echo "  qmi-watchdog.service — reconnects if dropped (checks every 60s)"
echo ""
echo "Commands:"
echo "  sudo systemctl start qmi-network    — connect now"
echo "  sudo systemctl stop qmi-network     — disconnect"
echo "  sudo systemctl status qmi-network   — check status"
echo "  journalctl -u qmi-network -f        — view logs"
echo "  journalctl -u qmi-watchdog -f       — view watchdog logs"
echo ""
echo "To start immediately:"
echo "  sudo systemctl start qmi-network && sudo systemctl start qmi-watchdog"
echo ""
