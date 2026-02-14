#!/usr/bin/env bash
# ============================================================
#  QMI Network Setup for SIM7600E on Raspberry Pi (USB)
#
#  Replaces PPP with QMI — gives you a real network interface
#  (wwan0) at full LTE speed instead of a slow serial tunnel.
#
#  Usage:
#    chmod +x setup_qmi_sim7600e.sh
#    sudo ./setup_qmi_sim7600e.sh [APN]
#
#  Example:
#    sudo ./setup_qmi_sim7600e.sh Econet
# ============================================================

set -euo pipefail

APN="${1:-Econet}"
QMI_DEVICE=""

echo "==========================================="
echo " SIM7600E QMI Setup"
echo "==========================================="
echo ""

# ---- Must run as root ----
if [ "$EUID" -ne 0 ]; then
    echo "ERROR: Run this script with sudo"
    exit 1
fi

# ---- Step 1: Install required packages ----
echo "[1/7] Installing QMI tools..."
apt-get update -qq
apt-get install -y libqmi-utils udhcpc

echo ""

# ---- Step 2: Find the QMI device ----
echo "[2/7] Searching for QMI device..."

# The SIM7600E exposes /dev/cdc-wdm0 when in QMI mode
if [ -e /dev/cdc-wdm0 ]; then
    QMI_DEVICE="/dev/cdc-wdm0"
    echo "  Found: $QMI_DEVICE"
elif [ -e /dev/cdc-wdm1 ]; then
    QMI_DEVICE="/dev/cdc-wdm1"
    echo "  Found: $QMI_DEVICE"
else
    echo ""
    echo "  No QMI device found (/dev/cdc-wdm*)."
    echo ""
    echo "  Your SIM7600E may not be in QMI mode yet."
    echo "  Checking USB devices..."
    echo ""
    lsusb | grep -i -E "sim|qualcomm|1e0e|2c7c" || echo "  (no SIMCom USB devices found)"
    echo ""
    echo "  Checking for ttyUSB ports..."
    ls -la /dev/ttyUSB* 2>/dev/null || echo "  (no ttyUSB devices found)"
    echo ""
    echo "  Trying to switch modem to QMI mode via AT command..."

    # Find a ttyUSB port to send AT commands
    AT_PORT=""
    for port in /dev/ttyUSB2 /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyUSB3; do
        if [ -e "$port" ]; then
            AT_PORT="$port"
            break
        fi
    done

    if [ -z "$AT_PORT" ]; then
        echo "  ERROR: No ttyUSB port found. Is the SIM7600E plugged in via USB?"
        exit 1
    fi

    echo "  Using $AT_PORT to send AT commands..."
    echo ""

    # Configure serial port
    stty -F "$AT_PORT" 115200 raw -echo -echoe -echok -echoctl -echoke

    # Check current USB mode
    echo -e "AT+CUSBPIDSWITCH?\r" > "$AT_PORT"
    sleep 1
    response=$(timeout 3 cat "$AT_PORT" 2>/dev/null || true)
    echo "  Current USB mode: $response"

    # Switch to QMI mode (PID 9011 = RNDIS+AT+DIAG+QMI)
    echo "  Switching to QMI-enabled USB mode..."
    echo -e "AT+CUSBPIDSWITCH=9011,1,1\r" > "$AT_PORT"
    sleep 2

    echo ""
    echo "  =========================================="
    echo "  USB mode switch command sent."
    echo "  The modem will REBOOT in ~5 seconds."
    echo "  Wait 15 seconds, then run this script again."
    echo "  =========================================="
    exit 0
fi

echo ""

# ---- Step 3: Stop PPP if running ----
echo "[3/7] Stopping PPP if active..."
poff -a 2>/dev/null || true
killall pppd 2>/dev/null || true
sleep 1

# Remove ppp0 default route if it exists
ip route del default dev ppp0 2>/dev/null || true
echo "  Done"
echo ""

# ---- Step 4: Check SIM status ----
echo "[4/7] Checking SIM and modem status..."

# Verify device is accessible
if ! qmicli -d "$QMI_DEVICE" --dms-get-manufacturer --device-open-proxy 2>/dev/null | head -3; then
    echo "  WARNING: Could not query modem. Trying with --device-open-sync..."
    qmicli -d "$QMI_DEVICE" --dms-get-manufacturer --device-open-sync 2>/dev/null | head -3 || true
fi

# Check SIM
echo ""
echo "  SIM status:"
qmicli -d "$QMI_DEVICE" --uim-get-card-status --device-open-proxy 2>/dev/null | grep -E "Card state|PIN" | head -5 || echo "  (could not read SIM status)"

# Check signal
echo ""
echo "  Signal strength:"
qmicli -d "$QMI_DEVICE" --nas-get-signal-strength --device-open-proxy 2>/dev/null | grep -E "Network|RSSI|strength" | head -5 || echo "  (could not read signal)"

echo ""

# ---- Step 5: Bring up the wwan0 interface ----
echo "[5/7] Configuring wwan0 interface..."

# Set interface to raw-ip mode (required for QMI)
IFACE="wwan0"
if [ -e "/sys/class/net/$IFACE/qmi/raw_ip" ]; then
    # Must bring down interface before switching mode
    ip link set "$IFACE" down 2>/dev/null || true
    echo Y > "/sys/class/net/$IFACE/qmi/raw_ip"
    echo "  Set raw_ip mode"
fi

ip link set "$IFACE" up
echo "  Interface $IFACE is UP"
echo ""

# ---- Step 6: Start QMI network connection ----
echo "[6/7] Connecting to network (APN: $APN)..."

# Stop any existing connection first
qmicli -d "$QMI_DEVICE" --wds-stop-network=disable-autoconnect --device-open-proxy 2>/dev/null || true

# Start the connection
qmicli -d "$QMI_DEVICE" \
    --wds-start-network="apn=$APN,ip-type=4" \
    --client-no-release-cid \
    --device-open-proxy

echo ""

# ---- Step 7: Get IP address via DHCP ----
echo "[7/7] Requesting IP address..."

# Try udhcpc first (lightweight), fall back to dhclient
if command -v udhcpc &>/dev/null; then
    udhcpc -i "$IFACE" -q -n -t 10 2>&1 | tail -3
elif command -v dhclient &>/dev/null; then
    dhclient "$IFACE" -v 2>&1 | tail -3
else
    echo "  ERROR: No DHCP client found. Install udhcpc or dhclient."
    exit 1
fi

echo ""

# ---- Verify ----
echo "==========================================="
echo " Connection Status"
echo "==========================================="
echo ""
echo "Interface:"
ip addr show "$IFACE" | grep -E "inet |state"
echo ""
echo "Default route:"
ip route show default | head -3
echo ""
echo "DNS:"
cat /etc/resolv.conf | grep nameserver | head -3
echo ""

# Quick connectivity test
echo "Testing connectivity..."
if ping -I "$IFACE" -c 2 -W 5 8.8.8.8 &>/dev/null; then
    echo "  ✓ Internet is WORKING via $IFACE"
else
    echo "  ✗ Ping failed. Checking if route needs priority..."
    # Add a higher priority default route via wwan0
    GATEWAY=$(ip route show dev "$IFACE" | grep default | awk '{print $3}')
    if [ -n "$GATEWAY" ]; then
        ip route add default via "$GATEWAY" dev "$IFACE" metric 100 2>/dev/null || true
        if ping -I "$IFACE" -c 2 -W 5 8.8.8.8 &>/dev/null; then
            echo "  ✓ Internet is WORKING (added route with metric 100)"
        else
            echo "  ✗ Still no connectivity. Check APN and SIM data plan."
        fi
    fi
fi

echo ""
echo "==========================================="
echo " QMI Setup Complete"
echo "==========================================="
echo ""
echo "To disconnect:  sudo qmicli -d $QMI_DEVICE --wds-stop-network=disable-autoconnect --device-open-proxy"
echo "To reconnect:   sudo ./setup_qmi_sim7600e.sh $APN"
echo ""
echo "For auto-connect on boot, run:"
echo "  sudo ./setup_qmi_autoconnect.sh"
echo ""
