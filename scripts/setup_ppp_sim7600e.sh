#!/usr/bin/env bash
# Setup PPP for SIM7600E on Raspberry Pi (USB or UART)
# This script installs ppp and creates chat scripts to bring up a PPP link.

set -euo pipefail

DEVICE=${1:-/dev/ttyUSB0}
APN=${2:-internet}
BAUD=${3:-115200}

sudo apt-get update && sudo apt-get install -y ppp

sudo mkdir -p /etc/ppp/peers
sudo tee /etc/ppp/peers/sim7600e >/dev/null <<EOF
${DEVICE}
115200
connect "/usr/sbin/chat -v -f /etc/ppp/chat-sim7600e"
noauth
usepeerdns
defaultroute
persist
holdoff 5
maxfail 0
crtscts
user ""
password ""
EOF

sudo tee /etc/ppp/chat-sim7600e >/dev/null <<EOF
ABORT "BUSY"
ABORT "NO CARRIER"
ABORT "ERROR"
TIMEOUT 12
"" AT
OK ATZ
OK AT+CFUN=1
OK AT+CPIN?
OK AT+CGATT=1
OK AT+CGDCONT=1,\"IP\",\"${APN}\"
OK ATD*99#
CONNECT ""
EOF

echo "Bring up PPP: sudo pppd call sim7600e"
echo "Tear down PPP: sudo poff -a"
