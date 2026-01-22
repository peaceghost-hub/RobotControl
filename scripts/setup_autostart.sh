#!/usr/bin/env bash
# Setup systemd service to auto-start the Robot Control on Raspberry Pi

set -euo pipefail

SERVICE_NAME="robotcontrol"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"

# Resolve repo root from this script location
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
START_SCRIPT="${REPO_ROOT}/scripts/start_robot.sh"
WORK_DIR="${REPO_ROOT}/raspberry_pi"

usage() {
  echo "Usage: $0 [--install|--remove]"
  echo "  --install  Install and enable systemd service"
  echo "  --remove   Disable and remove the systemd service"
}

if [[ ${1:-} == "--remove" ]]; then
  echo "Stopping and disabling ${SERVICE_NAME} service..."
  sudo systemctl stop "${SERVICE_NAME}" || true
  sudo systemctl disable "${SERVICE_NAME}" || true
  if [[ -f "${SERVICE_FILE}" ]]; then
    sudo rm -f "${SERVICE_FILE}"
    sudo systemctl daemon-reload
  fi
  echo "Removed ${SERVICE_NAME} service."
  exit 0
fi

if [[ ${1:-} != "--install" ]]; then
  usage
  exit 1
fi

if [[ ! -f "${START_SCRIPT}" ]]; then
  echo "Error: start script not found at ${START_SCRIPT}"
  exit 1
fi

echo "Creating systemd service at ${SERVICE_FILE}..."
sudo tee "${SERVICE_FILE}" > /dev/null <<EOF
[Unit]
Description=Robot Control System (Raspberry Pi)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=pi
WorkingDirectory=${WORK_DIR}
ExecStart=${START_SCRIPT}
Restart=always
RestartSec=5
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=multi-user.target
EOF

echo "Reloading systemd, enabling and starting service..."
sudo systemctl daemon-reload
sudo systemctl enable "${SERVICE_NAME}"
sudo systemctl start "${SERVICE_NAME}"

echo "${SERVICE_NAME} service installed and started."
echo "Check status:    sudo systemctl status ${SERVICE_NAME}"
echo "View logs:       journalctl -u ${SERVICE_NAME} -f"