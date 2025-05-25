#!/usr/bin/env bash
set -euo pipefail

SERVICE_DIR="/etc/systemd/system"
SERVICES=(
  "ros2_rai_whoami.service"
  "ros2_rai_voice_hmi.service"
)

echo "Stopping services..."
for svc in "${SERVICES[@]}"; do
  sudo systemctl stop "$svc" || true
done

echo "Disabling services..."
for svc in "${SERVICES[@]}"; do
  sudo systemctl disable "$svc" || true
done

echo "Removing unit files..."
for svc in "${SERVICES[@]}"; do
  sudo rm -f "${SERVICE_DIR}/${svc}"
done

echo "Reloading systemd daemon..."
sudo systemctl daemon-reload

echo "Resetting failed state..."
sudo systemctl reset-failed

echo "Cleanup complete."
