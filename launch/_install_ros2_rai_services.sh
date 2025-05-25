#!/usr/bin/env bash
set -euo pipefail

SERVICE_DIR="/etc/systemd/system"
USER_NAME="taro"
WORK_DIR="/home/${USER_NAME}/rai"

echo "Writing ${SERVICE_DIR}/ros2_rai_whoami.service..."
sudo tee "${SERVICE_DIR}/ros2_rai_whoami.service" > /dev/null << EOF
[Unit]
Description=ROS2 RAI WhoAmI Node
After=network.target

[Service]
User=${USER_NAME}
WorkingDirectory=${WORK_DIR}
# コマンドラインとの互換性を重視して、対話シェルで呼び出す(あまりいい方法ではなさそう)
ExecStart=/bin/bash -i -c '/home/taro/ros2_ws/src/susumu_robo/launch/run_ros2_rai_whoami.sh'
Restart=on-failure
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

echo "Writing ${SERVICE_DIR}/ros2_rai_voice_hmi.service..."
sudo tee "${SERVICE_DIR}/ros2_rai_voice_hmi.service" > /dev/null << EOF
[Unit]
Description=ROS2 RAI HMI Voice Node
After=network.target

[Service]
User=${USER_NAME}
WorkingDirectory=${WORK_DIR}
# コマンドラインとの互換性を重視して、対話シェルで呼び出す(あまりいい方法ではなさそう)
ExecStart=/bin/bash -i -c '/home/taro/ros2_ws/src/susumu_robo/launch/run_ros2_rai_voice_hmi.sh'
Restart=on-failure
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

echo "Reloading systemd daemon..."
sudo systemctl daemon-reload

echo "Enabling services..."
sudo systemctl enable ros2_rai_whoami.service ros2_rai_voice_hmi.service

echo "Starting services..."
sudo systemctl restart ros2_rai_whoami.service ros2_rai_voice_hmi.service

echo "Done. Service statuses:"
sudo systemctl status ros2_rai_whoami.service ros2_rai_voice_hmi.service --no-pager
