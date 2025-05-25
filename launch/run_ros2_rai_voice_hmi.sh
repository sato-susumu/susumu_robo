#!/usr/bin/env bash
set -eo pipefail

source ~/.bashrc
cd ~/rai
source ~/rai/setup_shell.sh

exec ros2 run rai_hmi voice_hmi_node \
  --ros-args -p robot_description_package:=$ROBOT_NAME
