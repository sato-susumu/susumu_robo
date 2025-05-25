#!/usr/bin/env bash
set -eo pipefail

source ~/.bashrc
cd ~/rai
source ~/rai/setup_shell.sh

ros2 run rai_whoami rai_whoami_node --ros-args -p robot_description_package:=$ROBOT_NAME
