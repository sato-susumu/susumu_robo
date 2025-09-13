#!/bin/bash

# GNSS起動スクリプト
# 残留プロセスを削除してからros2 launchを実行

echo "Stopping existing septentrio_gnss_driver processes..."
pkill -9 -f septentrio_gnss_driver

echo "Starting GNSS launch..."
ros2 launch susumu_robo gnss.launch.py