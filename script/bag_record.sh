#!/bin/bash
set -e

# 保存先ディレクトリ
OUTPUT_DIR="$HOME/ros2_bags/$(date +'%Y%m%d_%H%M%S')"

echo "Recording all ROS 2 topics to: $OUTPUT_DIR"
ros2 bag record -s mcap --all -o "$OUTPUT_DIR"
