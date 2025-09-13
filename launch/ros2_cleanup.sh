#!/bin/bash

echo "Cleaning up ROS2 processes..."

# Stop ROS2 daemon first
ros2 daemon stop || true

# Kill specific ROS2 launch processes first
pkill -f "ros2 launch susumu_robo robo.launch.py" || true
pkill -f "septentrio_gnss_driver" || true

# Kill all ROS2 related processes
pkill -9 -f "ros2" || true
pkill -9 -f "_ros" || true
pkill -9 -f "livox_ros_driver2" || true
pkill -9 -f "witmotion_ros" || true
pkill -9 -f "realsense" || true
pkill -9 -f "rviz" || true
pkill -9 -f "foxglove" || true
pkill -9 -f "static_transform_publisher" || true
pkill -9 -f "tf2_ros" || true
pkill -9 -f "pointcloud_to_laserscan" || true
pkill -9 -f "livox_to_pointcloud2" || true

# Wait a moment for processes to terminate
sleep 1

# Kill remaining ROS2 processes more aggressively if needed
ps aux | grep -E "(ros2|static_transform|tf2_|livox|witmotion|septentrio|pointcloud_to_laserscan)" | grep -v grep | awk '{ print $2 }' | xargs -r kill -9 2>/dev/null || true

# Final check - kill any remaining ROS-related processes
ps aux | grep -E "/opt/ros/" | grep -v grep | awk '{ print $2 }' | xargs -r kill -9 2>/dev/null || true

# Restart ROS2 daemon to clear latched topics like /tf_static
ros2 daemon stop || true
sleep 1
ros2 daemon start || true

echo "ROS2 cleanup completed."
exit 0