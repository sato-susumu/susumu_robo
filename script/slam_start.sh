#!/bin/bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false slam_params_file:=$(ros2 pkg prefix susumu_robo)/share/susumu_robo/config/slam_toolbox_params.yaml
