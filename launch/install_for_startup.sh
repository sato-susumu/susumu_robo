ros2 run robot_upstart install --job ros2_foxglove_bridge --systemd-after nss-lookup.target --setup /home/taro/ros2_ws/install/setup.bash --symlink susumu_robo/launch/ros2_foxglove_bridge.launch.py
ros2 run robot_upstart install --job ros2_teleop_twist_joy --symlink susumu_robo/launch/ros2_teleop_twist_joy.launch.py

ros2 run robot_upstart install --job ros2_collision_monitor --symlink susumu_robo/launch/ros2_collision_monitor.launch.py
ros2 run robot_upstart install --job ros2_kobuki_velocity_smoother --symlink susumu_robo/launch/ros2_kobuki_velocity_smoother.launch.py
ros2 run robot_upstart install --job ros2_micro_ros_agent --symlink susumu_robo/launch/ros2_micro_ros_agent.launch.py

ros2 run robot_upstart install --job ros2_msg_mid360 --symlink susumu_robo/launch/ros2_msg_mid360.launch.py
ros2 run robot_upstart install --job ros2_livox_to_pointcloud2 --symlink susumu_robo/launch/ros2_livox_to_pointcloud2.launch.py
