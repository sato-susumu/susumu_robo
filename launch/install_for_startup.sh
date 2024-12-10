ros2 run robot_upstart install --job ros2_susumu_robo_bringup --systemd-after nss-lookup.target --setup /home/taro/ros2_ws/install/setup.bash --symlink susumu_robo/launch/ros2_susumu_robo_bringup.launch.py

ros2 run robot_upstart install --job ros2_foxglove_bridge --systemd-after nss-lookup.target --setup /home/taro/ros2_ws/install/setup.bash --symlink susumu_robo/launch/ros2_foxglove_bridge.launch.py

ros2 run robot_upstart install --job ros2_msg_mid360 --symlink susumu_robo/launch/ros2_msg_mid360.launch.py
ros2 run robot_upstart install --job ros2_livox_to_pointcloud2 --symlink susumu_robo/launch/ros2_livox_to_pointcloud2.launch.py
ros2 run robot_upstart install --job ros2_pointcloud_to_laserscan --symlink susumu_robo/launch/ros2_pointcloud_to_laserscan.launch.py

