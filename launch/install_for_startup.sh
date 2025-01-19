ros2 run robot_upstart install --job ros2_bringup --systemd-after nss-lookup.target --setup /home/taro/ros2_ws/install/setup.bash --symlink susumu_robo/launch/ros2_bringup.launch.py
ros2 run robot_upstart install --job ros2_mid360 --systemd-after nss-lookup.target --setup /home/taro/ros2_ws/install/setup.bash --symlink susumu_robo/launch/ros2_mid360.launch.py
ros2 run robot_upstart install --job ros2_ddsm115 --systemd-after nss-lookup.target --setup /home/taro/ros2_ws/install/setup.bash --symlink susumu_robo/launch/ros2_ddsm115.launch.py
ros2 run robot_upstart install --job ros2_realsense --systemd-after nss-lookup.target --setup /home/taro/ros2_ws/install/setup.bash --symlink susumu_robo/launch/ros2_realsense.launch.py
ros2 run robot_upstart install --job ros2_audio --systemd-after nss-lookup.target --setup /home/taro/ros2_ws/install/setup.bash --symlink susumu_robo/launch/ros2_audio.launch.py --user $USER
ros2 run robot_upstart install --job ros2_led --systemd-after nss-lookup.target --setup /home/taro/ros2_ws/install/setup.bash --symlink susumu_robo/launch/ros2_led.launch.py

