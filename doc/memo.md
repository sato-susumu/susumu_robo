# micro-ROS Agent起動
### USB状況確認
```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
```

### /dev/ttyUSB0 のみ
```bash
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO serial --dev /dev/ttyUSB0 -v6
```
### /dev/ttyACM0 のみ
```bash
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO serial --dev /dev/ttyACM0 -v6
```

### /dev/ttyACM0, /dev/ttyACM1 両方
```bash
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO multiserial --devs "/dev/ttyACM0 /dev/ttyACM1" -v6
```

### /dev/ttyACM0, /dev/ttyUSB0 両方
```bash
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO multiserial --devs "/dev/ttyACM0 /dev/ttyUSB0" -v6
```


# foxglove_bridge
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

# ローンチサンプル(teleop_twist_joy, kobuki_velocity_smoother, susumu_robo)
```bash
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox' joy_vel:=/kobuki_velocity_smoother/input
```

```bash
ros2 run kobuki_velocity_smoother velocity_smoother --ros-args -r /kobuki_velocity_smoother/smoothed:=/collision_monitor/input_velocity
```

```bash
ros2 run susumu_robo collision_monitor --ros-args -r /collision_monitor/output_velocity:=/cmd_vel
```

# ローンチサンプル(teleop_twist_joy, susumu_robo)
```bash
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox' joy_vel:=/collision_monitor/input_velocity
```

```bash
ros2 run susumu_robo collision_monitor --ros-args -r /collision_monitor/output_velocity:=/cmd_vel
```

# memo
### build
```bash
colcon build --symlink-install
source ./install/setup.bash
```
### topic check
```bash
ros2 topic list
```

```bash
ros2 topic echo /ultrasonic0
```

### docker kill all
```bash	
docker kill $(docker ps -a)
```

### topic check
```bash
ros2 topic echo /cmd_vel
```

### publish cmd_vel
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" &
```

### publish ultrasonic sensor data
```bash
ros2 topic pub /ultrasonic0 sensor_msgs/msg/Range "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'range_sensor'}, radiation_type: 0, field_of_view: 0.0, min_range: 0.0, max_range: 0.0, range: 0.2}" &
ros2 topic pub /ultrasonic1 sensor_msgs/msg/Range "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'range_sensor'}, radiation_type: 0, field_of_view: 0.0, min_range: 0.0, max_range: 0.0, range: 0.2}" &
ros2 topic pub /ultrasonic2 sensor_msgs/msg/Range "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'range_sensor'}, radiation_type: 0, field_of_view: 0.0, min_range: 0.0, max_range: 0.0, range: 0.2}" &
```

