## run node
```bash
ros2 run susumu_robo ultrasonic_monitor
```

## test commands
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
