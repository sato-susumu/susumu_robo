# USBzyoukyoukakunin
```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
```
/dev/ttyACM0 to /dev/ttyUSB0 gaareba ok

# micro-ROS Agent kidou
### /dev/ttyUSB0 muke agent
```bash
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO serial --dev /dev/ttyUSB0 -v6
```
### /dev/ttyACM0 muke agent
```bash
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO serial --dev /dev/ttyACM0 -v6
```

### multi agent(/dev/ttyACM0, /dev/ttyACM1)
```bash
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO multiserial --devs "/dev/ttyACM0 /dev/ttyACM1" -v6
```

### multi agent(/dev/ttyACM0, /dev/ttyUSB0)
```bash
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO multiserial --devs "/dev/ttyACM0 /dev/ttyUSB0" -v6
```



# topic check
```bash
ros2 topic list
```

```bash
ros2 topic echo /ultrasonic0
```

# docker kill all
```bash	
docker kill $(docker ps -a)
```

# run ultrasonic_monitor
```bash
ros2 run susumu_robo ultrasonic_monitor
```

# run joystick nodes
```bash
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
```

# topic check
```bash
ros2 topic echo /cmd_vel
```

```bash
ros2 topic echo /cmd_vel_ddd
```
