# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 知見管理システム

このプロジェクトは体系的な知見管理を採用しています。以下のファイルで情報を管理：

- **`.claude/context.md`**: プロジェクト背景、技術環境、制約事項
- **`.claude/project-knowledge.md`**: 技術的知見、アーキテクチャパターン、実装詳細
- **`.claude/project-improvements.md`**: 改善履歴、学習内容、将来の改善案
- **`.claude/common-patterns.md`**: よく使うコマンド、コードテンプレート、テスト方法
- **`.claude/debug-log.md`**: 既知の問題と解決策、デバッグ情報

## Project Overview

Susumu Robot is a compact, powerful mobile robot running ROS2. It features 3D LiDAR, depth camera, audio processing with wake word detection, speech recognition/synthesis, LED control, and navigation capabilities. The robot uses DDSM115 direct drive servo motors and is designed to be safe and easy to handle.

## Build System

This is a ROS2 workspace using colcon as the build system:

```bash
# Build entire workspace
cd /home/taro/ros2_ws
colcon build

# Build specific package
colcon build --packages-select susumu_robo

# Clean build (when encountering build issues)
cd /home/taro/ros2_ws
rm -rf build install log
colcon build

# Source the workspace after building
source install/setup.bash

# Build with parallel jobs
colcon build --parallel-workers 4
```

## Testing

The package includes standard ROS2 Python tests:

```bash
# Run tests for this package
cd /home/taro/ros2_ws
colcon test --packages-select susumu_robo

# Check test results
colcon test-result --verbose

# Run specific linters directly
ament_flake8 src/susumu_robo/susumu_robo/
ament_pep257 src/susumu_robo/susumu_robo/

# Run integration tests (example pattern)
ros2 launch susumu_robo test_launch.py
```

Linting is configured with:
- ament_flake8 for Python code style
- ament_pep257 for Python docstring style
- ament_copyright for copyright headers (currently skipped)

## System Architecture

### Launch File Hierarchy

The system uses a layered launch file structure:

- `ros2_bringup.launch.py` → `bringup.launch.py` (main system launcher)
  - `base.launch.py` (twist stamper for motor control)
  - `teleop_twist_joy.launch.py` (PS5 DualSense gamepad control)
  - `collision_monitor.launch.py` (obstacle avoidance)
  - Foxglove Bridge (visualization on port 8765)

### Hardware Components Integration

- **Motors**: DDSM115 direct drive servos via RS485 (`ros2_ddsm115.launch.py`)
- **LiDAR**: Livox Mid-360 for 3D mapping (`ros2_mid360.launch.py`)
- **Camera**: RealSense D435i depth camera (`ros2_realsense.launch.py`)
- **Audio**: Anker PowerConf S3 for speech I/O (`ros2_audio.launch.py`)
- **LEDs**: BlinkStick Strip via USB (`ros2_led.launch.py`)
- **Navigation**: SLAM and Nav2 integration (`ros2_nav2.launch.py`)

### Key ROS2 Nodes

- `twist_filter_node`: Safety filter that stops linear motion when obstacles detected
- `laserscan_filter_node`: Processes LiDAR data for obstacle detection
- `led_controller_node`: Controls LED patterns and colors based on obstacle detection
- `tenkey_controller`: Interface for 10-key input device

### Audio Pipeline

The robot includes sophisticated audio processing:
- Voice Activity Detection (VAD) using Silero model
- Wake word detection with multiple models (Alexa, Hey Jarvis, Hey Mycroft)
- Automatic Speech Recognition (ASR) integration
- Text-to-Speech via VoiceVox

## Service Management

The robot uses systemd services for automatic startup. Service scripts are located in the `launch/` directory:

```bash
# Start all core services
cd /home/taro/ros2_ws/src/susumu_robo/launch
./start_ros2_service.sh

# Check service status
./status_ros2_service.sh

# Stop all services
./stop_ros2_service.sh

# Control individual services
sudo systemctl start ros2_bringup.service
sudo systemctl status ros2_mid360.service
sudo systemctl stop ros2_ddsm115.service

# View service logs
journalctl -u ros2_bringup.service -f
```

Services include: ros2_bringup, ros2_mid360, ros2_ddsm115, ros2_realsense, ros2_led, ros2_nav2

## GUI Interface

The system includes a touch-friendly PySide6 GUI (`gui/menu.py`) with categorized command buttons configured via `gui/menu.yaml`. The GUI provides quick access to development tasks, URDF operations, launch commands, and system information.

```bash
# Launch GUI
cd /home/taro/ros2_ws/src/susumu_robo/gui
python3 menu.py
```

## Development Workflow

1. Make code changes in the `susumu_robo/` directory
2. Build with `colcon build` from workspace root
3. Source the install: `source install/setup.bash`
4. Test individual nodes or launch complete system
5. Use Foxglove Studio for visualization and debugging (http://localhost:8765)

## Navigation and SLAM

- Uses slam_toolbox for mapping (`script/slam_start.sh`)
- Nav2 for autonomous navigation
- Custom collision monitoring for safety
- RViz configurations available in `config/`

```bash
# Start SLAM mapping
cd /home/taro/ros2_ws/src/susumu_robo/script
./slam_start.sh

# Save map
./slam_save_map.sh

# Launch navigation
ros2 launch susumu_robo ros2_nav2.launch.py
```

## Common Commands

```bash
# Build and source
cd /home/taro/ros2_ws && colcon build && source install/setup.bash

# Launch main system
ros2 launch susumu_robo ros2_bringup.launch.py

# Launch navigation
ros2 launch susumu_robo ros2_nav2.launch.py

# Launch audio processing
ros2 launch susumu_robo ros2_audio.launch.py

# Monitor topics
ros2 topic list
ros2 topic echo /cmd_vel
ros2 topic hz /scan

# Check transforms
ros2 run tf2_tools view_frames
cd /home/taro/ros2_ws/src/susumu_robo/script && ./tf_save_pdf.sh

# Debug tools
ros2 run rqt_graph rqt_graph
ros2 run rqt_console rqt_console

# Performance monitoring
ros2 topic hz /scan
ros2 topic bw /image_raw
```

## Troubleshooting

### LiDAR Connection Issues
- Livox Mid-360 requires static IP configuration
- Check connection: `ping 192.168.1.50`
- Verify network interface has IP in 192.168.1.x range
- Check dmesg for USB/network errors

### Motor Control Issues
- DDSM115 motors use RS485 communication
- Check USB-RS485 converter permissions: `ls -la /dev/ttyUSB*`
- May need to add user to dialout group: `sudo usermod -a -G dialout $USER`
- Verify motor power (12V from distribution terminal)

### Audio Device Issues
- Anker PowerConf S3 must be set as default audio device
- Check device list: `arecord -l` and `aplay -l`
- Update `.asoundrc` if needed
- Restart audio service after changes

### System Health Checks
```bash
# Check all nodes are running
ros2 node list

# Verify expected topics exist
ros2 topic list | grep -E "(cmd_vel|scan|image)"

# Check service health
systemctl --user status ros2_*.service

# Monitor CPU/memory usage
htop
```

### Common Error Messages
- `"No module named 'susumu_robo'"`: Source the workspace: `source install/setup.bash`
- `"Transform timeout"`: Check if all TF publishers are running (base, LiDAR, camera)
- `"Serial port permission denied"`: Add user to dialout group and logout/login
- `"Package not found"`: Rebuild workspace with `colcon build`

### Development Best Practices
- Always test safety features (collision detection) after motor control changes
- Verify transforms with `view_frames` after modifying URDF or launch files
- Use Foxglove Studio for real-time debugging
- Monitor `/rosout` for warnings and errors
- Test emergency stop functionality regularly

## Safety Features

The system implements multiple safety layers:
- Collision monitoring using LiDAR data
- Twist filtering to prevent motion into obstacles
- Emergency stop capabilities
- Safe power management with 12V distribution
- LED indicators for obstacle detection status

## Quick Reference

### Essential Commands
```bash
# Emergency stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once

# System startup sequence
cd /home/taro/ros2_ws/src/susumu_robo/launch
./start_ros2_service.sh

# Quick diagnostics
ros2 topic hz /scan  # Should be ~10Hz
ros2 topic echo /scan_in_range  # Check obstacle detection
```

### Key File Locations
- **Nodes**: `susumu_robo/` directory
- **Launch files**: `launch/` directory
- **Service scripts**: `launch/*_ros2_service.sh`
- **GUI**: `gui/menu.py` and `gui/menu.yaml`
- **SLAM scripts**: `script/slam_*.sh`
- **Models**: `models/` (audio processing models)