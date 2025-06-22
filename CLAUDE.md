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

# Source the workspace after building
source install/setup.bash
```

## Testing

The package includes standard ROS2 Python tests:

```bash
# Run tests for this package
colcon test --packages-select susumu_robo

# Check test results
colcon test-result --verbose
```

Linting is configured with:
- ament_flake8 for Python code style
- ament_pep257 for Python docstring style
- ament_copyright for copyright headers

## System Architecture

### Launch File Hierarchy

The system uses a layered launch file structure:

- `ros2_bringup.launch.py` → `bringup.launch.py` (main system launcher)
  - `base.launch.py` (twist stamper for motor control)
  - `teleop_twist_joy.launch.py` (PS5 DualSense gamepad control)
  - `collision_monitor.launch.py` (obstacle avoidance)
  - Foxglove Bridge (visualization)

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
- `led_controller_node`: Controls LED patterns and colors
- `tenkey_controller`: Interface for 10-key input device

### Audio Pipeline

The robot includes sophisticated audio processing:
- Voice Activity Detection (VAD) using Silero model
- Wake word detection with multiple models (Alexa, Hey Jarvis, Hey Mycroft)
- Automatic Speech Recognition (ASR) integration
- Text-to-Speech via VoiceVox

## Service Management

The robot uses systemd services for automatic startup:

```bash
# Start all core services
./start_ros2_service.sh

# Check service status
./status_ros2_service.sh

# Stop all services
./stop_ros2_service.sh
```

Services include: ros2_bringup, ros2_mid360, ros2_ddsm115, ros2_realsense, ros2_led, ros2_nav2

## GUI Interface

The system includes a touch-friendly PySide6 GUI (`gui/menu.py`) with categorized command buttons configured via `gui/menu.yaml`. The GUI provides quick access to development tasks, URDF operations, launch commands, and system information.

## Development Workflow

1. Make code changes in the `susumu_robo/` directory
2. Build with `colcon build` from workspace root
3. Source the install: `source install/setup.bash`
4. Test individual nodes or launch complete system
5. Use Foxglove Studio for visualization and debugging

## Navigation and SLAM

- Uses slam_toolbox for mapping (`script/slam_start.sh`)
- Nav2 for autonomous navigation
- Custom collision monitoring for safety
- RViz configurations available in `config/`

## File Structure

- `launch/`: ROS2 launch files (both legacy and ros2_ prefixed versions)
- `susumu_robo/`: Python source code for custom nodes
- `param/`: YAML parameter files
- `config/`: RViz configuration files
- `script/`: Utility bash scripts
- `gui/`: Touch interface application
- `models/`: ONNX/TensorFlow Lite models for audio processing
- `doc/`: Documentation files

## Dependencies

Key ROS2 packages this system depends on:
- rclpy, std_msgs (core ROS2)
- susumu_ros2_interfaces (custom message definitions)
- susumu_blinkstick_ros2 (LED control)
- respeaker_ros (audio hardware interface)
- nav2, slam_toolbox (navigation stack)
- foxglove_bridge (visualization)

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

# Check transforms
ros2 run tf2_tools view_frames
```

## Safety Features

The system implements multiple safety layers:
- Collision monitoring using LiDAR data
- Twist filtering to prevent motion into obstacles
- Emergency stop capabilities
- Safe power management with 12V distribution