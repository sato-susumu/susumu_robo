#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # パッケージのパス取得
    pkg_dir = FindPackageShare('susumu_robo')

    # Launch引数の宣言
    tenkey_device_arg = DeclareLaunchArgument(
        'tenkey_device',
        default_value='/dev/input/by-id/usb-05a4_Tenkey_Keyboard-event-kbd',
        description='Path to the tenkey device'
    )

    key_event_topic_arg = DeclareLaunchArgument(
        'key_event_topic',
        default_value='key_event',
        description='Topic name for key events'
    )

    robo_doctor_path_arg = DeclareLaunchArgument(
        'robo_doctor_path',
        default_value='/home/taro/ros2_ws/src/susumu_robo/launch/robo_doctor.py',
        description='Path to the robo_doctor.py script'
    )

    rosbag_base_dir_arg = DeclareLaunchArgument(
        'rosbag_base_dir',
        default_value=os.path.expanduser('~/ros2_bags'),
        description='Base directory for rosbag recordings'
    )

    enable_tenkey_arg = DeclareLaunchArgument(
        'enable_tenkey',
        default_value='false',
        description='Enable tenkey publisher'
    )

    # Tenkey Publisher (テンキーパッド)
    tenkey_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_dir, 'launch', 'tenkey_publisher.launch.py'])
        ]),
        launch_arguments={
            'keyboard_device': LaunchConfiguration('tenkey_device'),
            'key_event_topic': LaunchConfiguration('key_event_topic')
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_tenkey'))
    )

    # Key Event Handler (キーイベント処理)
    key_event_handler_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_dir, 'launch', 'key_event_handler.launch.py'])
        ]),
        launch_arguments={
            'key_event_topic': LaunchConfiguration('key_event_topic'),
            'robo_doctor_path': LaunchConfiguration('robo_doctor_path'),
            'rosbag_base_dir': LaunchConfiguration('rosbag_base_dir')
        }.items()
    )

    # 全体のグループ化
    key_event_system = GroupAction([
        tenkey_publisher_launch,
        key_event_handler_launch
    ])

    return LaunchDescription([
        # 引数の宣言
        tenkey_device_arg,
        key_event_topic_arg,
        robo_doctor_path_arg,
        rosbag_base_dir_arg,
        enable_tenkey_arg,
        # ノードの起動
        key_event_system
    ])