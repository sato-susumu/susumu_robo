#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Launch引数の宣言
    key_event_topic_arg = DeclareLaunchArgument(
        'key_event_topic',
        default_value='key_event',
        description='Topic name to subscribe for key events'
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

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the node'
    )

    # ノードの設定
    key_event_handler_node = Node(
        package='susumu_robo',
        executable='key_event_handler',
        name='key_event_handler',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'key_event_topic': LaunchConfiguration('key_event_topic'),
            'robo_doctor_path': LaunchConfiguration('robo_doctor_path'),
            'rosbag_base_dir': LaunchConfiguration('rosbag_base_dir')
        }],
        remappings=[
            # 必要に応じてトピックのリマッピングを追加
        ]
    )

    return LaunchDescription([
        key_event_topic_arg,
        robo_doctor_path_arg,
        rosbag_base_dir_arg,
        namespace_arg,
        key_event_handler_node
    ])