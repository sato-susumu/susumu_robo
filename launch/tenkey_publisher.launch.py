#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch引数の宣言
    keyboard_device_arg = DeclareLaunchArgument(
        'keyboard_device',
        default_value='/dev/input/by-id/usb-05a4_Tenkey_Keyboard-event-kbd',
        description='Path to the tenkey device'
    )

    key_event_topic_arg = DeclareLaunchArgument(
        'key_event_topic',
        default_value='key_event',
        description='Topic name for publishing key events'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the node'
    )

    # ノードの設定
    tenkey_publisher_node = Node(
        package='susumu_robo',
        executable='tenkey_publisher',
        name='tenkey_publisher',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'keyboard_device_path': LaunchConfiguration('keyboard_device'),
            'key_event_topic': LaunchConfiguration('key_event_topic')
        }],
        remappings=[
            # 必要に応じてトピックのリマッピングを追加
        ]
    )

    return LaunchDescription([
        keyboard_device_arg,
        key_event_topic_arg,
        namespace_arg,
        tenkey_publisher_node
    ])