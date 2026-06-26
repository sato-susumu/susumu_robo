from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='susumu_robo',
            executable='dummy_speak_action_server',
            name='dummy_speak_action_server',
            output='screen',
        ),
        Node(
            package='susumu_robo',
            executable='to_human_2_speak_ros',
            name='to_human_2_speak_ros',
            output='screen',
        ),
    ])
