from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='susumu_robo',
            executable='dummy_navsatfix_publisher',
            name='dummy_navsatfix_publisher',
            output='screen'
        ),
    ])