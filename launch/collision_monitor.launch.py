from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output_topic', default_value='/collision_monitor/output_velocity',
                             description='Output topic for collision monitor'),
        Node(
            package='susumu_robo',
            executable='collision_monitor',
#            remappings=[
#                ('/collision_monitor/output_velocity', '/diffbot_base_controller/cmd_vel')
#            ],
            output='screen',
        ),
    ])

