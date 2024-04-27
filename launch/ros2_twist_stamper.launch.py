from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='twist_stamper',
            executable='twist_stamper',
            remappings=[
                ('/cmd_vel_in','/collision_monitor/output_velocity'),
                ('/cmd_vel_out','/diffbot_base_controller/cmd_vel')
            ],
            output='screen',
        ),
    ])

