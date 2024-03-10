from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('outpu_topic', default_value='/collision_monitor/input_velocity',
                             description='Output topic for velocity smoother'),
        Node(
            package='kobuki_velocity_smoother',
            executable='velocity_smoother',
            # name='velocity_smoother',
            remappings=[
                ('/kobuki_velocity_smoother/smoothed', '/collision_monitor/input_velocity')
            ],
            output='screen',
        ),
    ])
