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
            parameters=[{
                "accel_lim_v": 0.2,
                "accel_lim_w": 3.5,
                "speed_lim_v": 1.5,
                "speed_lim_w": 6.0,
            }],
            output='screen',
        ),
    ])
