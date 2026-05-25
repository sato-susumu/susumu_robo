"""Launch file for ODrive BotWheel Explorer with joystick teleop control."""

from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os


def generate_launch_description():
    # ODrive BotWheel Explorer package
    botwheel_pkg = get_package_share_directory('susumu_robo')
    botwheel_launch_file = os.path.join(
        botwheel_pkg, 'susumu_diffbot/launch', 'botwheel_explorer.launch.py'
    )

    # Xbox config file path
    teleop_twist_joy_pkg = get_package_share_directory('teleop_twist_joy')
    xbox_config = os.path.join(
        teleop_twist_joy_pkg, 'config', 'xbox.config.yaml'
    )

    return LaunchDescription([
        # Launch ODrive BotWheel Explorer
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(botwheel_launch_file),
        ),

        # Joy node
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
        ),

        # Teleop twist joy node
        launch_ros.actions.Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[
                xbox_config,
                {
                    'publish_stamped_twist': True,
                    'scale_linear.x': 0.3,
                    'scale_linear_turbo.x': 0.9,
                    'scale_angular.yaw': 1.0,
                    'scale_angular_turbo.yaw': 1.0,
                    'enable_turbo_button': -1,  # disabled (was: 5 = R1 shoulder button)
                }
            ],
            remappings=[
                ('cmd_vel', '/joy_cmd_vel'),
            ],
        ),
    ])


if __name__ == '__main__':
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()
