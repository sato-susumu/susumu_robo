import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions
from launch import LaunchService


def generate_launch_description():
    joy_config = launch.substitutions.LaunchConfiguration('joy_config')
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('joy_vel', default_value='/input_twist'),
        # launch.actions.DeclareLaunchArgument('joy_vel', default_value='/cmd_vel'),
        launch.actions.DeclareLaunchArgument('joy_config', default_value='xbox'),
        launch.actions.DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        launch.actions.DeclareLaunchArgument('config_filepath', default_value=[
            launch.substitutions.TextSubstitution(text=os.path.join(
                get_package_share_directory('teleop_twist_joy'), 'config', '')),
            joy_config, launch.substitutions.TextSubstitution(text='.config.yaml')]),

        launch_ros.actions.Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]),
        launch_ros.actions.Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node',
            # PS5 DualSense用の設定
            parameters=[
                {
                    "require_enable_button": True,
                    "axis_linear.x": 1,
                    "axis_angular.yaw": 0,
                    "enable_button": 0,  # x button
                    "enable_turbo_button": 5,  # R1 shoulder button
                    "scale_angular.yaw": 1.0,
                    "scale_angular_turbo.yaw": 4.0,
                    "scale_linear.x": 0.3,
                    "scale_linear_turbo.x": 1.5,
                }
            ],
            remappings={('/cmd_vel', launch.substitutions.LaunchConfiguration('joy_vel'))},
            ),
    ])


if __name__ == '__main__':
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()