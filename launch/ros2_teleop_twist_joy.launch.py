import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions


def generate_launch_description():
    joy_config = launch.substitutions.LaunchConfiguration('joy_config')
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('joy_vel', default_value='/kobuki_velocity_smoother/input'),
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
            parameters=[
                launch.substitutions.LaunchConfiguration('config_filepath'),
                {
                    "require_enable_button": False,
                    "scale_angular.yaw": 1.0,
                    "scale_angular_turbo.yaw": 4.0,
                    "scale_linear.x": 0.3,
                    "scale_linear_turbo.x": 1.5,
                }
            ],
            remappings={('/cmd_vel', launch.substitutions.LaunchConfiguration('joy_vel'))},
            ),
    ])
