from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('susumu_robo')

    robo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'robo.launch.py')
        )
    )

    bringup_diagnostic_indoor_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, 'launch', 'bringup_diagnostic_indoor.launch.py')
                )
            )
        ]
    )

    bringup_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, 'launch', 'bringup.launch.py')
                )
            )
        ]
    )

    botwheel_teleop_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, 'launch', 'botwheel_teleop.launch.py')
                )
            )
        ]
    )

    return LaunchDescription([
        robo_launch,
        bringup_diagnostic_indoor_launch,
        bringup_launch,
        botwheel_teleop_launch,
    ])
