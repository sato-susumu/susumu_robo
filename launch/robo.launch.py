from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    package_share_dir = get_package_share_directory('susumu_robo')

    # Include mid360.launch.py (starts after 1 second delay)
    mid360_launch = TimerAction(
        period=1.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(package_share_dir, 'launch', 'mid360.launch.py')
                ])
            )
        ]
    )

    # Include gnss.launch.py (starts after 2 second delay)
    gnss_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(package_share_dir, 'launch', 'gnss.launch.py')
                ])
            )
        ]
    )

    # Include imu_wt901.launch.py (starts after 4 second delay)
    imu_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(package_share_dir, 'launch', 'imu_wt901.launch.py')
                ])
            )
        ]
    )

    # Include dummy_navsatfix.launch.py (starts after 3 second delay)
    dummy_navsatfix_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(package_share_dir, 'launch', 'dummy_navsatfix.launch.py')
                ])
            )
        ]
    )

    return LaunchDescription([
        mid360_launch,
        gnss_launch,
        imu_launch,
        dummy_navsatfix_launch,
    ])