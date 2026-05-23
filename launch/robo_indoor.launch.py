from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('susumu_robo')

    ecef_to_enu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'ecef_to_enu.launch.py')
        )
    )

    mid360_launch = TimerAction(
        period=1.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, 'launch', 'mid360.launch.py')
                )
            )
        ]
    )

    imu_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, 'launch', 'imu_wt901.launch.py')
                )
            )
        ]
    )

    key_event_system_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, 'launch', 'key_event_system.launch.py')
                )
            )
        ]
    )

    laser_filter_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, 'launch', 'laser_filter.launch.py')
                )
            )
        ]
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

    # bringup.launch.py から teleop_twist_joy を除いた構成
    # (botwheel_teleop.launch.py が joy_node と teleop_twist_joy_node を起動するため重複を避ける)
    base_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, 'launch', 'base.launch.py')
                )
            )
        ]
    )

    collision_monitor_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, 'launch', 'collision_monitor.launch.py')
                )
            )
        ]
    )

    foxglove_bridge_pkg = get_package_share_directory('foxglove_bridge')
    foxglove_bridge_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    os.path.join(foxglove_bridge_pkg, 'launch', 'foxglove_bridge_launch.xml')
                ),
                launch_arguments={"port": "8765"}.items(),
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
        ecef_to_enu_launch,
        mid360_launch,
        imu_launch,
        key_event_system_launch,
        laser_filter_launch,
        bringup_diagnostic_indoor_launch,
        base_launch,
        collision_monitor_launch,
        foxglove_bridge_launch,
        botwheel_teleop_launch,
    ])
