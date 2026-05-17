from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share_dir = get_package_share_directory('susumu_robo')

    # sensors_monitor from susumu_diagnostic
    sensors_monitor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('susumu_diagnostic'),
                'launch',
                'sensors_monitor.launch.py'
            )
        ])
    )

    # robo_doctor_node (indoor: GNSS/PTP checks disabled)
    robo_doctor_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='susumu_robo',
                executable='robo_doctor_node',
                name='robo_doctor_node',
                output='screen',
                parameters=[{
                    'enable_gnss_checks': False,
                    'enable_ptp_checks': False,
                }],
            )
        ]
    )

    # diagnostic_aggregator (starts after 4 second delay)
    diagnostic_aggregator_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='diagnostic_aggregator',
                executable='aggregator_node',
                name='diagnostic_aggregator',
                output='screen',
                parameters=[
                    os.path.join(package_share_dir, 'config', 'diagnostic_aggregator.yaml')
                ]
            )
        ]
    )

    # rqt_robot_monitor (starts after 6 second delay)
    rqt_robot_monitor_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rqt_robot_monitor',
                executable='rqt_robot_monitor',
                name='rqt_robot_monitor',
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        sensors_monitor_launch,
        robo_doctor_node,
        diagnostic_aggregator_node,
        rqt_robot_monitor_node,
    ])
