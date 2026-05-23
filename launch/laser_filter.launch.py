import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('susumu_robo')
    filter_config = os.path.join(pkg_dir, 'config', 'laser_filter.yaml')

    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',

        remappings=[
            ('scan', '/scan_raw'),
            ('scan_filtered', '/scan'),
        ],
        parameters=[filter_config],
    )

    return LaunchDescription([laser_filter_node])
