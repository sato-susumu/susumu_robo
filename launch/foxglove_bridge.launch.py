#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('susumu_robo')

    # Foxglove Bridge Node
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'tls': False,
            'certfile': '',
            'keyfile': '',
            'topic_whitelist': ['.*'],
            'service_whitelist': ['.*'],
            'param_whitelist': ['.*'],
            'use_sim_time': False,
            'num_threads': 0,
            'max_qos_depth': 10,
            'use_compression': False,
            'capabilities': ['clientPublish', 'services', 'connectionGraph', 'assets']
        }],
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        foxglove_bridge_node
    ])