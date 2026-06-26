"""シミュレータ用 collision_monitor.

frame を `livox_frame` (実機) から `laser_frame` (sim URDF) に切り替え.
出力 (TwistStamped) は直接 `/botwheel_explorer/cmd_vel` (DiffDriveController, use_stamped_vel=true) に流す.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='susumu_robo',
            executable='laserscan_filter_node',
            name='laserscan_filter_node',
            output='screen',
            parameters=[
                {'forward.x_min': 0.0},
                {'forward.x_max': 0.25},
                {'forward.y_min': -0.25},
                {'forward.y_max': 0.25},
                {'backward.x_min': -0.50},
                {'backward.x_max': 0.0},
                {'backward.y_min': -0.25},
                {'backward.y_max': 0.25},
                {'reference_link': 'laser_frame'},
                {'use_sim_time': True},
            ],
            remappings=[
                ('cmd_vel', '/botwheel_explorer/cmd_vel'),
            ],
        )
    ])
