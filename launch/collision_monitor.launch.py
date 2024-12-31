from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # laserscan_filter_nodeの起動
        Node(
            package='susumu_robo',
            executable='laserscan_filter_node',
            name='laserscan_filter_node',
            output='screen',
            parameters=[
                {'forward.x_min': -0.25},
                {'forward.x_max': 0.25},
                {'forward.y_min': -0.25},
                {'forward.y_max': 0.25},
                {'backward.x_min': -0.25},
                {'backward.x_max': 0.25},
                {'backward.y_min': -0.25},
                {'backward.y_max': 0.25},
                {'reference_link': 'livox_frame'},
            ],
            remappings=[
                ('cmd_vel', 'input_twist'),
            ]
        ),
        # twist_filter_nodeの起動
        Node(
            package='susumu_robo',
            executable='twist_filter_node',
            name='twist_filter_node',
            output='screen',
            parameters=[],
            remappings=[
                ('enable', 'scan_in_range'),
                ('output_twist', 'cmd_vel'),
            ]
        )
    ])

if __name__ == '__main__':
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()
