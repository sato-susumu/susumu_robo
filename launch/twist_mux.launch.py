import sys
import os
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('susumu_robo'),
        'config', 'twist_mux_topics.yaml'
    )

    return LaunchDescription([
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[config],
            remappings=[
                ('/cmd_vel_out', '/input_twist')
            ],
        )
    ])

if __name__ == '__main__':
    ls = LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())
