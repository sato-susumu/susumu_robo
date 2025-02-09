import sys
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[{
                'topics': {
                    'key': {'topic': '/key_cmd_vel', 'timeout': 10.0, 'priority': 10},
                    'joy': {'topic': '/joy_cmd_vel', 'timeout': 2.0, 'priority': 8},
                    'llm': {'topic': '/llm_cmd_vel', 'timeout': 0.5, 'priority': 5}
                },
            }],
            remappings=[
                ('/cmd_vel_out', '/merged_cmd_vel')
            ],
        )
    ])

if __name__ == '__main__':
    ls = LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())
