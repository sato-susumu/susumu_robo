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
                'use_stamped': True,
                'topics.joy.topic':    '/joy_cmd_vel',
                'topics.joy.timeout':  2.0,
                'topics.joy.priority': 10,
                'topics.nav.topic':    '/nav_cmd_vel',
                'topics.nav.timeout':  2.0,
                'topics.nav.priority': 5,
                'topics.llm.topic':    '/llm_cmd_vel',
                'topics.llm.timeout':  2.0,
                'topics.llm.priority': 3,
            }],
            remappings=[
                ('/cmd_vel_out', '/input_twist')
            ],
        )
    ])

if __name__ == '__main__':
    ls = LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())
