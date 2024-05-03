from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='odom_relay',
        arguments=['/diffbot_base_controller/odom', '/odom']
    )

    return LaunchDescription([
        relay_node
    ])
