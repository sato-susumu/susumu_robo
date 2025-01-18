import launch
import launch_ros.actions

from launch import LaunchDescription, LaunchService
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    return LaunchDescription([
        # respeaker_ros ノード
        Node(
            package='respeaker_ros',
            executable='respeaker_node',
            name='respeaker_node',
            output='screen',
        ),

        # susumu_gtts ノード
        Node(
            package='susumu_gtts',
            executable='susumu_gtts',
            name='susumu_gtts_node',
            output='screen'
        ),
    ])

if __name__ == '__main__':
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()
