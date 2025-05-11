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

        # voicevox_ros2 ノード
        Node(
            package='voicevox_ros2',
            executable='voicevox_ros2_core',
            name='voicevox_ros2',
            output='screen'
        ),

        # voicevox_ros2 ノード
        Node(
            package='to_human_2_voicevox',
            executable='to_human_2_voicevox',
            name='to_human_2_voicevox',
            output='screen'
        ),
    ])

if __name__ == '__main__':
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()
