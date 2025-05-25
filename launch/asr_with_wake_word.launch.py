# launch/combined_openwakeword_and_stt_bridge.py

from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    openwakeword_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('susumu_asr_ros'),
                'launch',
                'openwakeword_google.launch.py'
            )
        )
    )

    stt_bridge_node = Node(
        package='stt_2_from_human',
        executable='stt_2_from_human',
        name='stt_2_from_human',
        output='screen'
    )

    return LaunchDescription([
        openwakeword_launch,
        stt_bridge_node,
    ])


if __name__ == '__main__':
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()
