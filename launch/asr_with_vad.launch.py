from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    silero_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('susumu_asr_ros'),
                'launch',
                'silerovad_google.launch.py'
            )
        )
    )

    stt_bridge_node = Node(
        package='stt_2_from_human',
        executable='stt_2_from_human',
        output='screen',
        name='stt_2_from_human'
    )

    return LaunchDescription([
        silero_launch,
        stt_bridge_node,
    ])

if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
