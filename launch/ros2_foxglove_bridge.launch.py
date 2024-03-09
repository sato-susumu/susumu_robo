from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import SetRemap

def generate_launch_description():
    base_launch_share_dir = get_package_share_directory("foxglove_bridge")

    action = GroupAction(
        actions=[
            IncludeLaunchDescription(
                # XMLで書かれたlaunchファイルをベースにする
                XMLLaunchDescriptionSource([base_launch_share_dir + "/launch/foxglove_bridge_launch.xml"]),
                # 引数
                launch_arguments={
                    "port": "8765",
                }.items()
            )
        ]
    )

    return LaunchDescription([
        action,
    ])
