from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap

def generate_launch_description():
    base_launch_share_dir = get_package_share_directory("livox_ros_driver2")

    action = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([base_launch_share_dir + "/launch_ROS2/msg_MID360_launch.py"]),
            )
        ]
    )

    return LaunchDescription([
        action,
    ])
