from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap

def generate_launch_description():
    base_launch_share_dir = get_package_share_directory("teleop_twist_joy")
    susumu_robo_launch_share_dir = get_package_share_directory("susumu_robo")

    action = GroupAction(
        actions=[
            # remap
            SetRemap(src='/cmd_vel', dst='/kobuki_velocity_smoother/input'),
            IncludeLaunchDescription(
                # Pythonで書かれたlaunchファイルをベースにする
                PythonLaunchDescriptionSource([base_launch_share_dir + "/launch/teleop-launch.py"]),
                # 引数
                launch_arguments={
                   "joy_config": "xbox",
                   "config_filepath": susumu_robo_launch_share_dir + "/param/teleop_twist_joy_node.yaml",
                }.items()
            ),
        ]
    )

    return LaunchDescription([
        action,
    ])
