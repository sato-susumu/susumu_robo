"""シミュレータ用 slam_toolbox 起動.

実機 `script/slam_start.sh` (slam_toolbox online_async) を use_sim_time=true で起動する.
事後マップ保存は:
    ros2 run nav2_map_server map_saver_cli -f ~/sim_map
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('susumu_robo')
    slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    slam_share = get_package_share_directory('slam_toolbox')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true'
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': slam_params,
        }.items(),
    )

    return LaunchDescription([use_sim_time_arg, slam])
