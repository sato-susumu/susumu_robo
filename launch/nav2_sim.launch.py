"""シミュレータ用 Nav2 起動.

実機 `nav2.launch.py` と同等だが use_sim_time をデフォルトで true に固定.
マップは引数 `map` で指定 (デフォルトは ~/sim_map.yaml).

使い方:
    ros2 launch susumu_robo nav2_sim.launch.py
    ros2 launch susumu_robo nav2_sim.launch.py map:=/abs/path/to/foo.yaml

事前にマップが必要. 初回は下記で SLAM して保存:
    ros2 launch susumu_robo slam_toolbox_sim.launch.py
    ros2 run nav2_map_server map_saver_cli -f ~/sim_map
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('susumu_robo')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.expanduser('~/sim_map.yaml'),
        description='マップファイルパス. 事前に slam_toolbox_sim.launch.py で生成しておくこと.',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='RViz を起動するか'
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map': LaunchConfiguration('map'),
            'params_file': nav2_params,
        }.items(),
    )

    nav2_rviz = GroupAction(
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'rviz_launch.py')
            ),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
        )],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    relay_cmd_vel = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/cmd_vel_in', '/cmd_vel'),
            ('/cmd_vel_out', '/nav_cmd_vel'),
        ],
    )

    return LaunchDescription([
        map_arg,
        use_sim_time_arg,
        rviz_arg,
        nav2,
        nav2_rviz,
        relay_cmd_vel,
    ])
