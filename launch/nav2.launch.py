import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml = os.path.expanduser('~/map.yaml')

    pkg_share = get_package_share_directory('susumu_robo')
    nav2_params = pkg_share + '/config/nav2_params.yaml'

    nav2_bringup_share_dir = get_package_share_directory('nav2_bringup')

    # bringup_launch.py = localization (map_server + amcl) + navigation
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            nav2_bringup_share_dir + '/launch/bringup_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            'params_file': nav2_params,
        }.items(),
    )

    nav2_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            nav2_bringup_share_dir + '/launch/rviz_launch.py'
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    relay_cmd_vel = Node(
        package='twist_stamper',
        executable='twist_stamper',
        remappings=[
            ('/cmd_vel_in', '/cmd_vel'),
            ('/cmd_vel_out', '/botwheel_explorer/cmd_vel'),
        ],
    )

    return LaunchDescription([
        nav2_launch,
        nav2_rviz_launch,
        relay_cmd_vel,
    ])
