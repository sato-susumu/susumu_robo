import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    slam_toolbox_share_dir = get_package_share_directory('slam_toolbox')
    susumu_robo_share_dir = get_package_share_directory('susumu_robo')

    slam_params_file = os.path.join(
        susumu_robo_share_dir, 'config', 'slam_toolbox_params.yaml'
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': slam_params_file,
        }.items()
    )

    rviz_config = os.path.join(susumu_robo_share_dir, 'config', 'slam.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    ld.add_action(slam_launch)
    ld.add_action(rviz_node)
    return ld
