from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # SLAM Toolbox (online_async)
    slam_toolbox_share_dir = get_package_share_directory('slam_toolbox')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            slam_toolbox_share_dir + '/launch/online_async_launch.py'
        ])
    )

    # Nav2 bringup (navigation_launch)
    nav2_bringup_share_dir = get_package_share_directory('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            nav2_bringup_share_dir + '/launch/navigation_launch.py'
        ])
    )

    return LaunchDescription([
        slam_launch,
        nav2_launch
    ])
