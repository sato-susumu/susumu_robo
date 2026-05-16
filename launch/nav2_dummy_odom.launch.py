from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # odom -> base_link をゼロ値で固定配信（ダミーodom）
    dummy_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'odom', '--child-frame-id', 'base_link'],
    )

    slam_toolbox_share_dir = get_package_share_directory('slam_toolbox')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            slam_toolbox_share_dir + '/launch/online_async_launch.py'
        ])
    )

    nav2_bringup_share_dir = get_package_share_directory('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            nav2_bringup_share_dir + '/launch/navigation_launch.py'
        ])
    )

    return LaunchDescription([
        dummy_odom,
        slam_launch,
        nav2_launch,
    ])
