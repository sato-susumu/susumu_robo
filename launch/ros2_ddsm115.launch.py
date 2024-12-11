from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # diffbotのLaunchファイルを含める
    base_launch_share_dir = get_package_share_directory("diffdrive_ddsm115")
    diffbot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([base_launch_share_dir + "/launch/diffbot.launch.py"]),
    )

    # odom_relayノードを追加
    relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='odom_relay',
        arguments=['/diffbot_base_controller/odom', '/odom']
    )

    # LaunchDescriptionに両方のアクションを追加
    return LaunchDescription([
        diffbot_launch,
        relay_node
    ])
