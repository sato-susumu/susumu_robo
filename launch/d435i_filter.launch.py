from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pointcloud_filter_node = Node(
        package='susumu_robo',
        executable='pointcloud_filter_node',
        name='pointcloud_filter_node',
        parameters=[
            {'voxel_size': 0.05},  # ダウンサンプリングの格子サイズ(m)
            {'z_min': 0.1},        # 床面ノイズを除去する最低高さ(m)
            {'z_max': 2.0},        # 天井など不要な点を除去する最大高さ(m)
            {'max_range': 3.0},    # D435iの信頼できる最大距離(m)
        ],
        output='screen',
    )

    return LaunchDescription([
        pointcloud_filter_node,
    ])
