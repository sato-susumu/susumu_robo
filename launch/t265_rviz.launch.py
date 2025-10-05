import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # RViz設定ファイルのパス（このファイルと同じフォルダ）
    rviz_config = os.path.join(
        os.path.dirname(__file__),
        't265_rviz.rviz'
    )

    # RViz2ノード
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        rviz_node,
    ])
