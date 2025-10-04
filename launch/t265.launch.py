import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # パラメータファイルのパス（このファイルと同じフォルダ）
    config = os.path.join(
        os.path.dirname(__file__),
        't265_params.yaml'
    )

    # RealSense T265ノード
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='t265',
        parameters=[config],
        output='screen',
    )

    return LaunchDescription([
        realsense_node,
    ])