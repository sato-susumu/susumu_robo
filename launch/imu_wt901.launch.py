import os
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        os.path.dirname(__file__),
        'imu_wt901.yml'
        )

    tf_imu_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_camera_tf_publisher',
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu_link"]
    )

    node = Node(
        package='witmotion_ros',
        executable='witmotion_ros_node',
        parameters=[config]
    )

    ld.add_action(tf_imu_link)
    ld.add_action(node)
    return ld

if __name__ == '__main__':
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()