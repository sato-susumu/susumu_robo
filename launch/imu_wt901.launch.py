import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('susumu_robo'),
        'launch',
        'wt905.yml'
        )
        
    node = Node(
        package='witmotion_ros',
        executable='witmotion_ros_node',
        parameters=[config]
    )

    ld.add_action(node)
    return ld

if __name__ == '__main__':
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()