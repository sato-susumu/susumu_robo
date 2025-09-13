import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml

def generate_launch_description():
    # Load GNSS configuration from YAML file
    config_file = os.path.join(
        get_package_share_directory('susumu_robo'),
        'config',
        'gnss.yaml'
    )
    
    with open(config_file, 'r') as f:
        gnss_params = yaml.safe_load(f)

    composable_node = ComposableNode(
        name='septentrio_gnss_driver',
        package='septentrio_gnss_driver', 
        plugin='rosaic_node::ROSaicNode',
        parameters=[gnss_params]
    )

    container = ComposableNodeContainer(
        name='septentrio_gnss_driver_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        composable_node_descriptions=[composable_node],
        output='screen'
    )

    return LaunchDescription([container])