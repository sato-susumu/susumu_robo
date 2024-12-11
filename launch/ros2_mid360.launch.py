from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Include msg_MID360_launch
    base_launch_share_dir = get_package_share_directory("livox_ros_driver2")
    msg_mid360_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([base_launch_share_dir + "/launch_ROS2/msg_MID360_launch.py"])
    )

    # 2. Odom Relay Node
    odom_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='odom_topic_relay',
        arguments=['/diffbot_base_controller/odom', '/odom']
    )

    # 3. Livox to PointCloud2 Node
    livox_to_pointcloud2_node = Node(
        package='livox_to_pointcloud2',
        executable='livox_to_pointcloud2_node',
        name='livox_to_pointcloud2_node',
        output='screen',
        remappings=[('/livox_pointcloud', '/livox/lidar')]
    )

    # 4. PointCloud to LaserScan Node
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', '/converted_pointcloud2'),
                    ('scan', '/scan')],
        parameters=[{
            'target_frame': '',
            'transform_tolerance': 0.01,
            'min_height': -0.2,  # -20cm
            'max_height': 0.2,   # 20cm
            'angle_min': -3.1415,  # -M_PI / 2
            'angle_max': 3.1415,   # M_PI / 2
            'angle_increment': 0.0087,  # 1 degree
            'scan_time': 0.1,  # 10Hz
            'range_min': 0.1,  # 10cm
            'range_max': 70.0,  # 70m
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        name='pointcloud_to_laserscan'
    )

    # 5. Static Transform Publisher for Livox Frame
    tf_livox_frame_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["0", "0", "0", "0", "0", "0", "laser_frame", "livox_frame"]
    )

    return LaunchDescription([
        msg_mid360_action,
        odom_relay_node,
        livox_to_pointcloud2_node,
        pointcloud_to_laserscan_node,
        tf_livox_frame_node
    ])
