import json
import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _get_ip_on_subnet(target_ip: str) -> str | None:
    """Return the local IP the kernel would use to reach target_ip (via ip route get)."""
    result = subprocess.run(
        ['ip', 'route', 'get', target_ip],
        capture_output=True, text=True
    )
    # Output example: "192.168.1.145 dev enxf8e43b07988f src 192.168.1.5 uid 1000"
    for token in result.stdout.split():
        if token == 'src':
            idx = result.stdout.split().index('src')
            return result.stdout.split()[idx + 1]
    return None


def generate_launch_description():
    pkg_dir = get_package_share_directory('susumu_robo')
    config_path = os.path.join(pkg_dir, 'config', 'MID360_config.json')

    with open(config_path) as f:
        cfg = json.load(f)
    lidar_ip = cfg['lidar_configs'][0]['ip']
    detected_host_ip = _get_ip_on_subnet(lidar_ip)

    if not detected_host_ip:
        raise RuntimeError(
            f'[mid360.launch] No host IP found on the same /24 subnet as LiDAR ({lidar_ip}). '
            'Check network interface configuration.'
        )

    for key in cfg['MID360']['host_net_info']:
        if key.endswith('_ip'):
            cfg['MID360']['host_net_info'][key] = detected_host_ip
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.json', delete=False, dir='/tmp', prefix='MID360_config_'
    )
    json.dump(cfg, tmp, indent=2)
    tmp.close()
    runtime_config_path = tmp.name
    print(f'[mid360.launch] LiDAR IP: {lidar_ip}, Host IP detected: {detected_host_ip}')

    livox_ros2_params = [
        {'xfer_format': 1},
        {'multi_topic': 0},
        {'data_src': 0},
        {'publish_freq': 10.0},
        {'output_data_type': 0},
        {'frame_id': 'livox_frame'},
        {'lvx_file_path': '/home/livox/livox_test.lvx'},
        {'user_config_path': runtime_config_path},
        {'cmdline_input_bd_code': 'livox0000000001'},
    ]

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params,
    )

    livox_to_pointcloud2_node = Node(
        package='livox_to_pointcloud2',
        executable='livox_to_pointcloud2_node',
        name='livox_to_pointcloud2_node',
        output='screen',
        remappings=[('/livox_pointcloud', '/livox/lidar')]
    )

    # PointCloud to LaserScan Node
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', '/converted_pointcloud2'),
                    ('scan', '/scan')],
        parameters=[{
            'target_frame': 'base_link',
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

    # Static Transform Publisher for Livox Frame
    tf_livox_frame_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["0", "0", "0", "0", "0", "0", "laser_frame", "livox_frame"]
    )

    # Livox IMU Converter Node (G to m/s²)
    livox_imu_converter_node = Node(
        package='susumu_robo',
        executable='livox_imu_converter',
        name='livox_imu_converter',
        output='screen'
    )

    return LaunchDescription([
        livox_driver,
        livox_to_pointcloud2_node,
        pointcloud_to_laserscan_node,
        tf_livox_frame_node,
        livox_imu_converter_node
    ])
