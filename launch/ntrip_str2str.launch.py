from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch arguments
    ntrip_server_arg = DeclareLaunchArgument(
        'ntrip_server',
        default_value='ntrip://geortk.jp:2101/KOBEKOKA',
        description='NTRIP server URL'
    )

    output_dest_arg = DeclareLaunchArgument(
        'output_dest',
        default_value='tcpcli://192.168.3.1:28785',
        description='Output destination (GNSS receiver)'
    )

    # NTRIP str2str node
    ntrip_str2str_node = Node(
        package='susumu_robo',
        executable='ntrip_str2str_node',
        name='ntrip_str2str_node',
        output='screen',
        parameters=[{
            'ntrip_server': LaunchConfiguration('ntrip_server'),
            'output_dest': LaunchConfiguration('output_dest'),
        }]
    )

    return LaunchDescription([
        ntrip_server_arg,
        output_dest_arg,
        ntrip_str2str_node,
    ])
