import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    topic = LaunchConfiguration('topic').perform(context)

    template_path = os.path.join(
        get_package_share_directory('susumu_robo'),
        'config',
        'livox_imu.rviz'
    )
    with open(template_path, 'r') as f:
        rviz_config_text = f.read().replace('__TOPIC__', topic)

    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.rviz', prefix='livox_imu_', delete=False
    )
    tmp.write(rviz_config_text)
    tmp.close()

    return [
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', tmp.name],
            output='screen',
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'topic',
            default_value='/livox/imu_ms2',
            description='IMU topic to visualize in RViz',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
