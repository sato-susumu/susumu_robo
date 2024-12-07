from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('susumu_robo')
    base_launch_file = os.path.join(pkg, 'launch', 'base.launch.py')
    collision_monitor_launch_file = os.path.join(pkg, 'launch', 'collision_monitor.launch.py')
    teleop_twist_joy_launch_file = os.path.join(pkg, 'launch', 'teleop_twist_joy.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch_file),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleop_twist_joy_launch_file),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(collision_monitor_launch_file),
        ),
    ])

