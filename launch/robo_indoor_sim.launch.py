"""シミュレータ用 robo_indoor (Gazebo 上で robo_indoor 相当を立ち上げる).

構成:
- sim_robot (Gazebo + URDF + DiffDriveController)
- collision_monitor_sim (frame=laser_frame, 出力は /cmd_vel_stamped_filtered)
- twist_stamped_to_twist (上記出力を Twist に変換し DiffDriveController へ)
- twist_mux (joy/nav/llm を /input_twist にミックス)
- foxglove_bridge
- dummy_navsatfix (GNSS 代用)
- 任意: audio_option_sim (オフライン音声)

実機 launch との違い:
- mid360 / d435i / botwheel_teleop / laser_filter / key_event_system / led 等は使用しない
- 衝突回避とパイプラインの繋がりは実機と同等を再現

使い方:
    ros2 launch susumu_robo robo_indoor_sim.launch.py
    ros2 launch susumu_robo robo_indoor_sim.launch.py world:=empty
    ros2 launch susumu_robo robo_indoor_sim.launch.py use_audio:=true
    ros2 launch susumu_robo robo_indoor_sim.launch.py use_collision_monitor:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    pkg_susumu = FindPackageShare('susumu_robo')

    world_arg = DeclareLaunchArgument('world', default_value='turtlebot3_world.world')
    use_audio_arg = DeclareLaunchArgument('use_audio', default_value='false')
    use_cm_arg = DeclareLaunchArgument('use_collision_monitor', default_value='true')
    use_foxglove_arg = DeclareLaunchArgument('use_foxglove', default_value='true')

    sim_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_susumu, 'launch', 'sim_robot.launch.py'])
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )

    twist_mux = TimerAction(
        period=4.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_susumu, 'launch', 'twist_mux.launch.py'])
            )
        )],
    )

    collision_monitor = TimerAction(
        period=5.0,
        actions=[GroupAction(
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [pkg_susumu, 'launch', 'collision_monitor_sim.launch.py']
                    )
                )
            )],
            condition=IfCondition(LaunchConfiguration('use_collision_monitor')),
        )],
    )

    foxglove_bridge = TimerAction(
        period=4.0,
        actions=[GroupAction(
            actions=[IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('foxglove_bridge'),
                        'launch', 'foxglove_bridge_launch.xml'
                    )
                ),
                launch_arguments={'port': '8765'}.items(),
            )],
            condition=IfCondition(LaunchConfiguration('use_foxglove')),
        )],
    )

    dummy_gnss = TimerAction(
        period=2.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_susumu, 'launch', 'dummy_navsatfix.launch.py'])
            )
        )],
    )

    audio_option_sim = TimerAction(
        period=6.0,
        actions=[GroupAction(
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [pkg_susumu, 'launch', 'audio_option_sim.launch.py']
                    )
                )
            )],
            condition=IfCondition(LaunchConfiguration('use_audio')),
        )],
    )

    return LaunchDescription([
        world_arg,
        use_audio_arg,
        use_cm_arg,
        use_foxglove_arg,
        sim_robot,
        twist_mux,
        collision_monitor,
        foxglove_bridge,
        dummy_gnss,
        audio_option_sim,
    ])
