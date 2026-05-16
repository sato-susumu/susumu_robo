import os
import subprocess
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, EmitEvent, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
import launch_ros.events.lifecycle
from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.msg import Transition


def _check_odom_exists():
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True, text=True, timeout=3
        )
        return '/odom' in result.stdout.split()
    except Exception:
        return False


def _launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml = os.path.expanduser('~/map.yaml')

    actions = []

    if _check_odom_exists():
        print('[nav2_dummy_odom] /odom トピックを検出。ダミーodomはスキップします。')
    else:
        print('[nav2_dummy_odom] /odom トピックが見つかりません。ダミーodomを起動します。')
        dummy_odom = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--frame-id', 'odom', '--child-frame-id', 'base_link'],
            parameters=[{'use_sim_time': False}],
        )
        actions.append(dummy_odom)

    # 保存済み地図を配信するmap_server
    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': map_yaml,
        }],
    )
    map_server_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch_ros.events.lifecycle.matches_node_name('/map_server'),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    map_server_activate = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch_ros.events.lifecycle.matches_node_name('/map_server'),
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )

    slam_toolbox_share_dir = get_package_share_directory('slam_toolbox')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            slam_toolbox_share_dir + '/launch/online_async_launch.py'
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    pkg_share = get_package_share_directory('susumu_robo')
    nav2_params = pkg_share + '/config/nav2_params.yaml'

    nav2_bringup_share_dir = get_package_share_directory('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            nav2_bringup_share_dir + '/launch/navigation_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
        }.items(),
    )

    nav2_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            nav2_bringup_share_dir + '/launch/rviz_launch.py'
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    actions += [
        map_server,
        TimerAction(period=1.0, actions=[map_server_configure]),
        TimerAction(period=2.0, actions=[map_server_activate]),
        slam_launch,
        nav2_launch,
        nav2_rviz_launch,
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=_launch_setup),
    ])
