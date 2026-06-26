"""Gazebo Classic 上で susumu_diffbot を起動する.

- Gazebo (gazebo_ros) + world (TurtleBot3 同梱 world をデフォルト)
- robot_state_publisher (xacro を use_gazebo:=true で展開)
- spawn_entity でロボットを配置
- joint_state_broadcaster + DiffDriveController (gazebo_ros2_control 経由)

これにより `/odom` `/scan` `/imu/data` `/depth_camera/...` と、cmd_vel 入力 I/F が揃う.

使い方:
    ros2 launch susumu_robo sim_robot.launch.py
    ros2 launch susumu_robo sim_robot.launch.py world:=empty
    ros2 launch susumu_robo sim_robot.launch.py world:=/abs/path/to/foo.world
    ros2 launch susumu_robo sim_robot.launch.py x:=1.0 y:=0.5 yaw:=1.57
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _resolve_world_path(world):
    if not world:
        return ''
    if os.path.isabs(world) and os.path.isfile(world):
        return world

    try:
        gz_share = get_package_share_directory('gazebo_ros')
    except Exception:
        gz_share = None

    if world == 'empty':
        if gz_share:
            path = os.path.join(gz_share, 'worlds', 'empty.world')
            if os.path.isfile(path):
                return path
        return ''

    try:
        tb3_share = get_package_share_directory('turtlebot3_gazebo')
        tb3_worlds = os.path.join(tb3_share, 'worlds')
    except Exception:
        tb3_worlds = None

    candidates = []
    if tb3_worlds:
        candidates.append(os.path.join(tb3_worlds, world))
        if not world.endswith('.world'):
            candidates.append(os.path.join(tb3_worlds, f'{world}.world'))
    for c in candidates:
        if os.path.isfile(c):
            return c

    print(f'[sim_robot] world "{world}" not found, falling back to empty world.')
    return ''


def _launch_gazebo(context, *args, **kwargs):
    pkg_gazebo = get_package_share_directory('gazebo_ros')
    world_arg_value = LaunchConfiguration('world').perform(context)
    world_path = _resolve_world_path(world_arg_value)
    launch_args = {
        'verbose': 'false',
        'gui': LaunchConfiguration('gui').perform(context),
        'server': 'true',
    }
    if world_path:
        launch_args['world'] = world_path
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments=launch_args.items(),
        )
    ]


def generate_launch_description():
    pkg_susumu = FindPackageShare('susumu_robo')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='turtlebot3_world.world',
        description='Gazebo world. 絶対パス, turtlebot3_gazebo/worlds/ 下のファイル名, または "empty"',
    )
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.1')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Gazebo client (GUI). false で gzserver のみ起動',
    )

    xacro_path = PathJoinSubstitution(
        [pkg_susumu, 'susumu_diffbot/urdf', 'diffbot.urdf.xacro']
    )

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        xacro_path,
        ' use_gazebo:=true',
    ])
    robot_description = {
        'robot_description': robot_description_content,
        'use_sim_time': LaunchConfiguration('use_sim_time'),
    }

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'susumu_diffbot',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
        output='screen',
    )

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    diff_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['botwheel_explorer', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    delayed_diff_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[diff_spawner],
        )
    )

    relay_odom = Node(
        package='topic_tools',
        executable='relay',
        name='odom_topic_relay',
        arguments=['/botwheel_explorer/odom', '/odom'],
        output='log',
    )

    return LaunchDescription([
        world_arg,
        x_arg, y_arg, z_arg, yaw_arg,
        use_sim_time_arg,
        gui_arg,
        OpaqueFunction(function=_launch_gazebo),
        robot_state_pub,
        spawn_entity,
        jsb_spawner,
        delayed_diff_spawner,
        relay_odom,
    ])
