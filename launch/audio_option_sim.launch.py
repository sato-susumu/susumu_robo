"""シミュレータ用 オフライン音声スタック.

dummy_audio (ダミー speak action server + to_human ブリッジ)
+ stt_option_debug (WAV 入力)
+ agent_option_debug

を一括起動する。AivisSpeech (Docker) もマイクもスピーカーも不要。

使い方:
    ros2 launch susumu_robo audio_option_sim.launch.py
    ros2 launch susumu_robo audio_option_sim.launch.py use_stt:=false
    ros2 launch susumu_robo audio_option_sim.launch.py input_file:=/path/to/sample.wav
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    susumu_robo_share = FindPackageShare('susumu_robo')

    use_stt_arg = DeclareLaunchArgument(
        'use_stt',
        default_value='false',
        description='true の場合 stt_option_debug を起動 (WAV 入力推奨)',
    )
    use_agent_arg = DeclareLaunchArgument(
        'use_agent',
        default_value='true',
        description='true の場合 agent_option_debug を起動',
    )
    input_file_arg = DeclareLaunchArgument(
        'input_file',
        default_value='',
        description='stt_option_debug に渡す WAV ファイル (空ならマイク)',
    )

    dummy_audio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([susumu_robo_share, 'launch', 'dummy_audio.launch.py'])
        )
    )

    stt = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [susumu_robo_share, 'launch', 'stt_option_debug.launch.py']
                    )
                ),
                launch_arguments={
                    'input_file': LaunchConfiguration('input_file'),
                }.items(),
            ),
        ],
        condition=IfCondition(LaunchConfiguration('use_stt')),
    )

    agent = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [susumu_robo_share, 'launch', 'agent_option_debug.launch.py']
                    )
                ),
            ),
        ],
        condition=IfCondition(LaunchConfiguration('use_agent')),
    )

    return LaunchDescription([
        use_stt_arg,
        use_agent_arg,
        input_file_arg,
        LogInfo(msg='audio_option_sim: オフライン音声スタックを起動します'),
        dummy_audio,
        stt,
        agent,
    ])
