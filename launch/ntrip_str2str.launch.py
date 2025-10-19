from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'str2str',
                '-in', 'ntrip://geortk.jp:2101/KOBEKOKA',
                '-out', 'tcpcli://192.168.3.1:28785'
            ],
            output='screen',
        ),
    ])
