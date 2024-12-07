from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='susumu_robo',
            executable='tenkey_controller',
            name='tenkey_controller',
            output='screen',
            parameters=[{
#                'keyboard_device_path': '/dev/input/by-id/usb-05a4_Tenkey_Keyboard-event-kbd',
#                'cmd_vel_topic': '/turtle1/cmd_vel',
#                'linear_speed': 1.0,
#                'angular_speed': 1.0
            }]
        ),
    ])
