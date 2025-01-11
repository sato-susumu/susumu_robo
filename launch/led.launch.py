from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='susumu_robo',
            executable='led_controller_node',
            name='led_controller_node',
            output='screen'
        ),
        Node(
            package='susumu_blinkstick_ros2',
            executable='blinkstick_node',
            name='blinkstick_node',
            output='screen',
            parameters=[
                {'brightness': 0.3}
            ]
        ),
    ])

if __name__ == '__main__':
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()
