import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='susumu_gtts',
            executable='susumu_gtts',
            name='susumu_gtts_node',
            output='screen'
        )
    ])
