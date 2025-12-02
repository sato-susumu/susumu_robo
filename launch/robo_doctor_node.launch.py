#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    robo_doctor_node = Node(
        package='susumu_robo',
        executable='robo_doctor_node',
        name='robo_doctor_node',
        output='screen',
        parameters=[]
    )

    return LaunchDescription([
        robo_doctor_node
    ])
