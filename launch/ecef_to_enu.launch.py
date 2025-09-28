#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    ECEFからENUへの座標変換を行うstatic transform publisherを起動
    原点：大阪駅（WGS84） lat=34.701889°, lon=135.494972°, h=0 m（楕円体高）
    """

    # ECEF平行移動パラメータ [tx ty tz]（メートル）
    tx = '-3743760.863972'
    ty = '3679629.408384'
    tz = '3610726.856717'

    # ECEF→ENU（map）の回転クォータニオン [qx qy qz qw]
    qx = '-0.179436228762'
    qy = '0.427959490492'
    qz = '0.816906727234'
    qw = '0.342515274558'

    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ecef_to_enu_transform',
        output='screen',
        arguments=[
            tx, ty, tz,  # Translation
            qx, qy, qz, qw,  # Quaternion rotation
            'earth',  # Parent frame
            'map'  # Child frame
        ]
    )

    return LaunchDescription([
        static_transform_publisher
    ])