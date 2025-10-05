from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node

def generate_launch_description():
    # RealSenseカメラノード
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='',
        parameters=[
            {'enable_depth': True},           # 深度センサー有効
            {'enable_color': True},           # カラーセンサー有効
            {'enable_infra1': False},         # 赤外線1センサー無効
            {'enable_infra2': False},         # 赤外線2センサー無効
            {'enable_imu': False},            # IMU（加速度・ジャイロ）無効
            {'enable_accel': False},          # 加速度センサー無効
            {'enable_gyro': False},           # ジャイロセンサー無効
            {'pointcloud.enable': True},      # ポイントクラウド生成有効
            {'align_depth.enable': True},     # 深度画像をカラー画像に整列
        ],
        output='screen',
    )

    # Static Transform Publisher: ベースフレームとカメラのTF補完
    tf_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_camera_tf_publisher',
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "camera_link"]
    )

    return LaunchDescription([
        realsense_node,
        tf_camera_link,
    ])

if __name__ == '__main__':
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()
