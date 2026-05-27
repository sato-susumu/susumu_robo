from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node

def generate_launch_description():
    # RealSenseカメラノード
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='depth_camera',
        parameters=[
            {'enable_depth': True},           # 深度センサー有効
            {'enable_color': True},           # カラーセンサー有効
            {'rgb_camera.profile': '1280x720x15'},   # CPU負荷削減のため30FPSから15FPSに変更
            {'depth_module.profile': '848x480x15'},  # CPU負荷削減のため30FPSから15FPSに変更
            {'enable_infra1': False},         # 赤外線1センサー無効
            {'enable_infra2': False},         # 赤外線2センサー無効
            {'enable_imu': False},            # IMU（加速度・ジャイロ）無効
            {'enable_accel': False},          # 加速度センサー無効
            {'enable_gyro': False},           # ジャイロセンサー無効
            {'pointcloud.enable': False},     # ポイントクラウド生成無効
            {'align_depth.enable': True},     # 深度画像をカラー画像に整列
        ],
        remappings=[
            # aligned_depth_to_color で代替するため不要
            ('/depth_camera/depth/image_rect_raw', '/depth_camera/_hidden/depth/image_rect_raw'),
            # image_rect_raw を隠したため付随する圧縮版も非表示
            ('/depth_camera/depth/image_rect_raw/compressed', '/depth_camera/_hidden/depth/image_rect_raw/compressed'),
            # image_rect_raw を隠したため付随する圧縮版も非表示
            ('/depth_camera/depth/image_rect_raw/compressedDepth', '/depth_camera/_hidden/depth/image_rect_raw/compressedDepth'),
            # image_rect_raw を隠したため付随する圧縮版も非表示
            ('/depth_camera/depth/image_rect_raw/theora', '/depth_camera/_hidden/depth/image_rect_raw/theora'),
            # depth/image_rect_raw を隠したため付随するカメラ情報も非表示
            ('/depth_camera/depth/camera_info', '/depth_camera/_hidden/depth/camera_info'),
            # depth/image_rect_raw を隠したため付随するメタデータも非表示
            ('/depth_camera/depth/metadata', '/depth_camera/_hidden/depth/metadata'),
            # 16bit深度はJPEG非対応のためデータが空になる異常トピック
            ('/depth_camera/aligned_depth_to_color/image_raw/compressed', '/depth_camera/_hidden/aligned_depth_to_color/image_raw/compressed'),
            # 深度画像をtheora動画圧縮するのは非推奨
            ('/depth_camera/aligned_depth_to_color/image_raw/theora', '/depth_camera/_hidden/aligned_depth_to_color/image_raw/theora'),
            # ローカル処理では不要、ネットワーク転送が必要な場合のみ有用
            ('/depth_camera/color/image_raw/theora', '/depth_camera/_hidden/color/image_raw/theora'),
            # ローカル処理では圧縮・展開の二重CPU負荷が発生するため不要
            ('/depth_camera/color/image_raw/compressed', '/depth_camera/_hidden/color/image_raw/compressed'),
            # カラー画像にcompressedDepthは不適切（深度専用フォーマット）
            ('/depth_camera/color/image_raw/compressedDepth', '/depth_camera/_hidden/color/image_raw/compressedDepth'),
            # ローカル処理では圧縮・展開の二重CPU負荷が発生するため不要
            ('/depth_camera/aligned_depth_to_color/image_raw/compressedDepth', '/depth_camera/_hidden/aligned_depth_to_color/image_raw/compressedDepth'),
            # IMUは無効化（enable_imu: False）のためデータが流れない
            ('imu', '_hidden/imu'),
            # 深度センサーから自分自身への変換（単位行列）で実用的な用途がない
            ('extrinsics/depth_to_depth', '_hidden/extrinsics/depth_to_depth'),
            # align_depth.enable: True の場合は内部処理で完結するため外部から不要
            ('extrinsics/depth_to_color', '_hidden/extrinsics/depth_to_color'),
        ],
        output='screen',
    )

    # Static Transform Publisher: ベースフレームとカメラのTF補完
    tf_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_camera_tf_publisher',
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "base_link", "--child-frame-id", "camera_link"]
    )

    return LaunchDescription([
        realsense_node,
        tf_camera_link,
    ])

if __name__ == '__main__':
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()
