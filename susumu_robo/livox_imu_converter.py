#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class LivoxIMUConverter(Node):
    """
    Livox IMUの加速度データをG単位からm/s²に変換するノード
    Livox Mid-360のIMUはデフォルトでG単位で出力されるため、
    ROS2標準のm/s²に変換する必要がある
    """

    def __init__(self):
        super().__init__('livox_imu_converter')

        # 重力定数
        self.GRAVITY = 9.81  # m/s²

        # Subscriber: 元のLivox IMUトピック
        self.subscription = self.create_subscription(
            Imu,
            '/livox/imu',
            self.imu_callback,
            10
        )

        # Publisher: 変換後のIMUトピック
        self.publisher = self.create_publisher(
            Imu,
            '/livox/imu_ms2',
            10
        )

        self.get_logger().info('Livox IMU Converter started')
        self.get_logger().info('Converting from /livox/imu (G) to /livox/imu_ms2 (m/s²)')

    def imu_callback(self, msg):
        """
        IMUメッセージを受信して単位変換を行う
        """
        # 新しいメッセージを作成（元のメッセージをコピー）
        converted_msg = Imu()

        # ヘッダーをコピー
        converted_msg.header = msg.header

        # 姿勢データをそのままコピー（変換不要）
        converted_msg.orientation = msg.orientation
        converted_msg.orientation_covariance = msg.orientation_covariance

        # 角速度をそのままコピー（変換不要、rad/sで正しい）
        converted_msg.angular_velocity = msg.angular_velocity
        converted_msg.angular_velocity_covariance = msg.angular_velocity_covariance

        # 加速度をG単位からm/s²に変換
        converted_msg.linear_acceleration.x = msg.linear_acceleration.x * self.GRAVITY
        converted_msg.linear_acceleration.y = msg.linear_acceleration.y * self.GRAVITY
        converted_msg.linear_acceleration.z = msg.linear_acceleration.z * self.GRAVITY
        converted_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance

        # 変換後のメッセージをパブリッシュ
        self.publisher.publish(converted_msg)

def main(args=None):
    rclpy.init(args=args)

    livox_imu_converter = LivoxIMUConverter()

    try:
        rclpy.spin(livox_imu_converter)
    except KeyboardInterrupt:
        pass
    finally:
        livox_imu_converter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()