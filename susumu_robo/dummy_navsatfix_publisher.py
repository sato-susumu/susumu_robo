#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header

class DummyNavSatFixPublisher(Node):
    def __init__(self):
        super().__init__('dummy_navsatfix_publisher')
        self.publisher = self.create_publisher(NavSatFix, '/navsatfix', 10)
        self.timer = self.create_timer(5.0, self.publish_navsatfix)  # 1Hz
        self.check_timer = self.create_timer(5.0, self.check_real_gnss)  # 2秒毎にチェック
        self.get_logger().info('Dummy NavSatFix publisher started')

    def check_real_gnss(self):
        # 他のNavSatFixパブリッシャーが存在するかチェック
        topic_info = self.get_topic_names_and_types()
        publishers = self.get_publishers_info_by_topic('/navsatfix')

        # 自分以外のパブリッシャーが存在する場合は終了
        other_publishers = [pub for pub in publishers if pub.node_name != self.get_name()]
        if other_publishers:
            self.get_logger().info(f'Real GNSS node detected: {other_publishers[0].node_name}. Shutting down dummy publisher.')
            rclpy.shutdown()

    def publish_navsatfix(self):
        msg = NavSatFix()

        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gnss'

        # 大阪駅の座標（概算）
        msg.latitude = 34.702485
        msg.longitude = 135.495951
        msg.altitude = 20.0  # 海抜20m程度

        # ステータス
        msg.status.status = 0  # STATUS_FIX
        msg.status.service = 1  # SERVICE_GPS (NavSatFixではGPS定数のみ定義)

        # 精度情報
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        msg.position_covariance = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DummyNavSatFixPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()