#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header

class DummyNavSatFixPublisher(Node):
    def __init__(self):
        super().__init__('dummy_navsatfix_publisher')
        self.publisher = self.create_publisher(NavSatFix, '/fix', 10)
        self.timer = self.create_timer(5.0, self.publish_navsatfix)  # 1Hz
        self.check_timer = self.create_timer(5.0, self.check_real_gnss)  # 2秒毎にチェック
        self.get_logger().info('Dummy NavSatFix publisher started')

    def check_real_gnss(self):
        # 他のNavSatFixパブリッシャーが存在するかチェック
        topic_info = self.get_topic_names_and_types()
        publishers = self.get_publishers_info_by_topic('/fix')

        # 自分以外のパブリッシャーが存在する場合は終了
        other_publishers = [pub for pub in publishers if pub.node_name != self.get_name()]
        if other_publishers:
            self.get_logger().info(f'Real GNSS node detected: {other_publishers[0].node_name}. Shutting down dummy publisher.')
            raise SystemExit

    def publish_navsatfix(self):
        msg = NavSatFix()

        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gnss'

        # 無効な座標 (NaN)
        msg.latitude = float('nan')
        msg.longitude = float('nan')
        msg.altitude = float('nan')

        # ステータス: 位置情報なし
        msg.status.status = -1  # STATUS_NO_FIX
        msg.status.service = 1  # SERVICE_GPS

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
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()