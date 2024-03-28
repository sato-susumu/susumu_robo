import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
import copy

class LaserScanModifier(Node):
    def __init__(self):
        super().__init__('laser_scan_modifier')
        self.display_percentage = 0  # 表示パーセンテージの初期値

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile)
        self.publisher = self.create_publisher(LaserScan, 'modified_scan', qos_profile)

    def scan_callback(self, msg):
        modified_scan = copy.deepcopy(msg)
        num_ranges = len(msg.ranges)
        num_to_display = int(num_ranges * (self.display_percentage / 100.0))  # 表示する範囲データの数

        # 先頭からnum_to_displayまでを表示し、残りを無限大に設定
        for i in range(num_to_display, num_ranges):
            modified_scan.ranges[i] = float('inf')

        self.publisher.publish(modified_scan)

        # パーセンテージを更新
        self.display_percentage += 10
        if self.display_percentage > 100:
            self.display_percentage = 0


def main(args=None):
    rclpy.init(args=args)
    laser_scan_modifier = LaserScanModifier()
    rclpy.spin(laser_scan_modifier)
    laser_scan_modifier.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
