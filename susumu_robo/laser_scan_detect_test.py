import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
import math
from typing import List, Tuple


class LaserScanDetectTest(Node):
    def __init__(self):
        super().__init__('laser_scan_detect_test')

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

    def scan_callback(self, msg: LaserScan) -> None:
        # 判定結果を取得
        front_objects = self.detect_front_objects(msg)
        back_objects = self.detect_back_objects(msg)
        nearby_objects = self.detect_nearby_objects(msg)

        # ログ出力
        if front_objects:
            self.get_logger().info(f"Front objects: {front_objects}")
        if back_objects:
            self.get_logger().info(f"Back objects: {back_objects}")
        if nearby_objects:
            self.get_logger().info(f"Nearby objects: {nearby_objects}")

    def detect_front_objects(self, msg: LaserScan) -> List[Tuple[float, int]]:
        front_objects = []
        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if distance <= 0.5 and abs(angle) <= math.pi / 2:
                front_objects.append((round(distance, 2), int(math.degrees(angle))))
        return front_objects

    def detect_back_objects(self, msg: LaserScan) -> List[Tuple[float, int]]:
        back_objects = []
        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if distance <= 0.5 and abs(angle) > math.pi / 2:
                back_objects.append((round(distance, 2), int(math.degrees(angle))))
        return back_objects

    def detect_nearby_objects(self, msg: LaserScan) -> List[Tuple[float, int]]:
        close_objects = []
        for i, distance in enumerate(msg.ranges):
            if distance <= 0.3:
                angle = msg.angle_min + i * msg.angle_increment
                close_objects.append((round(distance, 2), int(math.degrees(angle))))
        return close_objects

def main(args=None) -> None:
    rclpy.init(args=args)
    laser_scan_modifier = LaserScanDetectTest()
    rclpy.spin(laser_scan_modifier)
    laser_scan_modifier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
