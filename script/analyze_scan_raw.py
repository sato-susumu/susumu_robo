#!/usr/bin/env python3
"""Analyze /scan_raw topic to find near-field noise angles."""
import math
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanAnalyzer(Node):
    def __init__(self):
        super().__init__('scan_analyzer')
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(LaserScan, '/scan_raw', self.callback, qos)
        self.samples = []
        self.max_samples = 30
        self.threshold = 0.5  # meters

    def callback(self, msg):
        hits = []
        for i, r in enumerate(msg.ranges):
            if math.isfinite(r) and r < self.threshold:
                angle_deg = math.degrees(msg.angle_min + i * msg.angle_increment)
                hits.append((angle_deg, r))
        self.samples.append(hits)
        self.get_logger().info(f'Sample {len(self.samples)}/{self.max_samples}: {len(hits)} hits under {self.threshold}m')
        if self.samples:
            for angle, dist in sorted(hits, key=lambda x: x[0]):
                self.get_logger().info(f'  {angle:.1f}deg: {dist:.3f}m')

        if len(self.samples) >= self.max_samples:
            self.report()
            rclpy.shutdown()

    def report(self):
        from collections import defaultdict
        angle_counts = defaultdict(list)
        for sample in self.samples:
            for angle, dist in sample:
                bucket = round(angle / 2) * 2
                angle_counts[bucket].append(dist)

        print('\n=== NOISE REPORT (scan_raw) ===')
        print(f'Threshold: {self.threshold}m, Samples: {len(self.samples)}')
        print(f'{"Angle":>8}  {"Count":>6}  {"MinDist":>8}  {"MaxDist":>8}')
        for angle in sorted(angle_counts.keys()):
            dists = angle_counts[angle]
            print(f'{angle:>8.1f}  {len(dists):>6}  {min(dists):>8.3f}  {max(dists):>8.3f}')
        print('==============================\n')


def main():
    rclpy.init()
    node = ScanAnalyzer()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
