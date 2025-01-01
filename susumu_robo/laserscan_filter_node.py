# このソースコードは laserscan_filter_node.cpp をPythonに移植し、改変したものです。
#
# オリジナルのソースコードとライセンス
#   https://github.com/whill-labs/ros2_whill_applications/blob/main/whill_auto_stop/src/laserscan_filter_node.cpp
#   Copyright (c) 2024 WHILL, Inc.
#   Released under the MIT license
#   https://opensource.org/licenses/mit-license.php

# 関連パッケージ
# sudo apt install -y ros-humble-tf-transformations ros-humble-tf2-tools  ros-humble-tf2-ros
# pip install transforms3d

import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PolygonStamped, Point32
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf_transformations import euler_from_quaternion

from rclpy.qos import QoSProfile, ReliabilityPolicy


class LaserScanFilterNode(Node):
    def __init__(self):
        super().__init__('laserscan_filter_node')

        # パラメータ宣言 (デフォルト値をセット)
        self.declare_parameter('reference_link', 'livox_frame')
        self.declare_parameter('forward.x_min', -0.22)
        self.declare_parameter('forward.x_max',  0.22)
        self.declare_parameter('forward.y_min', -0.22)
        self.declare_parameter('forward.y_max',  0.22)
        self.declare_parameter('backward.x_min', -0.22)
        self.declare_parameter('backward.x_max',  0.22)
        self.declare_parameter('backward.y_min', -0.22)
        self.declare_parameter('backward.y_max',  0.22)

        # パラメータ取得
        self.reference_link_ = self.get_parameter('reference_link').value
        self.forward_x_min_ = self.get_parameter('forward.x_min').value
        self.forward_x_max_ = self.get_parameter('forward.x_max').value
        self.forward_y_min_ = self.get_parameter('forward.y_min').value
        self.forward_y_max_ = self.get_parameter('forward.y_max').value
        self.backward_x_min_ = self.get_parameter('backward.x_min').value
        self.backward_x_max_ = self.get_parameter('backward.x_max').value
        self.backward_y_min_ = self.get_parameter('backward.y_min').value
        self.backward_y_max_ = self.get_parameter('backward.y_max').value

        # 変数初期化
        self.is_forward_ = True
        self.linear_x_ = 0.0

        # TFバッファとリスナーのセットアップ
        self.tf_buffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_, self)

        # QoS設定（ベストエフォート）
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscriber
        self.laser_scan_subscriber_ = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_scan_callback,
            qos_profile
        )
        self.velocity_subscriber_ = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_callback,
            qos_profile
        )

        # Publisher
        self.bool_publisher_ = self.create_publisher(
            Bool,
            'scan_in_range',
            qos_profile
        )
        self.detected_points_publisher_ = self.create_publisher(
            LaserScan,
            'detected_points',
            qos_profile
        )
        self.polygon_publisher_ = self.create_publisher(
            PolygonStamped,
            'scan_range_polygon',
            qos_profile
        )

        self.get_logger().info("Laser Scan Filter Node has been started.")
        self.get_logger().info(f"Reference link is set to: {self.reference_link_}")

    def velocity_callback(self, msg: Twist):
        """
        cmd_velの速度を受け取り、前進・後進のどちらかを判定する.
        """
        self.linear_x_ = msg.linear.x
        self.is_forward_ = (self.linear_x_ >= 0.0)
        self.get_logger().debug(f"Direction: {'Forward' if self.is_forward_ else 'Backward'}")

    def laser_scan_callback(self, msg: LaserScan):
        """
        LaserScanを受け取り、指定領域内に点があるかどうかを判定する.
        """
        in_range = False

        # 進行方向に応じて範囲を切り替える
        if self.is_forward_:
            x_min = self.forward_x_min_
            x_max = self.forward_x_max_ + self.linear_x_
            y_min = self.forward_y_min_
            y_max = self.forward_y_max_
        else:
            x_min = self.backward_x_min_ + self.linear_x_
            x_max = self.backward_x_max_
            y_min = self.backward_y_min_
            y_max = self.backward_y_max_

        # 可視化用にPolygonをPublish
        self.publish_polygon(x_min, x_max, y_min, y_max)

        # TFを取得
        try:
            transform_stamped = self.tf_buffer_.lookup_transform(
                self.reference_link_,
                msg.header.frame_id,
                rclpy.time.Time())  # 最新の変換を取得
        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            self.get_logger().warn(
                f"Could not transform {msg.header.frame_id} to {self.reference_link_}: {ex}")
            return

        # LaserScanのコピーを作る (点検出用)
        detected_points_msg = LaserScan()
        detected_points_msg.header = msg.header
        detected_points_msg.angle_min = msg.angle_min
        detected_points_msg.angle_max = msg.angle_max
        detected_points_msg.angle_increment = msg.angle_increment
        detected_points_msg.time_increment = msg.time_increment
        detected_points_msg.scan_time = msg.scan_time
        detected_points_msg.range_min = msg.range_min
        detected_points_msg.range_max = msg.range_max

        detected_points_msg.ranges = []
        detected_points_msg.intensities = []

        # 四元数からyaw角を取得
        yaw = self.get_yaw_from_quaternion(transform_stamped.transform.rotation)

        # `msg.intensities`のサイズを確認
        has_intensities = len(msg.intensities) == len(msg.ranges)

        # レーザースキャンの各点についてチェック
        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r):
                # 無効値はそのままNaNを入れる
                detected_points_msg.ranges.append(float('nan'))
                detected_points_msg.intensities.append(0.0)  # 常に0.0を追加
                continue

            angle = msg.angle_min + i * msg.angle_increment
            x_local = r * math.cos(angle)
            y_local = r * math.sin(angle)

            # TFに基づきreference_link座標系へ変換
            x_ref = transform_stamped.transform.translation.x + \
                    x_local * math.cos(yaw) - y_local * math.sin(yaw)
            y_ref = transform_stamped.transform.translation.y + \
                    x_local * math.sin(yaw) + y_local * math.cos(yaw)

            # 範囲内かどうか判定
            if x_min <= x_ref <= x_max and y_min <= y_ref <= y_max:
                in_range = True
                detected_points_msg.ranges.append(r)
                if has_intensities:
                    detected_points_msg.intensities.append(msg.intensities[i])
                else:
                    detected_points_msg.intensities.append(0.0)  # デフォルト値を追加
            else:
                detected_points_msg.ranges.append(float('nan'))
                detected_points_msg.intensities.append(0.0)  # 常に0.0を追加

        # in_range情報をBoolでPublish
        bool_msg = Bool()
        bool_msg.data = in_range
        self.bool_publisher_.publish(bool_msg)

        # 検出した点群をPublish
        self.detected_points_publisher_.publish(detected_points_msg)

        self.get_logger().info(f"Scan in range: {in_range}")

    def get_yaw_from_quaternion(self, quaternion):
        """
        geometry_msgs/msg/Quaternion からyaw角を取得する.
        """
        q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        _, _, yaw = euler_from_quaternion(q)
        return yaw

    def publish_polygon(self, x_min, x_max, y_min, y_max):
        """
        可視化用の矩形ポリゴンをpublishする.
        """
        polygon_msg = PolygonStamped()
        polygon_msg.header.frame_id = self.reference_link_
        polygon_msg.header.stamp = self.get_clock().now().to_msg()

        # 矩形の四隅を作成
        p1 = Point32(x=float(x_min), y=float(y_min), z=0.0)
        p2 = Point32(x=float(x_max), y=float(y_min), z=0.0)
        p3 = Point32(x=float(x_max), y=float(y_max), z=0.0)
        p4 = Point32(x=float(x_min), y=float(y_max), z=0.0)

        polygon_msg.polygon.points.append(p1)
        polygon_msg.polygon.points.append(p2)
        polygon_msg.polygon.points.append(p3)
        polygon_msg.polygon.points.append(p4)
        # 矩形を閉じるために始点に戻す
        polygon_msg.polygon.points.append(p1)

        self.polygon_publisher_.publish(polygon_msg)
        self.get_logger().debug("Published scan range polygon.")


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
