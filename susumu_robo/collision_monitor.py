import sys
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import copy as copy_module
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan
import math
from typing import List, Tuple

# 定数の定義
STOP_THRESHOLD = 0.25  # 停止しきい値 [m]
SLOWDOWN_THRESHOLD = 0.6  # 速度制限しきい値 [m]
SLOWDOWN_SPEED = 0.1  # 速度制限 [m/s]

# センサーIDのリスト
front_sensor_ids = [0, 1]  # 前方のセンサーID
rear_sensor_ids = [2]  # 後方のセンサーID


class CollisionMonitorNode(Node):
    def __init__(self, debug_mode=False):
        super().__init__('collision_monitor')
        self.debug_mode = debug_mode
        all_sensor_ids = front_sensor_ids + rear_sensor_ids
        self.range_list = [100.0] * (max(all_sensor_ids) + 1)
        self.front_objects = []
        self.back_objects = []
        self.nearby_objects = []
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=5)

        self.pub = self.create_publisher(Twist, "~/output_velocity", qos)
        self.sub = self.create_subscription(Twist, "~/input_velocity", self.cmd_vel_callback, qos)

        # センサーの購読設定
        for sensor_id in all_sensor_ids:
            self.create_subscription(Range, f"/ultrasonic{sensor_id}", self.create_ultrasonic_callback(sensor_id), qos)

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos)

        self.get_logger().info("CollisionMonitorNode start")

    def log_debug(self, msg):
        if self.debug_mode:
            self.get_logger().debug(msg)

    def scan_callback(self, msg: LaserScan) -> None:
        # 判定結果を取得
        self.front_objects = self.detect_front_objects(msg)
        self.back_objects = self.detect_back_objects(msg)
        self.nearby_objects = self.detect_nearby_objects(msg)

        # # ログ出力
        # if self.front_objects:
        #     self.get_logger().info(f"Front objects: {self.front_objects}")
        # if self.back_objects:
        #     self.get_logger().info(f"Back objects: {self.back_objects}")
        # if self.nearby_objects:
        #     self.get_logger().info(f"Nearby objects: {self.nearby_objects}")

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

    def cmd_vel_callback(self, sub_msg: Twist):
        pub_msg = copy_module.deepcopy(sub_msg)
        self.handle_forward_obstacle_detection(pub_msg)
        self.handle_backward_obstacle_detection(pub_msg)
        self.handle_near_obstacle_detection(pub_msg)
        self.log_debug(
            f"input: {sub_msg.linear.x}, {sub_msg.angular.z} output: {pub_msg.linear.x}, {pub_msg.angular.z}")
        self.pub.publish(pub_msg)

    def handle_forward_obstacle_detection(self, pub_msg):
        # 前方移動以外は処理しない
        if pub_msg.linear.x <= 0:
            return

        if len(self.front_objects) > 5:
            pub_msg.linear.x = 0.0
            self.log_debug(f"障害物を検知しました. 停止します. scan点数:{len(self.front_objects)}")
            return

        for sensor_id in front_sensor_ids:
            if self.range_list[sensor_id] < STOP_THRESHOLD:
                pub_msg.linear.x = 0.0
                self.log_debug(
                    f"障害物を検知しました. 停止します. センサーid:{sensor_id} range:{self.range_list[sensor_id]}")
                return  # 1つでも障害物を検知したら、直ちに処理を終了
            elif self.range_list[sensor_id] < SLOWDOWN_THRESHOLD and pub_msg.linear.x > SLOWDOWN_SPEED:
                pub_msg.linear.x = SLOWDOWN_SPEED
                self.log_debug(
                    f"障害物を検知しました. 速度を制限します. センサーid:{sensor_id} range:{self.range_list[sensor_id]}")

    def handle_backward_obstacle_detection(self, pub_msg):
        # 後方移動以外は処理しない
        if pub_msg.linear.x >= 0:
            return

        if len(self.back_objects) > 5:
            pub_msg.linear.x = 0.0
            self.log_debug(f"障害物を検知しました. 停止します. scan点数:{len(self.back_objects)}")
            return

        for sensor_id in rear_sensor_ids:
            if self.range_list[sensor_id] < STOP_THRESHOLD:
                pub_msg.linear.x = 0.0
                self.log_debug(
                    f"障害物を検知しました. 停止します. センサーid:{sensor_id} range:{self.range_list[sensor_id]}")
                return  # 1つでも障害物を検知したら、直ちに処理を終了
            elif self.range_list[sensor_id] < SLOWDOWN_THRESHOLD and pub_msg.linear.x < -SLOWDOWN_SPEED:
                pub_msg.linear.x = -SLOWDOWN_SPEED
                self.log_debug(
                    f"障害物を検知しました. 速度を制限します. センサーid:{sensor_id} range:{self.range_list[sensor_id]}")

    def handle_near_obstacle_detection(self, pub_msg):
        # 旋回以外は処理しない
        if pub_msg.angular.z == 0.0:
            return

        if len(self.nearby_objects) > 5:
            pub_msg.angular.z = 0.0
            self.log_debug(f"障害物を検知しました. 停止します. scan点数:{len(self.nearby_objects)}")
            return


    def create_ultrasonic_callback(self, index):
        def callback(sub_msg: Range):
            # if index <= 1:
            self.log_debug(f"センサー{index}の距離: {sub_msg.range}")
            self.range_list[index] = sub_msg.range

        return callback


def main():
    rclpy.init()
    node = CollisionMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()
