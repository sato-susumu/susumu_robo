import sys
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import copy as copy_module

# 定数の定義
STOP_THRESHOLD = 0.25  # 停止しきい値 [m]
SLOWDOWN_THRESHOLD = 0.6  # 速度制限しきい値 [m]
SLOWDOWN_SPEED = 0.1  # 速度制限 [m/s]

# センサーIDのリスト
front_sensor_ids = [0, 1]  # 前方のセンサーID
rear_sensor_ids = [2]  # 後方のセンサーID


class UltrasonicMonitorNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_monitor')
        all_sensor_ids = front_sensor_ids + rear_sensor_ids
        self.range_list = [0.0] * (max(all_sensor_ids) + 1)
        self.pub = self.create_publisher(Twist, "/cmd_vel_ddd", 10)
        self.sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        # センサーの購読設定
        for sensor_id in all_sensor_ids:
            self.create_subscription(Range, f"/ultrasonic{sensor_id}", self.create_ultrasonic_callback(sensor_id), 10)

        self.get_logger().info("UltrasonicMonitorNode start")

    def cmd_vel_callback(self, sub_msg: Twist):
        pub_msg = copy_module.deepcopy(sub_msg)
        self.get_logger().info(f"zyusin速度: {pub_msg.linear.x}, {pub_msg.angular.z}")
        self.handle_forward_obstacle_detection(pub_msg)
        self.handle_backward_obstacle_detection(pub_msg)
        self.get_logger().info(f"速度kekka: {pub_msg.linear.x}, {pub_msg.angular.z}")
        self.pub.publish(pub_msg)

    def handle_forward_obstacle_detection(self, pub_msg):
        # 前方の障害物検知
        if pub_msg.linear.x <= 0:
            return
        for sensor_id in front_sensor_ids:
            if self.range_list[sensor_id] < STOP_THRESHOLD:
                pub_msg.linear.x = 0.0
                self.get_logger().info(f"障害物を検知しました. 停止します. センサー: {sensor_id}")
                return  # 1つでも障害物を検知したら、直ちに処理を終了
            elif self.range_list[sensor_id] < SLOWDOWN_THRESHOLD and pub_msg.linear.x > SLOWDOWN_SPEED:
                pub_msg.linear.x = SLOWDOWN_SPEED
                self.get_logger().info(f"障害物を検知しました. 速度を制限します. センサー: {sensor_id}")

    def handle_backward_obstacle_detection(self, pub_msg):
        # 後方の障害物検知
        if pub_msg.linear.x >= 0:
            return
        for sensor_id in rear_sensor_ids:
            if self.range_list[sensor_id] < STOP_THRESHOLD:
                pub_msg.linear.x = 0.0
                self.get_logger().info(f"障害物を検知しました. 停止します. センサー: {sensor_id}")
                return  # 1つでも障害物を検知したら、直ちに処理を終了
            elif self.range_list[sensor_id] < SLOWDOWN_THRESHOLD and pub_msg.linear.x < -SLOWDOWN_SPEED:
                pub_msg.linear.x = -SLOWDOWN_SPEED
                self.get_logger().info(f"障害物を検知しました. 速度を制限します. センサー: {sensor_id}")

    def create_ultrasonic_callback(self, index):
        def callback(sub_msg: Range):
            # if index <= 1:
            #     self.get_logger().info(f"センサー{index}の距離: {sub_msg.range}")
            self.range_list[index] = sub_msg.range

        return callback


def main():
    rclpy.init()
    node = UltrasonicMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()
