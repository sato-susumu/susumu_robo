#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from collections import deque
import time
import math
import sys

class IMUVisualizer(Node):
    def __init__(self, topic1='/imu', topic2=None):
        super().__init__('imu_visualizer')

        # トピック名設定
        self.topic1 = topic1
        self.topic2 = topic2
        self.dual_mode = topic2 is not None

        # データ保存用（トピック1）
        self.max_samples = 50
        self.timestamps1 = deque(maxlen=self.max_samples)
        self.accel_x1 = deque(maxlen=self.max_samples)
        self.accel_y1 = deque(maxlen=self.max_samples)
        self.accel_z1 = deque(maxlen=self.max_samples)
        self.gyro_x1 = deque(maxlen=self.max_samples)
        self.gyro_y1 = deque(maxlen=self.max_samples)
        self.gyro_z1 = deque(maxlen=self.max_samples)
        self.roll1 = deque(maxlen=self.max_samples)
        self.pitch1 = deque(maxlen=self.max_samples)
        self.yaw1 = deque(maxlen=self.max_samples)

        # データ保存用（トピック2）
        if self.dual_mode:
            self.timestamps2 = deque(maxlen=self.max_samples)
            self.accel_x2 = deque(maxlen=self.max_samples)
            self.accel_y2 = deque(maxlen=self.max_samples)
            self.accel_z2 = deque(maxlen=self.max_samples)
            self.gyro_x2 = deque(maxlen=self.max_samples)
            self.gyro_y2 = deque(maxlen=self.max_samples)
            self.gyro_z2 = deque(maxlen=self.max_samples)
            self.roll2 = deque(maxlen=self.max_samples)
            self.pitch2 = deque(maxlen=self.max_samples)
            self.yaw2 = deque(maxlen=self.max_samples)

        self.start_time = time.time()

        # IMUトピックの購読
        self.subscription1 = self.create_subscription(
            Imu, self.topic1, lambda msg: self.imu_callback(msg, 1), 10)

        if self.dual_mode:
            self.subscription2 = self.create_subscription(
                Imu, self.topic2, lambda msg: self.imu_callback(msg, 2), 10)

        # プロット初期化
        plt.ion()  # インタラクティブモード
        self.fig, self.axes = plt.subplots(3, 1, figsize=(10, 8))

        if self.dual_mode:
            self.fig.suptitle(f'IMU Dual Visualization: {self.topic1} vs {self.topic2}')
        else:
            self.fig.suptitle(f'IMU Data Visualization: {self.topic1}')

        # 加速度プロット
        self.axes[0].set_ylabel('Acceleration (m/s²)')
        self.axes[0].grid(True)
        self.line_acc_x1, = self.axes[0].plot([], [], 'r-', label=f'X ({self.topic1})' if self.dual_mode else 'X')
        self.line_acc_y1, = self.axes[0].plot([], [], 'g-', label=f'Y ({self.topic1})' if self.dual_mode else 'Y')
        self.line_acc_z1, = self.axes[0].plot([], [], 'b-', label=f'Z ({self.topic1})' if self.dual_mode else 'Z')

        if self.dual_mode:
            self.line_acc_x2, = self.axes[0].plot([], [], 'r--', label=f'X ({self.topic2})', alpha=0.7)
            self.line_acc_y2, = self.axes[0].plot([], [], 'g--', label=f'Y ({self.topic2})', alpha=0.7)
            self.line_acc_z2, = self.axes[0].plot([], [], 'b--', label=f'Z ({self.topic2})', alpha=0.7)
        self.axes[0].legend(loc='upper right', fontsize=8)

        # 角速度プロット
        self.axes[1].set_ylabel('Angular Velocity (rad/s)')
        self.axes[1].grid(True)
        self.line_gyro_x1, = self.axes[1].plot([], [], 'r-', label=f'X ({self.topic1})' if self.dual_mode else 'X')
        self.line_gyro_y1, = self.axes[1].plot([], [], 'g-', label=f'Y ({self.topic1})' if self.dual_mode else 'Y')
        self.line_gyro_z1, = self.axes[1].plot([], [], 'b-', label=f'Z ({self.topic1})' if self.dual_mode else 'Z')

        if self.dual_mode:
            self.line_gyro_x2, = self.axes[1].plot([], [], 'r--', label=f'X ({self.topic2})', alpha=0.7)
            self.line_gyro_y2, = self.axes[1].plot([], [], 'g--', label=f'Y ({self.topic2})', alpha=0.7)
            self.line_gyro_z2, = self.axes[1].plot([], [], 'b--', label=f'Z ({self.topic2})', alpha=0.7)
        self.axes[1].legend(loc='upper right', fontsize=8)

        # 姿勢プロット（オイラー角）
        self.axes[2].set_ylabel('Orientation (degrees)')
        self.axes[2].set_xlabel('Time (s)')
        self.axes[2].grid(True)
        self.line_roll1, = self.axes[2].plot([], [], 'r-', label=f'Roll ({self.topic1})' if self.dual_mode else 'Roll')
        self.line_pitch1, = self.axes[2].plot([], [], 'g-', label=f'Pitch ({self.topic1})' if self.dual_mode else 'Pitch')
        self.line_yaw1, = self.axes[2].plot([], [], 'b-', label=f'Yaw ({self.topic1})' if self.dual_mode else 'Yaw')

        if self.dual_mode:
            self.line_roll2, = self.axes[2].plot([], [], 'r--', label=f'Roll ({self.topic2})', alpha=0.7)
            self.line_pitch2, = self.axes[2].plot([], [], 'g--', label=f'Pitch ({self.topic2})', alpha=0.7)
            self.line_yaw2, = self.axes[2].plot([], [], 'b--', label=f'Yaw ({self.topic2})', alpha=0.7)
        self.axes[2].legend(loc='upper right', fontsize=8)

        self.get_logger().info(f'IMU Visualizer started - Mode: {"Dual" if self.dual_mode else "Single"}')
        self.get_logger().info(f'Subscribing to: {self.topic1}' + (f' and {self.topic2}' if self.dual_mode else ''))

    def quaternion_to_euler(self, x, y, z, w):
        """クォータニオンをオイラー角に変換"""
        # Roll (x軸周り)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y軸周り)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z軸周り)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # ラジアンから度に変換
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    def imu_callback(self, msg, source):
        current_time = time.time() - self.start_time

        if source == 1:
            self.timestamps1.append(current_time)
            # データ保存
            self.accel_x1.append(msg.linear_acceleration.x)
            self.accel_y1.append(msg.linear_acceleration.y)
            self.accel_z1.append(msg.linear_acceleration.z)
            self.gyro_x1.append(msg.angular_velocity.x)
            self.gyro_y1.append(msg.angular_velocity.y)
            self.gyro_z1.append(msg.angular_velocity.z)

            # クォータニオンをオイラー角に変換
            roll_deg, pitch_deg, yaw_deg = self.quaternion_to_euler(
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            )
            self.roll1.append(roll_deg)
            self.pitch1.append(pitch_deg)
            self.yaw1.append(yaw_deg)
        else:
            self.timestamps2.append(current_time)
            # データ保存
            self.accel_x2.append(msg.linear_acceleration.x)
            self.accel_y2.append(msg.linear_acceleration.y)
            self.accel_z2.append(msg.linear_acceleration.z)
            self.gyro_x2.append(msg.angular_velocity.x)
            self.gyro_y2.append(msg.angular_velocity.y)
            self.gyro_z2.append(msg.angular_velocity.z)

            # クォータニオンをオイラー角に変換
            roll_deg, pitch_deg, yaw_deg = self.quaternion_to_euler(
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            )
            self.roll2.append(roll_deg)
            self.pitch2.append(pitch_deg)
            self.yaw2.append(yaw_deg)

        # グラフ更新
        self.update_plot()

    def update_plot(self):
        # トピック1の更新
        if len(self.timestamps1) >= 2:
            t1 = list(self.timestamps1)

            self.line_acc_x1.set_data(t1, list(self.accel_x1))
            self.line_acc_y1.set_data(t1, list(self.accel_y1))
            self.line_acc_z1.set_data(t1, list(self.accel_z1))

            self.line_gyro_x1.set_data(t1, list(self.gyro_x1))
            self.line_gyro_y1.set_data(t1, list(self.gyro_y1))
            self.line_gyro_z1.set_data(t1, list(self.gyro_z1))

            self.line_roll1.set_data(t1, list(self.roll1))
            self.line_pitch1.set_data(t1, list(self.pitch1))
            self.line_yaw1.set_data(t1, list(self.yaw1))

        # トピック2の更新（デュアルモードの場合）
        if self.dual_mode and len(self.timestamps2) >= 2:
            t2 = list(self.timestamps2)

            self.line_acc_x2.set_data(t2, list(self.accel_x2))
            self.line_acc_y2.set_data(t2, list(self.accel_y2))
            self.line_acc_z2.set_data(t2, list(self.accel_z2))

            self.line_gyro_x2.set_data(t2, list(self.gyro_x2))
            self.line_gyro_y2.set_data(t2, list(self.gyro_y2))
            self.line_gyro_z2.set_data(t2, list(self.gyro_z2))

            self.line_roll2.set_data(t2, list(self.roll2))
            self.line_pitch2.set_data(t2, list(self.pitch2))
            self.line_yaw2.set_data(t2, list(self.yaw2))

        # 軸の範囲を自動調整
        for ax in self.axes:
            ax.relim()
            ax.autoscale_view()

        # 描画更新
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)

    # コマンドライン引数からトピック名を取得
    import argparse
    parser = argparse.ArgumentParser(description='IMU Visualizer - Display 1 or 2 IMU topics')
    parser.add_argument('topic1', nargs='?', default='/imu', help='First IMU topic (default: /imu)')
    parser.add_argument('topic2', nargs='?', default='/livox/imu_ms2', help='Second IMU topic (default: /livox/imu_ms2)')

    # ROS2の引数を除外
    argv = [arg for arg in sys.argv if not arg.startswith('__')]
    parsed_args = parser.parse_args(argv[1:])

    node = IMUVisualizer(parsed_args.topic1, parsed_args.topic2)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close('all')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()