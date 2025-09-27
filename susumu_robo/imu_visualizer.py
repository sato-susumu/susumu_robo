#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from collections import deque
import time
import math

class IMUVisualizer(Node):
    def __init__(self):
        super().__init__('imu_visualizer')

        # データ保存用
        self.max_samples = 100
        self.timestamps = deque(maxlen=self.max_samples)
        self.accel_x = deque(maxlen=self.max_samples)
        self.accel_y = deque(maxlen=self.max_samples)
        self.accel_z = deque(maxlen=self.max_samples)
        self.gyro_x = deque(maxlen=self.max_samples)
        self.gyro_y = deque(maxlen=self.max_samples)
        self.gyro_z = deque(maxlen=self.max_samples)
        self.roll = deque(maxlen=self.max_samples)
        self.pitch = deque(maxlen=self.max_samples)
        self.yaw = deque(maxlen=self.max_samples)

        self.start_time = time.time()

        # IMUトピックの購読
        self.subscription = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)

        # プロット初期化
        plt.ion()  # インタラクティブモード
        self.fig, self.axes = plt.subplots(3, 1, figsize=(10, 8))
        self.fig.suptitle('IMU Data Visualization (9-axis)')

        # 加速度プロット
        self.axes[0].set_ylabel('Acceleration (m/s²)')
        self.axes[0].grid(True)
        self.line_acc_x, = self.axes[0].plot([], [], 'r-', label='X')
        self.line_acc_y, = self.axes[0].plot([], [], 'g-', label='Y')
        self.line_acc_z, = self.axes[0].plot([], [], 'b-', label='Z')
        self.axes[0].legend()

        # 角速度プロット
        self.axes[1].set_ylabel('Angular Velocity (rad/s)')
        self.axes[1].grid(True)
        self.line_gyro_x, = self.axes[1].plot([], [], 'r-', label='X')
        self.line_gyro_y, = self.axes[1].plot([], [], 'g-', label='Y')
        self.line_gyro_z, = self.axes[1].plot([], [], 'b-', label='Z')
        self.axes[1].legend()

        # 姿勢プロット（オイラー角）
        self.axes[2].set_ylabel('Orientation (degrees)')
        self.axes[2].set_xlabel('Time (s)')
        self.axes[2].grid(True)
        self.line_roll, = self.axes[2].plot([], [], 'r-', label='Roll')
        self.line_pitch, = self.axes[2].plot([], [], 'g-', label='Pitch')
        self.line_yaw, = self.axes[2].plot([], [], 'b-', label='Yaw')
        self.axes[2].legend()

        self.get_logger().info('IMU Visualizer started')

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

    def imu_callback(self, msg):
        current_time = time.time() - self.start_time
        self.timestamps.append(current_time)

        # データ保存
        self.accel_x.append(msg.linear_acceleration.x)
        self.accel_y.append(msg.linear_acceleration.y)
        self.accel_z.append(msg.linear_acceleration.z)
        self.gyro_x.append(msg.angular_velocity.x)
        self.gyro_y.append(msg.angular_velocity.y)
        self.gyro_z.append(msg.angular_velocity.z)

        # クォータニオンをオイラー角に変換
        roll_deg, pitch_deg, yaw_deg = self.quaternion_to_euler(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        self.roll.append(roll_deg)
        self.pitch.append(pitch_deg)
        self.yaw.append(yaw_deg)

        # グラフ更新
        self.update_plot()

    def update_plot(self):
        if len(self.timestamps) < 2:
            return

        # 時間軸のデータ
        t = list(self.timestamps)

        # 加速度グラフ更新
        self.line_acc_x.set_data(t, list(self.accel_x))
        self.line_acc_y.set_data(t, list(self.accel_y))
        self.line_acc_z.set_data(t, list(self.accel_z))

        # 角速度グラフ更新
        self.line_gyro_x.set_data(t, list(self.gyro_x))
        self.line_gyro_y.set_data(t, list(self.gyro_y))
        self.line_gyro_z.set_data(t, list(self.gyro_z))

        # 姿勢グラフ更新
        self.line_roll.set_data(t, list(self.roll))
        self.line_pitch.set_data(t, list(self.pitch))
        self.line_yaw.set_data(t, list(self.yaw))

        # 軸の範囲を自動調整
        for ax in self.axes:
            ax.relim()
            ax.autoscale_view()

        # 描画更新
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = IMUVisualizer()

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