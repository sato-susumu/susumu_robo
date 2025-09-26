from __future__ import annotations

from typing import Optional, List
import os
import subprocess
import signal
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription
from std_msgs.msg import String


class KeyEventHandlerNode(Node):
    def __init__(self) -> None:
        super().__init__('key_event_handler')

        # パラメータの宣言
        self.declare_parameter('key_event_topic', 'key_event')
        self.declare_parameter('robo_doctor_path', '/home/taro/ros2_ws/src/susumu_robo/launch/robo_doctor.py')
        self.declare_parameter('rosbag_base_dir', os.path.expanduser('~/ros2_bags'))

        # パラメータの取得
        self.key_event_topic: str = self.get_parameter('key_event_topic').get_parameter_value().string_value
        self.robo_doctor_path: str = self.get_parameter('robo_doctor_path').get_parameter_value().string_value
        self.rosbag_base_dir: str = self.get_parameter('rosbag_base_dir').get_parameter_value().string_value

        # rosbagプロセスの管理
        self.rosbag_process: Optional[subprocess.Popen] = None
        self.current_rosbag_dir: Optional[str] = None

        # サブスクライバーの作成
        self.key_subscriber: Subscription = self.create_subscription(
            String,
            self.key_event_topic,
            self.key_event_callback,
            10
        )

        self.get_logger().info(f"Key Event Handler started")
        self.get_logger().info(f"Listening on topic: {self.key_event_topic}")
        self.get_logger().info(f"Key 1: Toggle rosbag recording")
        self.get_logger().info(f"Key 2: Run system diagnostics")

    def key_event_callback(self, msg: String) -> None:
        """キーイベントの処理"""
        self.get_logger().info(f"Received key event: {msg.data}")

        key_value: str = msg.data
        if key_value == '1':
            self.toggle_rosbag_recording()
        elif key_value == '2':
            self.run_diagnostics()
        else:
            # 他のキーは無視
            pass

    def toggle_rosbag_recording(self) -> None:
        """rosbag録画の開始/停止を切り替え"""
        self.get_logger().info("Toggling rosbag recording...")

        if self.rosbag_process is None or self.rosbag_process.poll() is not None:
            # 録画していない場合は開始
            self.get_logger().info("Starting rosbag recording...")
            self.start_rosbag_recording()
        else:
            # 録画中の場合は停止
            self.get_logger().info("Stopping rosbag recording...")
            self.stop_rosbag_recording()

    def start_rosbag_recording(self) -> None:
        """rosbag録画を開始"""
        try:
            # タイムスタンプ付きディレクトリ名を生成
            timestamp: str = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.current_rosbag_dir = os.path.join(self.rosbag_base_dir, timestamp)

            # ディレクトリを作成
            os.makedirs(self.rosbag_base_dir, exist_ok=True)

            # rosbag recordコマンドを構築
            cmd: List[str] = [
                'ros2', 'bag', 'record',
                '--all',
                '-s', 'mcap',
                '-o', self.current_rosbag_dir
            ]

            # プロセスを開始
            self.rosbag_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # プロセスグループを作成（後でグループごと終了するため）
            )

            self.get_logger().info(f"Started rosbag recording to: {self.current_rosbag_dir}")

        except Exception as e:
            self.get_logger().error(f"Failed to start rosbag recording: {e}")
            self.rosbag_process = None
            self.current_rosbag_dir = None

    def stop_rosbag_recording(self) -> None:
        """rosbag録画を停止"""
        if self.rosbag_process is not None:
            try:
                # プロセスグループ全体にSIGINTを送信（Ctrl+C相当）
                os.killpg(os.getpgid(self.rosbag_process.pid), signal.SIGINT)

                # プロセスの終了を待つ（最大5秒）
                try:
                    self.rosbag_process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    # タイムアウトした場合は強制終了
                    os.killpg(os.getpgid(self.rosbag_process.pid), signal.SIGTERM)
                    self.rosbag_process.wait(timeout=2.0)

                self.get_logger().info(f"Stopped rosbag recording. Data saved to: {self.current_rosbag_dir}")

            except Exception as e:
                self.get_logger().error(f"Error stopping rosbag recording: {e}")
                # 最終手段としてSIGKILL
                try:
                    os.killpg(os.getpgid(self.rosbag_process.pid), signal.SIGKILL)
                except Exception:
                    pass

            finally:
                self.rosbag_process = None
                self.current_rosbag_dir = None

    def run_diagnostics(self) -> None:
        """システム診断を実行"""
        try:
            self.get_logger().info("Running system diagnostics...")

            # robo_doctor.pyを実行
            result = subprocess.run(
                ['python3', self.robo_doctor_path],
                capture_output=True,
                text=True,
                timeout=30
            )

            # 結果を判定
            if result.returncode == 0:
                self.get_logger().info("System diagnostics: OK - All systems operational")
            elif result.returncode == 1:
                self.get_logger().warn("System diagnostics: WARNING - Partial operation")
            else:
                self.get_logger().error("System diagnostics: NG - System issues detected")

            # 詳細ログを出力（デバッグ用）
            if result.stdout:
                for line in result.stdout.strip().split('\n'):
                    self.get_logger().debug(f"Doctor output: {line}")

        except subprocess.TimeoutExpired:
            self.get_logger().error("System diagnostics timeout")
        except FileNotFoundError:
            self.get_logger().error(f"robo_doctor.py not found at: {self.robo_doctor_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to run diagnostics: {e}")

    def destroy_node(self) -> None:
        """ノード終了時のクリーンアップ"""
        # 録画中の場合は停止
        if self.rosbag_process is not None and self.rosbag_process.poll() is None:
            self.get_logger().info("Stopping rosbag recording before shutdown...")
            self.stop_rosbag_recording()
        super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node: KeyEventHandlerNode = KeyEventHandlerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()