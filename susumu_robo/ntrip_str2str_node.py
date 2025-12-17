#!/usr/bin/env python3
"""
NTRIP str2str ROS2 Node

str2strコマンドをROS2ノードとしてラップし、
NTRIPクライアントの状態をサービスで公開する。

str2strの出力フォーマット:
  [CC---] 12345 B 1200 bps message

  ストリーム状態文字:
    C = Connected (接続済み)
    W = Wait (接続待ち)
    E = Error (エラー)
    - = 未使用
"""

import re
import subprocess
import threading
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from susumu_ros2_interfaces.srv import NtripStatus


# 状態文字から数値への変換マップ
STATE_MAP = {
    'C': NtripStatus.Response.STATE_CONNECTED,   # 2
    'W': NtripStatus.Response.STATE_WAITING,     # 1
    'E': NtripStatus.Response.STATE_ERROR,       # -1
    '-': NtripStatus.Response.STATE_CLOSED,      # 0
}


@dataclass
class Str2StrStatus:
    """str2strのステータス情報"""
    input_state: str = '-'      # 入力ストリーム状態 (C/W/E/-)
    output_state: str = '-'     # 出力ストリーム状態 (C/W/E/-)
    total_bytes: int = 0        # 累計受信バイト数
    bps: int = 0                # 現在のビットレート (bps)
    message: str = ''           # ステータスメッセージ
    is_running: bool = False    # プロセスが動作中か


class NtripStr2StrNode(Node):
    """
    NTRIP str2str ROS2ノード

    str2strコマンドを起動し、NTRIPサーバーから補正データを取得して
    GNSSレシーバーに転送する。状態はサービスで公開。
    """

    def __init__(self):
        super().__init__('ntrip_str2str_node')

        # パラメータ宣言
        self.declare_parameter('ntrip_server', 'ntrip://ntrip1.bizstation.jp:2101/0C8BD4BE')
        self.declare_parameter('output_dest', 'tcpcli://192.168.3.1:28785')

        # パラメータ取得
        self.ntrip_server = self.get_parameter('ntrip_server').get_parameter_value().string_value
        self.output_dest = self.get_parameter('output_dest').get_parameter_value().string_value

        # 状態
        self.status = Str2StrStatus()
        self.process: Optional[subprocess.Popen] = None
        self.lock = threading.Lock()
        self.shutdown_flag = False

        # Service: ステータス取得
        self.status_srv = self.create_service(
            NtripStatus,
            '~/get_status',
            self.get_status_callback
        )

        # str2strプロセス起動
        self.start_str2str()

        self.get_logger().info(f'NTRIP str2str node started')
        self.get_logger().info(f'  Input:  {self.ntrip_server}')
        self.get_logger().info(f'  Output: {self.output_dest}')

    def start_str2str(self):
        """str2strプロセスを起動"""
        cmd = [
            'str2str',
            '-in', self.ntrip_server,
            '-out', self.output_dest
        ]

        try:
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1
            )

            with self.lock:
                self.status.is_running = True

            # 出力読み取りスレッド起動
            self.reader_thread = threading.Thread(target=self._read_output, daemon=True)
            self.reader_thread.start()

            self.get_logger().info(f'str2str process started (PID: {self.process.pid})')

        except FileNotFoundError:
            self.get_logger().error('str2str command not found. Please install RTKLIB.')
            with self.lock:
                self.status.is_running = False
                self.status.message = 'str2str command not found'
        except Exception as e:
            self.get_logger().error(f'Failed to start str2str: {e}')
            with self.lock:
                self.status.is_running = False
                self.status.message = str(e)

    def _read_output(self):
        """str2strの出力を読み取ってステータスを更新"""
        if self.process is None or self.process.stdout is None:
            return

        # str2str出力パターン: [CC---] 12345 B 1200 bps message
        pattern = re.compile(
            r'\[([CWER-])([CWER-])([CWER-])([CWER-])([CWER-])\]\s+'
            r'(\d+)\s*B\s+'
            r'(\d+)\s*bps\s*'
            r'(.*)?'
        )

        while not self.shutdown_flag:
            try:
                line = self.process.stdout.readline()
                if not line:
                    # プロセス終了
                    break

                line = line.strip()
                if not line:
                    continue

                # ログ出力
                self.get_logger().debug(f'str2str: {line}')

                # パターンマッチ
                match = pattern.search(line)
                if match:
                    with self.lock:
                        self.status.input_state = match.group(1)
                        self.status.output_state = match.group(2)
                        self.status.total_bytes = int(match.group(6))
                        self.status.bps = int(match.group(7))
                        self.status.message = match.group(8) or ''
                else:
                    # パターンにマッチしない行はメッセージとして保存
                    with self.lock:
                        self.status.message = line

            except Exception as e:
                self.get_logger().warn(f'Error reading str2str output: {e}')
                break

        # プロセス終了処理
        with self.lock:
            self.status.is_running = False

        if self.process:
            return_code = self.process.poll()
            self.get_logger().info(f'str2str process exited (return code: {return_code})')

    def get_status_callback(self, request, response):
        """ステータス取得サービスコールバック"""
        with self.lock:
            # 基本ステータス
            response.is_running = self.status.is_running
            response.is_connected = (
                self.status.is_running and
                self.status.input_state == 'C' and
                self.status.output_state == 'C'
            )

            # 接続状態（数値）
            response.input_state = STATE_MAP.get(
                self.status.input_state,
                NtripStatus.Response.STATE_CLOSED
            )
            response.output_state = STATE_MAP.get(
                self.status.output_state,
                NtripStatus.Response.STATE_CLOSED
            )

            # データ統計
            response.total_bytes = self.status.total_bytes
            response.bps = self.status.bps

            # 接続情報
            response.ntrip_server = self.ntrip_server
            response.output_dest = self.output_dest

            # メッセージ
            response.message = self.status.message

        return response

    def destroy_node(self):
        """ノード終了処理"""
        self.shutdown_flag = True

        # str2strプロセス終了
        if self.process:
            self.get_logger().info('Terminating str2str process...')
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.get_logger().warn('str2str did not terminate, killing...')
                self.process.kill()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = NtripStr2StrNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
