#!/usr/bin/env python3

"""
robo_doctor.py - Diagnostic script for robo.launch.py
Checks if all required nodes and topics are running properly
"""

import subprocess
import sys
import socket
import json
import os
from typing import List, Tuple


class RoboDoctor:
    def __init__(self):
        self.expected_nodes = [
            # Mid-360 LiDAR nodes
            'livox_lidar_publisher',
            'livox_to_pointcloud2_node',
            'pointcloud_to_laserscan',
            'livox_imu_converter',

            # GNSS node
            'septentrio_gnss_driver_container',

            # IMU node
            'witmotion',

            # # Dummy NavSatFix node
            # 'dummy_navsatfix_publisher',

            # Static transform nodes
            'ecef_to_enu_transform',
            'static_transform_publisher',

            # Key event system nodes
            'key_event_handler',
            'number_key_publisher',

            # TTS/VoiceVox nodes
            'voicevox_ros2',
            'to_human_2_voicevox'
        ]

        self.expected_topics = [
            # LiDAR topics
            '/livox/lidar',
            '/livox/imu',
            '/livox/imu_ms2',
            '/converted_pointcloud2',
            '/scan',

            # IMU topics
            '/imu',

            # GNSS topics
            '/fix',

            # # TF topics
            # '/tf',
            # '/tf_static',

            # Key event topics
            '/key_event',

            # VoiceVox topics
            '/voicevox_ros2/speaker',
            '/to_human'
        ]

        # Critical topics that must have data flowing (IMU, GNSS, LiDAR)
        self.critical_data_topics = [
            ('/livox/lidar', 'LiDAR データ'),
            ('/scan', 'LiDAR スキャン'),
            ('/imu', 'IMU データ'),
            ('/fix', 'GNSS 位置情報'),
        ]

        self.network_checks = [
            ('Livox LiDAR', '192.168.1.145', None, 'ping'),
            ('GNSS Septentrio', '192.168.3.1', 28784, 'tcp'),
            ('Host Network (192.168.1.x)', '192.168.1.5', None, 'interface'),
            ('Livox Config Validation', None, None, 'livox_config')
        ]

        # Required PTP checks
        self.ptp_checks = [
            ('PTP Service Status', 'ptpd', None, 'service'),
            ('PTP Process Running', 'ptpd', None, 'process'),
            ('PTP Traffic Check', 'any', 319, 'ptp_tshark'),
            ('PTP Slave Status', '192.168.1.145', None, 'ptp_slave')
        ]

        # Optional PTP checks (warnings only, not critical for operation)
        self.ptp_optional_checks = [
            ('PTP Master Status (Optional)', '192.168.1.5', None, 'ptp_master'),
        ]

        # Process checks (warnings for missing processes)
        self.process_checks = [
            ('str2str NTRIP Process', 'str2str', 'process'),
        ]

        # Device checks (check if devices are present and sending data)
        self.device_checks = [
            ('IMU WT901 Device', '/dev/imu_wt901', 'device_data'),
        ]

    def run_ros2_command(self, cmd: List[str]) -> Tuple[bool, str]:
        """Run a ROS2 command and return success status and output"""
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=10
            )
            return result.returncode == 0, result.stdout.strip()
        except subprocess.TimeoutExpired:
            return False, "Command timeout"
        except Exception as e:
            return False, str(e)

    def check_node(self, node_name: str) -> bool:
        """Check if a ROS2 node is running"""
        success, output = self.run_ros2_command(['ros2', 'node', 'list'])
        if success:
            nodes = [node.strip() for node in output.split('\n') if node.strip()]
            # Check for exact match or as substring (for namespaced nodes)
            for node in nodes:
                if node_name in node or node.endswith(f'/{node_name}') or node == f'/{node_name}':
                    return True
        return False

    def check_topic(self, topic_name: str) -> bool:
        """Check if a ROS2 topic exists"""
        success, output = self.run_ros2_command(['ros2', 'topic', 'list'])
        if success:
            return topic_name in output.split('\n')
        return False

    def check_topic_data_flow(self, topic_name: str, timeout: int = 3) -> Tuple[bool, str]:
        """Check if a topic is publishing data"""
        try:
            # First check if topic exists
            if not self.check_topic(topic_name):
                return False, "トピックが存在しません"

            # Check publisher count
            result = subprocess.run(
                ['ros2', 'topic', 'info', topic_name],
                capture_output=True,
                text=True,
                timeout=5
            )

            if result.returncode != 0:
                return False, "トピック情報取得失敗"

            # Parse publisher count
            pub_count = 0
            for line in result.stdout.split('\n'):
                if 'Publisher count:' in line:
                    try:
                        pub_count = int(line.split(':')[1].strip())
                    except:
                        pass

            if pub_count == 0:
                return False, f"パブリッシャーなし (0個)"

            # Try to receive at least one message
            result = subprocess.run(
                ['timeout', str(timeout), 'ros2', 'topic', 'echo', topic_name, '--once'],
                capture_output=True,
                text=True,
                timeout=timeout + 1
            )

            if result.returncode == 0 and len(result.stdout.strip()) > 0:
                return True, f"データ受信OK (パブリッシャー: {pub_count}個)"
            elif result.returncode == 124:  # timeout exit code
                return False, f"タイムアウト ({timeout}秒、パブリッシャー: {pub_count}個)"
            else:
                return False, f"データ受信失敗 (パブリッシャー: {pub_count}個)"

        except subprocess.TimeoutExpired:
            return False, f"タイムアウト ({timeout}秒)"
        except Exception as e:
            return False, f"エラー: {str(e)}"


    def ping_host(self, host: str, timeout: int = 1) -> bool:
        """Ping a host to check network connectivity"""
        try:
            result = subprocess.run(
                ['ping', '-c', '1', '-W', str(timeout), host],
                capture_output=True,
                text=True,
                timeout=timeout + 1
            )
            return result.returncode == 0
        except (subprocess.TimeoutExpired, Exception):
            return False

    def check_tcp_connection(self, host: str, port: int, timeout: int = 1) -> bool:
        """Check if TCP connection to host:port is possible"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(timeout)
            result = sock.connect_ex((host, port))
            sock.close()
            return result == 0
        except Exception:
            return False

    def check_network_interface(self, expected_ip: str) -> bool:
        """Check if network interface has the expected IP or is in same subnet"""
        try:
            # Get all network interfaces
            result = subprocess.run(
                ['ip', 'addr', 'show'],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                # Check if the expected IP or subnet exists
                lines = result.stdout.split('\n')
                subnet = '.'.join(expected_ip.split('.')[:-1])  # Get subnet (e.g., "192.168.1")
                for line in lines:
                    if 'inet ' in line and subnet in line:
                        return True
            return False
        except Exception:
            return False

    def get_current_host_ip_for_livox(self) -> str:
        """Get the current host IP in 192.168.1.x subnet"""
        try:
            result = subprocess.run(
                ['ip', 'addr', 'show'],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                lines = result.stdout.split('\n')
                for line in lines:
                    if 'inet ' in line and '192.168.1.' in line:
                        # Extract IP address
                        parts = line.strip().split()
                        for part in parts:
                            if '192.168.1.' in part and '/' in part:
                                return part.split('/')[0]
            return None
        except Exception:
            return None

    def validate_livox_config(self) -> Tuple[bool, str]:
        """Validate Livox MID360 config file matches current network configuration"""
        try:
            # Find MID360_config.json in workspace
            config_paths = [
                '/home/taro/ros2_ws/src/livox_ros_driver2/config/MID360_config.json',
                '/home/taro/ros2_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json'
            ]

            config_path = None
            for path in config_paths:
                if os.path.exists(path):
                    config_path = path
                    break

            if not config_path:
                return False, "MID360_config.jsonが見つかりません"

            # Read config file
            with open(config_path, 'r') as f:
                config = json.load(f)

            # Get current host IP
            current_ip = self.get_current_host_ip_for_livox()
            if not current_ip:
                return False, "192.168.1.xサブネットのホストIPが見つかりません"

            # Check if config matches current IP
            host_net_info = config.get('MID360', {}).get('host_net_info', {})
            config_ips = [
                host_net_info.get('cmd_data_ip'),
                host_net_info.get('push_msg_ip'),
                host_net_info.get('point_data_ip'),
                host_net_info.get('imu_data_ip')
            ]

            # All IPs should match current host IP
            mismatches = []
            for key, ip in zip(['cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip'], config_ips):
                if ip != current_ip:
                    mismatches.append(f"{key}={ip}")

            if mismatches:
                return False, f"設定IPが不一致: {', '.join(mismatches)} (正しくは {current_ip})"

            return True, f"設定検証OK: ホストIP {current_ip}"

        except Exception as e:
            return False, f"検証エラー: {str(e)}"

    def check_service_status(self, service_name: str) -> bool:
        """Check if a systemd service is running"""
        try:
            result = subprocess.run(
                ['systemctl', 'is-active', service_name],
                capture_output=True,
                text=True,
                timeout=5
            )
            status = result.stdout.strip()
            # Consider both 'active' and 'activating' as running for ptpd with -C option
            return status in ['active', 'activating']
        except Exception:
            return False

    def check_ptpd_process(self) -> bool:
        """Check if ptpd process is actually running (not just service active)"""
        try:
            # Check for ptpd process
            result = subprocess.run(
                ['pgrep', '-f', 'ptpd'],
                capture_output=True,
                text=True,
                timeout=5
            )
            return result.returncode == 0 and len(result.stdout.strip()) > 0
        except Exception:
            return False

    def check_process_running(self, process_name: str) -> bool:
        """Check if a process is running by name"""
        try:
            result = subprocess.run(
                ['pgrep', '-f', process_name],
                capture_output=True,
                text=True,
                timeout=5
            )
            return result.returncode == 0 and len(result.stdout.strip()) > 0
        except Exception:
            return False

    def check_device_data(self, device_path: str, timeout: int = 2) -> bool:
        """Check if device exists and is sending data (or busy/in use)"""
        import os
        import select
        import errno

        # First check if device exists
        if not os.path.exists(device_path):
            return False

        # Try to read data from device
        try:
            fd = os.open(device_path, os.O_RDONLY | os.O_NONBLOCK)
            try:
                # Use select to wait for data with timeout
                ready, _, _ = select.select([fd], [], [], timeout)
                if ready:
                    # Try to read some data
                    data = os.read(fd, 100)
                    return len(data) > 0
                return False
            finally:
                os.close(fd)
        except OSError as e:
            # If device is busy (EBUSY), it means another program is using it
            # This is OK - the device exists and is being used
            if e.errno == errno.EBUSY or e.errno == errno.EAGAIN:
                return True
            return False
        except Exception:
            return False

    def check_ptp_tshark(self, interface: str, port: int, timeout: int = 3) -> bool:
        """Check if PTP packets are being received using tshark"""
        try:
            # Use specific interface instead of 'any' and check for PTP traffic
            if interface == 'any':
                interface = 'enx00e04c682856'  # Use the actual PTP interface

            # Use tshark with PTP-specific filters for better detection
            cmd = [
                'sudo', 'timeout', str(timeout),
                'tshark', '-i', interface,
                '-f', 'udp port 319 or udp port 320 or ether proto 0x88f7',
                '-Y', 'ptp',
                '-c', '1', '-q'  # -q for quiet output
            ]

            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=timeout + 1
            )

            # Check if tshark captured any PTP packets
            output = result.stderr + result.stdout
            has_packets = (result.returncode == 0 and
                          ('captured' in output or
                           'PTPv2' in output or
                           'Sync Message' in output or
                           'Follow_Up Message' in output or
                           'Delay_Req' in output))
            return has_packets

        except subprocess.TimeoutExpired:
            return False
        except Exception:
            # Fallback to tcpdump if tshark fails
            return self._check_ptp_tcpdump_fallback(interface, port, timeout)

    def _check_ptp_tcpdump_fallback(self, interface: str, port: int, timeout: int = 3) -> bool:
        """Fallback tcpdump method for PTP packet detection"""
        try:
            cmd = [
                'sudo', 'timeout', str(timeout),
                'tcpdump', '-i', interface, '-c', '1', '-n',
                'port 319 or port 320'
            ]

            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=timeout + 1
            )

            output = result.stderr + result.stdout
            has_packets = (result.returncode == 0 and
                          ('PTPv2' in output or
                           'IP 192.168.1.5' in output or
                           'captured' in output))
            return has_packets

        except subprocess.TimeoutExpired:
            return False
        except Exception:
            return False

    def check_ptp_master_status(self, master_ip: str) -> bool:
        """Check if PTP master is reachable and potentially acting as master"""
        # First check basic connectivity
        if not self.ping_host(master_ip, timeout=2):
            return False

        # Check if ptpd process is running (if this is the local master)
        if master_ip in ['127.0.0.1', 'localhost'] or self.is_local_ip(master_ip):
            return self.check_service_status('ptpd')

        # For remote masters, ping is sufficient for now
        return True

    def check_ptp_slave_status(self, slave_ip: str) -> bool:
        """Check if PTP slave is reachable"""
        return self.ping_host(slave_ip, timeout=2)

    def is_local_ip(self, ip: str) -> bool:
        """Check if IP address is local to this machine"""
        try:
            result = subprocess.run(
                ['ip', 'addr', 'show'],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                return ip in result.stdout
            return False
        except Exception:
            return False

    def check_network_connectivity(self) -> Tuple[int, int]:
        """Check network connectivity for all required devices"""
        print("\n--- ネットワーク接続 ---")
        network_ok = 0

        for name, host, port, check_type in self.network_checks:
            if check_type == 'ping':
                if self.ping_host(host):
                    print(f"[OK] {name}: {host} (ping成功)")
                    network_ok += 1
                else:
                    print(f"[エラー] {name}: {host} (ping失敗)")

            elif check_type == 'tcp':
                if port is not None:
                    if self.check_tcp_connection(host, port):
                        print(f"[OK] {name}: {host}:{port} (TCP接続成功)")
                        network_ok += 1
                    else:
                        print(f"[エラー] {name}: {host}:{port} (TCP接続失敗)")
                else:
                    print(f"[エラー] {name}: TCPチェックのポートが未指定")

            elif check_type == 'interface':
                if self.check_network_interface(host):
                    print(f"[OK] {name}: {host}サブネット用のネットワークインターフェース設定済み")
                    network_ok += 1
                else:
                    print(f"[エラー] {name}: {host}サブネット用のネットワークインターフェースが見つかりません")

            elif check_type == 'livox_config':
                success, message = self.validate_livox_config()
                if success:
                    print(f"[OK] {name}: {message}")
                    network_ok += 1
                else:
                    print(f"[エラー] {name}: {message}")
                    print(f"       対処法: MID360_config.jsonを正しいホストIPで更新してパッケージを再ビルド")

        return network_ok, len(self.network_checks)

    def check_ptp_services(self) -> Tuple[int, int]:
        """Check PTP server status and packet flow"""
        print("\n--- PTP状態 ---")
        ptp_ok = 0

        for name, target, port_or_service, check_type in self.ptp_checks:
            if check_type == 'service':
                if self.check_service_status(target):
                    print(f"[OK] {name}: サービス稼働中")
                    ptp_ok += 1
                else:
                    print(f"[エラー] {name}: サービスが動作していません")

            elif check_type == 'process':
                if self.check_ptpd_process():
                    print(f"[OK] {name}: プロセス稼働中")
                    ptp_ok += 1
                else:
                    print(f"[エラー] {name}: プロセスが動作していません")

            elif check_type == 'ptp_tshark':
                print(f"[情報] {name}: tsharkでPTPトラフィックを確認中 (数秒かかります...)")
                if self.check_ptp_tshark(target, port_or_service):
                    print(f"[OK] {name}: PTPトラフィック検出")
                    ptp_ok += 1
                else:
                    print(f"[エラー] {name}: PTPトラフィックが検出されません")

            elif check_type == 'ptp_master':
                if self.check_ptp_master_status(target):
                    print(f"[OK] {name}: PTPマスター {target} に到達可能")
                    ptp_ok += 1
                else:
                    print(f"[エラー] {name}: PTPマスター {target} に到達できません")

            elif check_type == 'ptp_slave':
                if self.check_ptp_slave_status(target):
                    print(f"[OK] {name}: PTPスレーブ {target} に到達可能")
                    ptp_ok += 1
                else:
                    print(f"[エラー] {name}: PTPスレーブ {target} に到達できません")

        return ptp_ok, len(self.ptp_checks)

    def check_ptp_optional(self) -> None:
        """Check optional PTP features (warnings only)"""
        print("\n--- PTPオプション機能 ---")

        for name, target, port_or_service, check_type in self.ptp_optional_checks:
            if check_type == 'ptp_master':
                if self.check_ptp_master_status(target):
                    print(f"[OK] {name}: PTPマスター {target} に到達可能")
                else:
                    print(f"[警告] {name}: PTPマスター {target} に到達不可 (現在はOK、マルチセンサー同期時に必要)")

    def check_processes(self) -> None:
        """Check if important processes are running (warnings only)"""
        print("\n--- プロセス状態 ---")

        for name, process_name, check_type in self.process_checks:
            if check_type == 'process':
                if self.check_process_running(process_name):
                    print(f"[OK] {name}: プロセス稼働中")
                else:
                    print(f"[警告] {name}: プロセス '{process_name}' が動作していません (GNSS RTK補正が利用不可)")

    def check_devices(self) -> Tuple[int, int]:
        """Check if devices are present and sending data"""
        print("\n--- デバイス状態 ---")
        device_ok = 0

        for name, device_path, check_type in self.device_checks:
            if check_type == 'device_data':
                import os
                if not os.path.exists(device_path):
                    print(f"[エラー] {name}: デバイス {device_path} が見つかりません")
                else:
                    if self.check_device_data(device_path):
                        print(f"[OK] {name}: デバイス {device_path} 存在、稼働中 (使用中またはデータ送信中)")
                        device_ok += 1
                    else:
                        print(f"[エラー] {name}: デバイス {device_path} は存在しますが非稼働 (電源/接続を確認)")

        return device_ok, len(self.device_checks)

    def check_nodes(self) -> Tuple[int, int]:
        """Check all expected nodes"""
        print("\n--- ノード状態 ---")
        node_ok = 0

        for node in self.expected_nodes:
            if self.check_node(node):
                print(f"[OK] ノード: {node}")
                node_ok += 1
            else:
                print(f"[未検出] ノード: {node}")

        return node_ok, len(self.expected_nodes)

    def check_topics(self) -> Tuple[int, int]:
        """Check all expected topics"""
        print("\n--- トピック状態 ---")
        topic_ok = 0

        for topic_name in self.expected_topics:
            if self.check_topic(topic_name):
                print(f"[OK] トピック: {topic_name}")
                topic_ok += 1
            else:
                print(f"[未検出] トピック: {topic_name}")

        return topic_ok, len(self.expected_topics)

    def check_critical_topic_data(self) -> Tuple[int, int]:
        """Check if critical topics are publishing data"""
        print("\n--- 重要トピックデータフロー (IMU/GNSS/LiDAR) ---")
        data_ok = 0

        for topic_name, description in self.critical_data_topics:
            # Use longer timeout for slower topics (IMU: ~10Hz, GNSS: ~1Hz)
            if topic_name in ['/imu', '/fix']:
                timeout = 10  # IMU and GNSS can publish at 1-10Hz
            else:
                timeout = 3   # LiDAR is fast

            success, message = self.check_topic_data_flow(topic_name, timeout=timeout)
            if success:
                print(f"[OK] {description} ({topic_name}): {message}")
                data_ok += 1
            else:
                print(f"[エラー] {description} ({topic_name}): {message}")

        return data_ok, len(self.critical_data_topics)

    def run_diagnostics(self) -> int:
        """Run full diagnostics and return exit code"""
        print("=== すすむロボット診断 (Robo Doctor) ===")
        print("robo.launch.py コンポーネントをチェック中...")
        print()

        # Check network connectivity first
        network_ok, network_total = self.check_network_connectivity()

        # Check PTP services
        ptp_ok, ptp_total = self.check_ptp_services()

        # Check optional PTP features (warnings only)
        self.check_ptp_optional()

        # Check processes (warnings only)
        self.check_processes()

        # Check devices
        device_ok, device_total = self.check_devices()

        # Check nodes
        node_ok, node_total = self.check_nodes()

        # Check topics
        topic_ok, topic_total = self.check_topics()

        # Check critical topic data flow (NEW)
        data_ok, data_total = self.check_critical_topic_data()

        # Summary
        print(f"\n--- サマリー ---")
        print(f"ネットワーク: {network_ok}/{network_total} OK")
        print(f"PTP: {ptp_ok}/{ptp_total} OK")
        print(f"デバイス: {device_ok}/{device_total} OK")
        print(f"ノード: {node_ok}/{node_total} OK")
        print(f"トピック: {topic_ok}/{topic_total} OK")
        print(f"データフロー: {data_ok}/{data_total} OK")

        # Overall status
        total_checks = network_total + ptp_total + device_total + node_total + topic_total + data_total
        total_ok = network_ok + ptp_ok + device_ok + node_ok + topic_ok + data_ok

        if total_ok == total_checks:
            print("状態: すべて正常! [成功]")
            return 0
        elif total_ok > total_checks * 0.5:  # More than 50% working
            print("状態: 部分的に動作 [警告]")
            return 1
        else:
            print("状態: システムダウン [エラー]")
            return 2


def main():
    doctor = RoboDoctor()
    exit_code = doctor.run_diagnostics()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()