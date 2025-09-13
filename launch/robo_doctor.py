#!/usr/bin/env python3

"""
robo_doctor.py - Diagnostic script for robo.launch.py
Checks if all required nodes and topics are running properly
"""

import subprocess
import sys
import os
import socket
from typing import List, Tuple


class RoboDoctor:
    def __init__(self):
        self.expected_nodes = [
            'livox_lidar_publisher',
            'livox_to_pointcloud2_node',
            'pointcloud_to_laserscan',
            'septentrio_gnss_driver_container',
            'witmotion'
        ]

        self.expected_topics = [
            '/livox/lidar',
            '/livox/imu',
            '/scan',
            '/imu',
            '/navsatfix',
            '/converted_pointcloud2',
            '/tf_static'
        ]

        self.network_checks = [
            ('Livox LiDAR', '192.168.1.145', None, 'ping'),
            ('GNSS Septentrio', '192.168.3.1', 28784, 'tcp'),
            ('Host Network (192.168.1.x)', '192.168.1.144', None, 'interface')
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

    def check_network_connectivity(self) -> Tuple[int, int]:
        """Check network connectivity for all required devices"""
        print("\n--- Network Connectivity ---")
        network_ok = 0

        for name, host, port, check_type in self.network_checks:
            if check_type == 'ping':
                if self.ping_host(host):
                    print(f"[OK] {name}: {host} (ping successful)")
                    network_ok += 1
                else:
                    print(f"[ERROR] {name}: {host} (ping failed)")

            elif check_type == 'tcp':
                if port is not None:
                    if self.check_tcp_connection(host, port):
                        print(f"[OK] {name}: {host}:{port} (TCP connection successful)")
                        network_ok += 1
                    else:
                        print(f"[ERROR] {name}: {host}:{port} (TCP connection failed)")
                else:
                    print(f"[ERROR] {name}: No port specified for TCP check")

            elif check_type == 'interface':
                if self.check_network_interface(host):
                    print(f"[OK] {name}: Network interface configured for {host} subnet")
                    network_ok += 1
                else:
                    print(f"[ERROR] {name}: No network interface found for {host} subnet")

        return network_ok, len(self.network_checks)

    def check_nodes(self) -> Tuple[int, int]:
        """Check all expected nodes"""
        print("\n--- Node Status ---")
        node_ok = 0

        for node in self.expected_nodes:
            if self.check_node(node):
                print(f"[OK] Node: {node}")
                node_ok += 1
            else:
                print(f"[MISSING] Node: {node}")

        return node_ok, len(self.expected_nodes)

    def check_topics(self) -> Tuple[int, int]:
        """Check all expected topics"""
        print("\n--- Topic Status ---")
        topic_ok = 0

        for topic_name in self.expected_topics:
            if self.check_topic(topic_name):
                print(f"[OK] Topic: {topic_name}")
                topic_ok += 1
            else:
                print(f"[MISSING] Topic: {topic_name}")

        return topic_ok, len(self.expected_topics)

    def run_diagnostics(self) -> int:
        """Run full diagnostics and return exit code"""
        print("=== Susumu Robot Doctor - Diagnostics ===")
        print("Checking robo.launch.py components...")
        print()

        # Check network connectivity first
        network_ok, network_total = self.check_network_connectivity()

        # Check nodes
        node_ok, node_total = self.check_nodes()

        # Check topics
        topic_ok, topic_total = self.check_topics()

        # Summary
        print(f"\n--- Summary ---")
        print(f"Network: {network_ok}/{network_total} OK")
        print(f"Nodes: {node_ok}/{node_total} OK")
        print(f"Topics: {topic_ok}/{topic_total} OK")

        # Overall status
        total_checks = network_total + node_total + topic_total
        total_ok = network_ok + node_ok + topic_ok

        if total_ok == total_checks:
            print("Status: ALL SYSTEMS GO! [SUCCESS]")
            return 0
        elif total_ok > total_checks * 0.5:  # More than 50% working
            print("Status: PARTIAL OPERATION [WARNING]")
            return 1
        else:
            print("Status: SYSTEM DOWN [ERROR]")
            return 2


def main():
    doctor = RoboDoctor()
    exit_code = doctor.run_diagnostics()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()