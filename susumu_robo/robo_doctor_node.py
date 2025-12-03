#!/usr/bin/env python3
"""
robo_doctor_node.py - ROS2 Diagnostic Node for Susumu Robot

Publishes diagnostic information to /diagnostics topic at different intervals:
- Light checks (60s): Battery, PTP service/process
- Medium checks (120s): Network ping, TCP connections, devices
- Heavy checks (180s): ROS2 nodes, topics, NTRIP status
"""

import os
import socket
import subprocess
import threading
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class RoboDoctorNode(Node):
    """Diagnostic node for Susumu Robot system health monitoring."""

    def __init__(self):
        super().__init__('robo_doctor_node')

        # Configuration
        self.hardware_id = 'susumu_robo'
        self.battery_warning_threshold = 80

        # Network targets
        self.lidar_ip = '192.168.1.145'
        self.gnss_ip = '192.168.3.1'
        self.gnss_port = 28784
        self.host_subnet = '192.168.1'

        # Expected nodes and topics
        self.expected_nodes = [
            'livox_lidar_publisher',
            'livox_to_pointcloud2_node',
            'pointcloud_to_laserscan',
            'livox_imu_converter',
            'septentrio_gnss_driver_container',
            'ntrip_str2str_node',
            'witmotion',
            'ecef_to_enu_transform',
            'static_transform_publisher',
            'key_event_handler',
            'voicevox_ros2',
            'to_human_2_voicevox'
        ]

        self.expected_topics = [
            '/livox/lidar',
            '/livox/imu',
            '/livox/imu_ms2',
            '/converted_pointcloud2',
            '/scan',
            '/imu',
            '/fix',
            '/key_event',
            '/voicevox_ros2/speaker',
            '/to_human'
        ]

        # Critical topics that must have data flowing
        self.critical_data_topics = [
            ('/livox/lidar', 'LiDAR data'),
            ('/scan', 'LiDAR scan'),
            ('/imu', 'IMU data'),
            ('/fix', 'GNSS position'),
        ]

        # Livox config paths
        self.livox_config_paths = [
            '/home/taro/ros2_ws/src/livox_ros_driver2/config/MID360_config.json',
            '/home/taro/ros2_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json'
        ]

        # PTP check targets
        self.ptp_slave_ip = '192.168.1.145'  # Livox LiDAR
        self.ptp_master_ip = '192.168.1.156'  # Local host as PTP master

        # Device checks
        self.imu_device = '/dev/imu_wt901'

        # Cache for check results (thread-safe)
        self._cache_lock = threading.Lock()
        self._cached_statuses = []  # Cached DiagnosticStatus list

        # Background thread management (prevents concurrent execution of same check)
        self._running_lock = threading.Lock()
        self._running_checks = set()  # Set of currently running check names

        # Publisher
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        # Timer for publishing cached results (every 3 seconds to avoid stale)
        self.timer_publish = self.create_timer(3.0, self._publish_cached_diagnostics)

        # Check interval (all checks run at the same interval)
        check_interval = 60.0
        self.timer_light = self.create_timer(check_interval, self._schedule_light_checks)
        self.timer_medium = self.create_timer(check_interval, self._schedule_medium_checks)
        self.timer_heavy = self.create_timer(check_interval, self._schedule_heavy_checks)

        # Run initial checks after short delay
        self._initial_done = False
        self.timer_initial = self.create_timer(2.0, self._initial_check)

        self.get_logger().info('RoboDoctorNode started')
        self.get_logger().info(f'  Publish interval: 3s, Check interval: {check_interval}s')

    def _initial_check(self):
        """Run all checks once at startup (in background thread)."""
        if self._initial_done:
            return
        self._initial_done = True

        # Cancel this one-shot timer
        self.timer_initial.cancel()

        # Run initial checks in background thread
        def run_initial():
            self.get_logger().info('Running initial diagnostics...')
            self._run_light_checks()
            self._run_medium_checks()
            self._run_heavy_checks()
            self.get_logger().info('Initial diagnostics complete')

        thread = threading.Thread(target=run_initial, daemon=True)
        thread.start()

    # =========================================================================
    # Background Thread Scheduler
    # =========================================================================

    def _schedule_in_background(self, name: str, func):
        """
        Schedule a function to run in background thread.

        Args:
            name: Unique identifier to prevent concurrent execution of same task
            func: Function to execute in background
        """
        with self._running_lock:
            if name in self._running_checks:
                return  # Already running, skip
            self._running_checks.add(name)

        def run():
            try:
                func()
            finally:
                with self._running_lock:
                    self._running_checks.discard(name)

        thread = threading.Thread(target=run, daemon=True)
        thread.start()

    def _schedule_light_checks(self):
        """Schedule light checks in background thread."""
        self._schedule_in_background('light', self._run_light_checks)

    def _schedule_medium_checks(self):
        """Schedule medium checks in background thread."""
        self._schedule_in_background('medium', self._run_medium_checks)

    def _schedule_heavy_checks(self):
        """Schedule heavy checks in background thread."""
        self._schedule_in_background('heavy', self._run_heavy_checks)

    # =========================================================================
    # Cache Management and Publishing
    # =========================================================================

    def _update_cache(self, statuses: list):
        """Update cached statuses with new results."""
        with self._cache_lock:
            # Update or add statuses by name
            for new_status in statuses:
                found = False
                for i, cached in enumerate(self._cached_statuses):
                    if cached.name == new_status.name:
                        self._cached_statuses[i] = new_status
                        found = True
                        break
                if not found:
                    self._cached_statuses.append(new_status)

    def _publish_cached_diagnostics(self):
        """Publish all cached diagnostics (called every 5 seconds)."""
        with self._cache_lock:
            if not self._cached_statuses:
                return

            msg = DiagnosticArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.status = list(self._cached_statuses)
            self.diag_pub.publish(msg)

    # =========================================================================
    # Check Runners (update cache)
    # =========================================================================

    def _run_light_checks(self):
        """Run light checks and update cache."""
        statuses = [
            self._check_battery(),
            self._check_ptp_service(),
            self._check_ptp_process(),
        ]
        self._update_cache(statuses)

    def _run_medium_checks(self):
        """Run medium checks and update cache."""
        statuses = [
            self._check_network_lidar(),
            self._check_network_gnss(),
            self._check_host_interface(),
            self._check_device_imu(),
        ]
        self._update_cache(statuses)

    def _run_heavy_checks(self):
        """Run heavy checks and update cache."""
        statuses = []
        statuses.extend(self._check_ros2_nodes())
        statuses.extend(self._check_ros2_topics())
        statuses.extend(self._check_critical_data_flow())
        statuses.append(self._check_livox_config())
        statuses.append(self._check_ptp_slave_status())
        statuses.append(self._check_ptp_master_status())
        statuses.append(self._check_ntrip_status())
        self._update_cache(statuses)

    # =========================================================================
    # Light Checks
    # =========================================================================

    def _check_battery(self) -> DiagnosticStatus:
        """Check PC battery level."""
        status = DiagnosticStatus()
        status.name = '/robo_doctor/System/Battery'
        status.hardware_id = self.hardware_id

        try:
            power_supply_path = '/sys/class/power_supply'
            battery_path = None

            if os.path.exists(power_supply_path):
                for device in os.listdir(power_supply_path):
                    device_type_path = os.path.join(power_supply_path, device, 'type')
                    if os.path.exists(device_type_path):
                        with open(device_type_path, 'r') as f:
                            if f.read().strip() == 'Battery':
                                battery_path = os.path.join(power_supply_path, device)
                                break

            if not battery_path:
                status.level = DiagnosticStatus.OK
                status.message = 'No battery (Desktop PC)'
                return status

            capacity_path = os.path.join(battery_path, 'capacity')
            status_path = os.path.join(battery_path, 'status')

            if os.path.exists(capacity_path):
                with open(capacity_path, 'r') as f:
                    capacity = int(f.read().strip())

                charging_status = 'Unknown'
                if os.path.exists(status_path):
                    with open(status_path, 'r') as f:
                        charging_status = f.read().strip()

                status.values.append(KeyValue(key='capacity', value=f'{capacity}%'))
                status.values.append(KeyValue(key='charging_status', value=charging_status))

                if capacity < self.battery_warning_threshold:
                    status.level = DiagnosticStatus.WARN
                    status.message = f'Low battery: {capacity}% ({charging_status})'
                else:
                    status.level = DiagnosticStatus.OK
                    status.message = f'Battery OK: {capacity}% ({charging_status})'
            else:
                status.level = DiagnosticStatus.OK
                status.message = 'Cannot read battery capacity'

        except Exception as e:
            status.level = DiagnosticStatus.WARN
            status.message = f'Battery check error: {str(e)}'

        return status

    def _check_ptp_service(self) -> DiagnosticStatus:
        """Check PTP service status."""
        status = DiagnosticStatus()
        status.name = '/robo_doctor/PTP/Service'
        status.hardware_id = self.hardware_id

        try:
            result = subprocess.run(
                ['systemctl', 'is-active', 'ptpd'],
                capture_output=True,
                text=True,
                timeout=5
            )
            service_status = result.stdout.strip()
            status.values.append(KeyValue(key='service_status', value=service_status))

            if service_status in ['active', 'activating']:
                status.level = DiagnosticStatus.OK
                status.message = 'PTP service running'
            else:
                status.level = DiagnosticStatus.ERROR
                status.message = f'PTP service not running: {service_status}'

        except Exception as e:
            status.level = DiagnosticStatus.ERROR
            status.message = f'PTP service check error: {str(e)}'

        return status

    def _check_ptp_process(self) -> DiagnosticStatus:
        """Check PTP process is running."""
        status = DiagnosticStatus()
        status.name = '/robo_doctor/PTP/Process'
        status.hardware_id = self.hardware_id

        try:
            result = subprocess.run(
                ['pgrep', '-f', 'ptpd'],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0 and result.stdout.strip():
                pids = result.stdout.strip().split('\n')
                status.level = DiagnosticStatus.OK
                status.message = f'PTP process running (PIDs: {", ".join(pids)})'
                status.values.append(KeyValue(key='pids', value=','.join(pids)))
            else:
                status.level = DiagnosticStatus.ERROR
                status.message = 'PTP process not running'

        except Exception as e:
            status.level = DiagnosticStatus.ERROR
            status.message = f'PTP process check error: {str(e)}'

        return status

    # =========================================================================
    # Medium Checks
    # =========================================================================

    def _check_network_lidar(self) -> DiagnosticStatus:
        """Check LiDAR network connectivity."""
        status = DiagnosticStatus()
        status.name = '/robo_doctor/Network/LiDAR'
        status.hardware_id = self.hardware_id
        status.values.append(KeyValue(key='target_ip', value=self.lidar_ip))

        if self._ping_host(self.lidar_ip):
            status.level = DiagnosticStatus.OK
            status.message = f'LiDAR reachable ({self.lidar_ip})'
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = f'LiDAR not reachable ({self.lidar_ip})'

        return status

    def _check_network_gnss(self) -> DiagnosticStatus:
        """Check GNSS network connectivity."""
        status = DiagnosticStatus()
        status.name = '/robo_doctor/Network/GNSS'
        status.hardware_id = self.hardware_id
        status.values.append(KeyValue(key='target', value=f'{self.gnss_ip}:{self.gnss_port}'))

        if self._check_tcp_connection(self.gnss_ip, self.gnss_port):
            status.level = DiagnosticStatus.OK
            status.message = f'GNSS reachable ({self.gnss_ip}:{self.gnss_port})'
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = f'GNSS not reachable ({self.gnss_ip}:{self.gnss_port})'

        return status

    def _check_host_interface(self) -> DiagnosticStatus:
        """Check host network interface for LiDAR subnet."""
        status = DiagnosticStatus()
        status.name = '/robo_doctor/Network/HostInterface'
        status.hardware_id = self.hardware_id

        try:
            result = subprocess.run(
                ['ip', 'addr', 'show'],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0 and self.host_subnet in result.stdout:
                status.level = DiagnosticStatus.OK
                status.message = f'Host interface configured for {self.host_subnet}.x'
            else:
                status.level = DiagnosticStatus.ERROR
                status.message = f'No interface for {self.host_subnet}.x subnet'

        except Exception as e:
            status.level = DiagnosticStatus.ERROR
            status.message = f'Interface check error: {str(e)}'

        return status

    def _check_device_imu(self) -> DiagnosticStatus:
        """Check IMU device exists."""
        status = DiagnosticStatus()
        status.name = '/robo_doctor/Device/IMU'
        status.hardware_id = self.hardware_id
        status.values.append(KeyValue(key='device_path', value=self.imu_device))

        if os.path.exists(self.imu_device):
            status.level = DiagnosticStatus.OK
            status.message = f'IMU device found ({self.imu_device})'
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = f'IMU device not found ({self.imu_device})'

        return status

    # =========================================================================
    # Heavy Checks
    # =========================================================================

    def _check_ros2_nodes(self) -> list:
        """Check expected ROS2 nodes are running. Returns list of DiagnosticStatus."""
        statuses = []

        try:
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True,
                text=True,
                timeout=10
            )

            if result.returncode != 0:
                # Return error status for all nodes
                for node_name in self.expected_nodes:
                    status = DiagnosticStatus()
                    status.name = f'/robo_doctor/Nodes/{node_name}'
                    status.hardware_id = self.hardware_id
                    status.level = DiagnosticStatus.ERROR
                    status.message = 'Failed to get node list'
                    statuses.append(status)
                return statuses

            running_nodes = result.stdout.strip().split('\n')

            for node_name in self.expected_nodes:
                status = DiagnosticStatus()
                status.name = f'/robo_doctor/Nodes/{node_name}'
                status.hardware_id = self.hardware_id

                found = any(node_name in node for node in running_nodes)
                if found:
                    status.level = DiagnosticStatus.OK
                    status.message = 'Running'
                else:
                    status.level = DiagnosticStatus.ERROR
                    status.message = 'Not running'

                statuses.append(status)

        except subprocess.TimeoutExpired:
            for node_name in self.expected_nodes:
                status = DiagnosticStatus()
                status.name = f'/robo_doctor/Nodes/{node_name}'
                status.hardware_id = self.hardware_id
                status.level = DiagnosticStatus.ERROR
                status.message = 'Node list command timeout'
                statuses.append(status)
        except Exception as e:
            for node_name in self.expected_nodes:
                status = DiagnosticStatus()
                status.name = f'/robo_doctor/Nodes/{node_name}'
                status.hardware_id = self.hardware_id
                status.level = DiagnosticStatus.ERROR
                status.message = f'Check error: {str(e)}'
                statuses.append(status)

        return statuses

    def _check_ros2_topics(self) -> list:
        """Check expected ROS2 topics exist. Returns list of DiagnosticStatus."""
        statuses = []

        try:
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=10
            )

            if result.returncode != 0:
                for topic_name in self.expected_topics:
                    status = DiagnosticStatus()
                    status.name = f'/robo_doctor/Topics{topic_name}'
                    status.hardware_id = self.hardware_id
                    status.level = DiagnosticStatus.ERROR
                    status.message = 'Failed to get topic list'
                    statuses.append(status)
                return statuses

            existing_topics = result.stdout.strip().split('\n')

            for topic_name in self.expected_topics:
                status = DiagnosticStatus()
                status.name = f'/robo_doctor/Topics{topic_name}'
                status.hardware_id = self.hardware_id

                if topic_name in existing_topics:
                    status.level = DiagnosticStatus.OK
                    status.message = 'Exists'
                else:
                    status.level = DiagnosticStatus.ERROR
                    status.message = 'Not found'

                statuses.append(status)

        except subprocess.TimeoutExpired:
            for topic_name in self.expected_topics:
                status = DiagnosticStatus()
                status.name = f'/robo_doctor/Topics{topic_name}'
                status.hardware_id = self.hardware_id
                status.level = DiagnosticStatus.ERROR
                status.message = 'Topic list command timeout'
                statuses.append(status)
        except Exception as e:
            for topic_name in self.expected_topics:
                status = DiagnosticStatus()
                status.name = f'/robo_doctor/Topics{topic_name}'
                status.hardware_id = self.hardware_id
                status.level = DiagnosticStatus.ERROR
                status.message = f'Check error: {str(e)}'
                statuses.append(status)

        return statuses

    def _check_ntrip_status(self) -> DiagnosticStatus:
        """Check NTRIP str2str node status."""
        status = DiagnosticStatus()
        status.name = '/robo_doctor/GNSS/NTRIP'
        status.hardware_id = self.hardware_id

        try:
            result = subprocess.run(
                ['ros2', 'service', 'call', '/ntrip_str2str_node/get_status',
                 'susumu_ros2_interfaces/srv/NtripStatus'],
                capture_output=True,
                text=True,
                timeout=10
            )

            if result.returncode != 0:
                status.level = DiagnosticStatus.WARN
                status.message = 'NTRIP service not available'
                return status

            output = result.stdout

            is_running = 'is_running=True' in output or 'is_running: true' in output

            # Extract bps value
            import re
            bps_match = re.search(r'bps=(\d+)', output)
            bps = int(bps_match.group(1)) if bps_match else 0

            status.values.append(KeyValue(key='is_running', value=str(is_running)))
            status.values.append(KeyValue(key='bps', value=str(bps)))

            # Use bps > 0 as indicator that data is flowing
            if is_running and bps > 0:
                status.level = DiagnosticStatus.OK
                status.message = f'NTRIP data flowing ({bps} bps)'
            elif is_running:
                status.level = DiagnosticStatus.WARN
                status.message = 'NTRIP running but no data'
            else:
                status.level = DiagnosticStatus.ERROR
                status.message = 'NTRIP not running'

        except subprocess.TimeoutExpired:
            status.level = DiagnosticStatus.WARN
            status.message = 'NTRIP status check timeout'
        except Exception as e:
            status.level = DiagnosticStatus.WARN
            status.message = f'NTRIP check error: {str(e)}'

        return status

    def _check_critical_data_flow(self) -> list:
        """Check if critical topics are publishing data. Returns list of DiagnosticStatus."""
        statuses = []

        for topic_name, description in self.critical_data_topics:
            status = DiagnosticStatus()
            status.name = f'/robo_doctor/DataFlow{topic_name}'
            status.hardware_id = self.hardware_id

            # Use longer timeout for slower topics (IMU: ~10Hz, GNSS: ~1Hz)
            if topic_name in ['/imu', '/fix']:
                timeout = 10
            else:
                timeout = 3

            try:
                # First check if topic exists
                result = subprocess.run(
                    ['ros2', 'topic', 'info', topic_name],
                    capture_output=True,
                    text=True,
                    timeout=5
                )

                if result.returncode != 0:
                    status.level = DiagnosticStatus.ERROR
                    status.message = f'{description}: Topic does not exist'
                    statuses.append(status)
                    continue

                # Parse publisher count
                pub_count = 0
                for line in result.stdout.split('\n'):
                    if 'Publisher count:' in line:
                        try:
                            pub_count = int(line.split(':')[1].strip())
                        except ValueError:
                            pass

                status.values.append(KeyValue(key='publisher_count', value=str(pub_count)))

                if pub_count == 0:
                    status.level = DiagnosticStatus.ERROR
                    status.message = f'{description}: No publishers'
                    statuses.append(status)
                    continue

                # Try to receive at least one message
                result = subprocess.run(
                    ['timeout', str(timeout), 'ros2', 'topic', 'echo', topic_name, '--once'],
                    capture_output=True,
                    text=True,
                    timeout=timeout + 2
                )

                if result.returncode == 0 and len(result.stdout.strip()) > 0:
                    status.level = DiagnosticStatus.OK
                    status.message = f'{description}: OK (publishers: {pub_count})'
                elif result.returncode == 124:  # timeout exit code
                    status.level = DiagnosticStatus.ERROR
                    status.message = f'{description}: Timeout ({timeout}s, publishers: {pub_count})'
                else:
                    status.level = DiagnosticStatus.ERROR
                    status.message = f'{description}: No data received (publishers: {pub_count})'

            except subprocess.TimeoutExpired:
                status.level = DiagnosticStatus.ERROR
                status.message = f'{description}: Timeout'
            except Exception as e:
                status.level = DiagnosticStatus.ERROR
                status.message = f'{description}: Error - {str(e)}'

            statuses.append(status)

        return statuses

    def _check_livox_config(self) -> DiagnosticStatus:
        """Validate Livox MID360 config matches current network configuration."""
        status = DiagnosticStatus()
        status.name = '/robo_doctor/Config/Livox'
        status.hardware_id = self.hardware_id

        try:
            import json

            # Find config file
            config_path = None
            for path in self.livox_config_paths:
                if os.path.exists(path):
                    config_path = path
                    break

            if not config_path:
                status.level = DiagnosticStatus.WARN
                status.message = 'MID360_config.json not found'
                return status

            # Get current host IP in 192.168.1.x subnet
            current_ip = self._get_current_host_ip_for_livox()
            if not current_ip:
                status.level = DiagnosticStatus.WARN
                status.message = 'No host IP in 192.168.1.x subnet'
                return status

            # Read config file
            with open(config_path, 'r') as f:
                config = json.load(f)

            # Check if config matches current IP
            host_net_info = config.get('MID360', {}).get('host_net_info', {})
            config_ips = {
                'cmd_data_ip': host_net_info.get('cmd_data_ip'),
                'push_msg_ip': host_net_info.get('push_msg_ip'),
                'point_data_ip': host_net_info.get('point_data_ip'),
                'imu_data_ip': host_net_info.get('imu_data_ip')
            }

            mismatches = []
            for key, ip in config_ips.items():
                if ip != current_ip:
                    mismatches.append(f'{key}={ip}')

            status.values.append(KeyValue(key='current_host_ip', value=current_ip))
            status.values.append(KeyValue(key='config_path', value=config_path))

            if mismatches:
                status.level = DiagnosticStatus.ERROR
                status.message = f'IP mismatch: {", ".join(mismatches)} (expected: {current_ip})'
            else:
                status.level = DiagnosticStatus.OK
                status.message = f'Config OK (host IP: {current_ip})'

        except Exception as e:
            status.level = DiagnosticStatus.WARN
            status.message = f'Config validation error: {str(e)}'

        return status

    def _check_ptp_slave_status(self) -> DiagnosticStatus:
        """Check if PTP slave (Livox LiDAR) is reachable."""
        status = DiagnosticStatus()
        status.name = '/robo_doctor/PTP/Slave'
        status.hardware_id = self.hardware_id
        status.values.append(KeyValue(key='target_ip', value=self.ptp_slave_ip))

        if self._ping_host(self.ptp_slave_ip, timeout=2):
            status.level = DiagnosticStatus.OK
            status.message = f'PTP Slave reachable ({self.ptp_slave_ip})'
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = f'PTP Slave not reachable ({self.ptp_slave_ip})'

        return status

    def _check_ptp_master_status(self) -> DiagnosticStatus:
        """Check if PTP master (local host) is reachable and running."""
        status = DiagnosticStatus()
        status.name = '/robo_doctor/PTP/Master'
        status.hardware_id = self.hardware_id
        status.values.append(KeyValue(key='target_ip', value=self.ptp_master_ip))

        # Check if this is a local IP
        if self._is_local_ip(self.ptp_master_ip):
            # For local master, check if ptpd service is running
            try:
                result = subprocess.run(
                    ['systemctl', 'is-active', 'ptpd'],
                    capture_output=True,
                    text=True,
                    timeout=5
                )
                service_status = result.stdout.strip()
                if service_status in ['active', 'activating']:
                    status.level = DiagnosticStatus.OK
                    status.message = f'PTP Master running locally ({self.ptp_master_ip})'
                else:
                    status.level = DiagnosticStatus.WARN
                    status.message = f'PTP Master service not active ({service_status})'
            except Exception as e:
                status.level = DiagnosticStatus.WARN
                status.message = f'PTP Master check error: {str(e)}'
        else:
            # For remote master, just check connectivity
            if self._ping_host(self.ptp_master_ip, timeout=2):
                status.level = DiagnosticStatus.OK
                status.message = f'PTP Master reachable ({self.ptp_master_ip})'
            else:
                status.level = DiagnosticStatus.WARN
                status.message = f'PTP Master not reachable ({self.ptp_master_ip})'

        return status

    def _get_current_host_ip_for_livox(self) -> Optional[str]:
        """Get the current host IP in 192.168.1.x subnet."""
        try:
            result = subprocess.run(
                ['ip', 'addr', 'show'],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                for line in result.stdout.split('\n'):
                    if 'inet ' in line and '192.168.1.' in line:
                        parts = line.strip().split()
                        for part in parts:
                            if '192.168.1.' in part and '/' in part:
                                return part.split('/')[0]
            return None
        except Exception:
            return None

    def _is_local_ip(self, ip: str) -> bool:
        """Check if IP address is local to this machine."""
        if ip in ['127.0.0.1', 'localhost']:
            return True
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

    # =========================================================================
    # Utility Methods
    # =========================================================================

    def _ping_host(self, host: str, timeout: int = 1) -> bool:
        """Ping a host to check network connectivity."""
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

    def _check_tcp_connection(self, host: str, port: int, timeout: int = 1) -> bool:
        """Check if TCP connection to host:port is possible."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(timeout)
            result = sock.connect_ex((host, port))
            sock.close()
            return result == 0
        except Exception:
            return False


def main(args=None):
    rclpy.init(args=args)
    node = RoboDoctorNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
