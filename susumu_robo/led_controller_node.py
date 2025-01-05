#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from susumu_ros2_interfaces.msg import LED
from rclpy.qos import QoSProfile, ReliabilityPolicy

class LEDControllerNode(Node):
    def __init__(self):
        super().__init__('led_controller_node')

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.subscription = self.create_subscription(
            Bool,
            '/scan_in_range',
            self.scan_in_range_callback,
            qos_profile
        )
        self.subscription  # 未使用警告を防ぐため

        self.publisher = self.create_publisher(LED, '/led', 10)

        self.previous_state = False
        self.get_logger().info("LED Controller Node が起動しました。")

    def scan_in_range_callback(self, msg: Bool):
        current_state = msg.data
        self.get_logger().debug(f'Received /scan_in_range: {current_state}')

        if not self.previous_state and current_state:
            self.get_logger().info('状態が False から True に変化しました。LED点灯を開始します。')
            self.publish_led_command(
                pattern='BLINK',
                color1='yellow',
                color2='black',
                duration=60.0,
                priority=1,
                decay_rate=0.8,
                speed=3.0
            )
        elif self.previous_state and not current_state:
            self.get_logger().info('状態が True から False に変化しました。LEDをオフにします。')
            self.publish_led_command(
                pattern='SOLID',
                color1='black',
                color2='black',
                duration=1.0,
                priority=2,
                decay_rate=0.0,
                speed=1.0
            )

        self.previous_state = current_state

    def publish_led_command(self, pattern, color1, color2, duration, priority, decay_rate, speed):
        led_msg = LED()
        led_msg.pattern = pattern
        led_msg.color1 = color1
        led_msg.color2 = color2
        led_msg.duration = duration  # 秒数
        led_msg.priority = priority
        led_msg.decay_rate = decay_rate
        led_msg.speed = speed

        self.publisher.publish(led_msg)
        self.get_logger().debug(
            f'LEDコマンドをパブリッシュしました: pattern={pattern}, color1={color1}, duration={duration}'
        )

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LEDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
