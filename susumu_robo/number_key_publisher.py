from __future__ import annotations

from typing import Dict, Optional, List

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String
from evdev import InputDevice, categorize, ecodes, InputEvent
from evdev.events import KeyEvent

class NumberKeyPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__('number_key_publisher')

        self.declare_parameter('keyboard_device_path', '/dev/input/by-id/usb-INSTANT_USB_GAMING_MOUSE-if01-event-kbd')
        self.declare_parameter('key_event_topic', 'key_event')  # ROS2標準: 名前空間なし、スラッシュなし

        # 通常の数字キー（テンキーではない）のマッピング
        self.key_map: Dict[str, str] = {
            'KEY_KP1': '1',
            'KEY_KP2': '2',
            'KEY_KP3': '3',
            'KEY_KP4': '4'
        }

        self.keyboard_device_path: str = self.get_parameter('keyboard_device_path').get_parameter_value().string_value
        self.key_event_topic: str = self.get_parameter('key_event_topic').get_parameter_value().string_value

        self.device: InputDevice = InputDevice(self.keyboard_device_path)
        self.key_publisher: Publisher[String] = self.create_publisher(String, self.key_event_topic, 10)
        self.timer = self.create_timer(0.1, self.read_keyboard_events)
        self.get_logger().info(f"Monitoring keyboard at {self.keyboard_device_path}")
        self.get_logger().info(f"Publishing to topic: {self.key_event_topic}")

    def read_keyboard_events(self) -> None:
        for event in self.device.read_loop():
            if event.type == ecodes.EV_KEY:
                self.process_key_event(event)
                break

    def process_key_event(self, event: InputEvent) -> None:
        key_event: KeyEvent = categorize(event)
        # キーが押されたときのみ反応
        if key_event.keycode in self.key_map and key_event.keystate == key_event.key_down:
            key_value: str = self.key_map[key_event.keycode]
            self.publish_key_event(key_value)

    def publish_key_event(self, key_value: str) -> None:
        msg: String = String()
        msg.data = key_value
        self.key_publisher.publish(msg)
        self.get_logger().info(f"Published key event: {key_value}")

def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node: NumberKeyPublisherNode = NumberKeyPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()