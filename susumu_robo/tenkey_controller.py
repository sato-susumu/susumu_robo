
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from evdev import InputDevice, categorize, ecodes

class KeyboardTurtleControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_turtle_control')

        self.declare_parameter('keyboard_device_path', '/dev/input/by-id/usb-05a4_Tenkey_Keyboard-event-kbd')
        self.declare_parameter('cmd_vel_topic', '/turtle1/cmd_vel')
        self.declare_parameter('linear_speed', 1.0)
        self.declare_parameter('angular_speed', 1.0)
        
        self.key_action_map = {
            'KEY_KP8': self.move_forward,
            'KEY_KP2': self.move_backward,
            'KEY_KP4': self.turn_left,
            'KEY_KP6': self.turn_right,
            'KEY_KP5': self.move_stop
        }

        self.keyboard_device_path = self.get_parameter('keyboard_device_path').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value

        self.device = InputDevice(self.keyboard_device_path)
        self.velocity_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.timer = self.create_timer(0.1, self.read_keyboard_events)
        self.get_logger().info(f"Monitoring keyboard at {self.keyboard_device_path}")

    def read_keyboard_events(self):
        for event in self.device.read_loop():
            if event.type == ecodes.EV_KEY:
                self.process_key_event(event)
                break

    def process_key_event(self, event):
        key_event = categorize(event)
        if key_event.keycode in self.key_action_map and key_event.keystate == key_event.key_down:
            self.key_action_map[key_event.keycode]()

    def move_forward(self):
        twist = Twist()
        twist.linear.x = self.linear_speed
        self.velocity_publisher.publish(twist)
        self.get_logger().info("Moving forward")

    def move_backward(self):
        twist = Twist()
        twist.linear.x = -self.linear_speed
        self.velocity_publisher.publish(twist)
        self.get_logger().info("Moving backward")

    def turn_left(self):
        twist = Twist()
        twist.angular.z = self.angular_speed
        self.velocity_publisher.publish(twist)
        self.get_logger().info("Turning left")

    def turn_right(self):
        twist = Twist()
        twist.angular.z = -self.angular_speed
        self.velocity_publisher.publish(twist)
        self.get_logger().info("Turning right")

    def move_stop(self):
        twist = Twist()
        self.velocity_publisher.publish(twist)
        self.get_logger().info("Stopping movement")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTurtleControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
