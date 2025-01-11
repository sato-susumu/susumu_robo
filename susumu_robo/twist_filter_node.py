# このソースコードは twist_filter_node.cpp をPythonに移植し、改変したものです。
#
# オリジナルのソースコードとライセンス
#   https://github.com/whill-labs/ros2_whill_applications/blob/main/whill_auto_stop/src/twist_filter_node.cpp
#   Copyright (c) 2024 WHILL, Inc.
#   Released under the MIT license
#   https://opensource.org/licenses/mit-license.php

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy

class TwistFilterNode(Node):
    def __init__(self):
        super().__init__('twist_filter_node')

        # 初期化
        self.in_range = False

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.twist_subscriber_ = self.create_subscription(
            Twist,
            'input_twist',
            self.twist_callback,
            qos_profile
        )

        self.bool_subscriber_ = self.create_subscription(
            Bool,
            'enable',
            self.bool_callback,
            qos_profile
        )

        self.twist_publisher_ = self.create_publisher(
            Twist,
            'output_twist',
            qos_profile
        )

        self.get_logger().info("Twist Filter Node has been started.")

    def twist_callback(self, msg):
        """
        Twistメッセージを受信し、条件に応じてパブリッシュする.
        """
        if self.in_range:
            new_msg = Twist()
            new_msg.angular = msg.angular
            new_msg.linear.x = 0.0
            new_msg.linear.y = 0.0
            new_msg.linear.z = 0.0
            self.twist_publisher_.publish(new_msg)
            self.get_logger().debug("Publishing Twist message with linear.x, linear.y, linear.z set to 0.0.")
        else:
            self.twist_publisher_.publish(msg)
            self.get_logger().debug("Publishing Twist message.")

    def bool_callback(self, msg):
        """
        Boolメッセージを受信し、パブリッシュの有効フラグを設定する.
        """
        self.in_range = msg.data
        self.get_logger().debug(f"Enable flag set to: {'True' if self.in_range else 'False'}")


def main(args=None):
    rclpy.init(args=args)
    node = TwistFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
