import re
from typing import List

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String as StdString
from speak_ros_interfaces.action import Speak

DEFAULT_NODE_NAME = 'to_human_2_speak_ros'
DEFAULT_INPUT_TOPIC = '/to_human'
DEFAULT_ACTION_NAME = '/speak'
DELIMITERS = '、。！？'


class ToHuman2SpeakRosNode(Node):
    def __init__(self) -> None:
        super().__init__(DEFAULT_NODE_NAME)
        self.declare_parameter('input_topic', DEFAULT_INPUT_TOPIC)
        topic: str = self.get_parameter('input_topic').value

        self._action_client = ActionClient(self, Speak, DEFAULT_ACTION_NAME)
        self.get_logger().info(f"Waiting for action server '{DEFAULT_ACTION_NAME}'...")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server ready.")

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(StdString, topic, self._on_to_human, qos_profile=qos)

        self._text_queue: List[str] = []
        self._is_processing = False

        self.get_logger().info(f"Node '{DEFAULT_NODE_NAME}' started. Listening on '{topic}'")

    def _split_text(self, text: str) -> List[str]:
        pattern = f'([^{DELIMITERS}]+[{DELIMITERS}]+|[^{DELIMITERS}]+$)'
        return re.findall(pattern, text)

    def _on_to_human(self, msg: StdString) -> None:
        self.get_logger().info(f"Received: {msg.data}")
        self._text_queue.extend(self._split_text(msg.data))
        if not self._is_processing:
            self._process_next()

    def _process_next(self) -> None:
        if not self._text_queue:
            self._is_processing = False
            return
        self._is_processing = True
        text = self._text_queue.pop(0)
        goal = Speak.Goal()
        goal.text = text
        self.get_logger().info(f'Sending to speak_ros: "{text}"')
        send_goal_future = self._action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by speak_ros')
            self._process_next()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        result = future.result().result
        reason = result.termination_reason
        if reason == Speak.Result.COMPLETED:
            self.get_logger().info('speak_ros: completed')
        elif reason == Speak.Result.CANCELLED:
            self.get_logger().warning('speak_ros: cancelled')
        else:
            self.get_logger().error('speak_ros: error')
        self._process_next()


def main() -> None:
    rclpy.init()
    node = ToHuman2SpeakRosNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
