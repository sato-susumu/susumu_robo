import time

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from speak_ros_interfaces.action import Speak

NODE_NAME = 'dummy_speak_action_server'
ACTION_NAME = '/speak'


class DummySpeakActionServer(Node):

    def __init__(self) -> None:
        super().__init__(NODE_NAME)
        self.declare_parameter('seconds_per_char', 0.08)
        self.declare_parameter('max_delay_seconds', 5.0)
        self._action_server = ActionServer(
            self,
            Speak,
            ACTION_NAME,
            execute_callback=self._execute,
        )
        self.get_logger().info(
            f"Dummy speak action server ready on '{ACTION_NAME}'."
        )

    def _execute(self, goal_handle):
        text = goal_handle.request.text
        self.get_logger().info(f'[dummy speak] "{text}"')
        print(f'[dummy speak] {text}', flush=True)

        spc = float(self.get_parameter('seconds_per_char').value)
        max_delay = float(self.get_parameter('max_delay_seconds').value)
        delay = min(len(text) * spc, max_delay)

        start = time.monotonic()
        while time.monotonic() - start < delay:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = Speak.Result()
                result.termination_reason = Speak.Result.CANCELLED
                return result
            time.sleep(0.05)

        goal_handle.succeed()
        result = Speak.Result()
        result.termination_reason = Speak.Result.COMPLETED
        result.total_samples_played = 0
        return result


def main() -> None:
    rclpy.init()
    node = DummySpeakActionServer()
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
