import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    devices = [dev for dev in os.listdir("/dev") if dev.startswith("ttyACM") or dev.startswith("ttyUSB")]
    devices_str = " ".join([f"/dev/{dev}" for dev in devices])

    micro_ros_agent_node = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        namespace="micro_ros",
        arguments=["multiserial", "--devs", f"{devices_str}", "--verbose", "4"]
    )

    return LaunchDescription([
        micro_ros_agent_node,
    ])
