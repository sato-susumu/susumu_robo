"""tts_option + stt_option_debug + agent_gui + turtlesim_debug を一括起動する。"""
import ctypes
import subprocess
import time

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

_DEBUG_DIR = '/home/taro/ros2_ws/src/susumu_asr/debug'
_ENV_FILE = '/home/taro/ros2_ws/src/susumu_asr/.env'

CPU_IMAGE = "ghcr.io/aivis-project/aivisspeech-engine:cpu-latest"
GPU_IMAGE = "ghcr.io/aivis-project/aivisspeech-engine:nvidia-latest"


def _has_nvidia_gpu() -> bool:
    try:
        nvml = ctypes.CDLL("libnvidia-ml.so.1")
        if nvml.nvmlInit_v2() != 0:
            return False
        count = ctypes.c_uint(0)
        result = nvml.nvmlDeviceGetCount_v2(ctypes.byref(count))
        nvml.nvmlShutdown()
        return result == 0 and count.value > 0
    except OSError:
        return False


def start_docker_and_wait(context):
    data_dir = "~/.local/share/AivisSpeech-Engine"
    expanded = subprocess.run(
        ["bash", "-c", f"echo {data_dir}"],
        capture_output=True, text=True
    ).stdout.strip()

    subprocess.run(["bash", "-c", f"mkdir -p {expanded}"], check=True)
    subprocess.run(["docker", "stop", "aivis_engine"], capture_output=True)

    use_gpu = _has_nvidia_gpu()
    if use_gpu:
        image = GPU_IMAGE
        docker_cmd = [
            "docker", "run", "--pull", "always", "--rm", "-d",
            "--gpus", "all",
            "-p", "10101:10101",
            "-v", f"{expanded}:/home/user/.local/share/AivisSpeech-Engine-Dev",
            "--name", "aivis_engine",
            image,
        ]
        print("[audio_option_debug] NVIDIA GPU detected. Starting GPU image: " + image)
    else:
        image = CPU_IMAGE
        docker_cmd = [
            "docker", "run", "--pull", "always", "--rm", "-d",
            "-p", "10101:10101",
            "-v", f"{expanded}:/home/user/.local/share/AivisSpeech-Engine-Dev",
            "--name", "aivis_engine",
            image,
        ]
        print("[audio_option_debug] No NVIDIA GPU detected. Starting CPU image: " + image)

    subprocess.run(docker_cmd, check=True)

    print("[audio_option_debug] Waiting for AivisSpeech Engine to be ready...")
    for _ in range(120):
        result = subprocess.run(
            ["curl", "--silent", "--max-time", "3", "http://localhost:10101/version"],
            capture_output=True, text=True
        )
        if result.returncode == 0 and result.stdout.strip():
            print("[audio_option_debug] AivisSpeech Engine is ready.")
            return []
        time.sleep(5)

    print("[audio_option_debug] WARNING: AivisSpeech Engine did not respond within 600s.")
    return []


def generate_launch_description():
    susumu_robo_share = FindPackageShare("susumu_robo")
    susumu_agent_share = FindPackageShare("susumu_agent")

    tts = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([susumu_robo_share, "launch", "tts_option.launch.py"])
        )
    )

    stt_debug = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([susumu_robo_share, "launch", "stt_option_debug.launch.py"])
        )
    )

    agent_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([susumu_agent_share, "launch", "gui.launch.py"])
        )
    )

    turtlesim_debug = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([susumu_agent_share, "launch", "turtlesim_debug.launch.py"])
        )
    )

    return LaunchDescription([
        LogInfo(msg="audio_option_debug: tts_option + stt_option_debug + agent_gui + turtlesim_debug を起動します..."),
        OpaqueFunction(function=start_docker_and_wait),
        tts,
        stt_debug,
        agent_gui,
        turtlesim_debug,
    ])
