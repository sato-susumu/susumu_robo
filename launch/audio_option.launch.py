import ctypes
import subprocess
import time

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

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

    # データディレクトリ作成
    subprocess.run(["bash", "-c", f"mkdir -p {expanded}"], check=True)

    # 既存コンテナが残っていれば停止
    subprocess.run(
        ["docker", "stop", "aivis_engine"],
        capture_output=True
    )

    # GPU検出してイメージ・起動オプションを切り替え
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
        print("[audio_option] NVIDIA GPU detected. Starting GPU image: " + image)
    else:
        image = CPU_IMAGE
        docker_cmd = [
            "docker", "run", "--pull", "always", "--rm", "-d",
            "-p", "10101:10101",
            "-v", f"{expanded}:/home/user/.local/share/AivisSpeech-Engine-Dev",
            "--name", "aivis_engine",
            image,
        ]
        print("[audio_option] No NVIDIA GPU detected. Starting CPU image: " + image)

    subprocess.run(docker_cmd, check=True)

    # 起動完了待ち（最大600秒）
    print("[audio_option] Waiting for AivisSpeech Engine to be ready...")
    for _ in range(120):
        result = subprocess.run(
            ["curl", "--silent", "--max-time", "3",
             "http://localhost:10101/version"],
            capture_output=True, text=True
        )
        if result.returncode == 0 and result.stdout.strip():
            print("[audio_option] AivisSpeech Engine is ready.")
            return []
        time.sleep(5)

    print("[audio_option] WARNING: AivisSpeech Engine did not respond within 600s.")
    return []


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=start_docker_and_wait),

        # speak_ros ノード（AivisSpeech、阿井田 茂 ノーマルをデフォルト）
        Node(
            package="speak_ros",
            executable="speak_ros_node",
            name="speak_ros",
            output="screen",
            parameters=[{
                "plugin_name": "aivis_plugin::AivisPlugin",
                "aivis_plugin/speaker": 1310138976,  # 阿井田 茂 ノーマル
            }],
        ),

        # /to_human → /speak ブリッジ
        Node(
            package="susumu_robo",
            executable="to_human_2_speak_ros",
            name="to_human_2_speak_ros",
            output="screen",
        ),
    ])
