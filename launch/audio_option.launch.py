import subprocess
import time

from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, TimerAction
from launch_ros.actions import Node


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

    # CPU版コンテナ起動（--pull always で常に最新イメージを取得）
    subprocess.run([
        "docker", "run", "--pull", "always", "--rm", "-d",
        "-p", "10101:10101",
        "-v", f"{expanded}:/home/user/.local/share/AivisSpeech-Engine-Dev",
        "--name", "aivis_engine",
        "ghcr.io/aivis-project/aivisspeech-engine:cpu-latest",
    ], check=True)

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

        # speak_ros ノード（AivisSpeech CPU版、阿井田 茂 ノーマルをデフォルト）
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
    ])
