#!/usr/bin/env python3
"""
保存済みの地図ファイルを選択してRViz2で表示するツール。
nav2_map_server で地図を配信し、RViz2を起動する。
RViz2を閉じると map_server も自動終了する。
"""
import os
import sys
import subprocess
import glob
import time
import signal
from datetime import datetime
from PySide6.QtWidgets import (
    QApplication, QDialog, QVBoxLayout, QHBoxLayout,
    QPushButton, QListWidget, QListWidgetItem, QLabel
)
from PySide6.QtCore import Qt


ROS_SETUP = ". /opt/ros/humble/setup.bash"
WS_SETUP = ". /home/taro/ros2_ws/install/setup.bash"


class MapSelectorDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("地図ファイルを選択")
        self.setMinimumSize(500, 400)

        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("表示する地図を選択してください:"))

        self.list_widget = QListWidget()
        self.list_widget.setMinimumHeight(300)
        self.list_widget.doubleClicked.connect(self.accept)
        layout.addWidget(self.list_widget)

        btn_layout = QHBoxLayout()
        ok_btn = QPushButton("表示")
        ok_btn.setMinimumHeight(40)
        ok_btn.clicked.connect(self.accept)
        cancel_btn = QPushButton("キャンセル")
        cancel_btn.setMinimumHeight(40)
        cancel_btn.clicked.connect(self.reject)
        btn_layout.addWidget(ok_btn)
        btn_layout.addWidget(cancel_btn)
        layout.addLayout(btn_layout)

        self._load_map_files()

    def _load_map_files(self):
        home = os.path.expanduser("~")
        yaml_files = sorted(glob.glob(os.path.join(home, "map*.yaml")), reverse=True)

        if not yaml_files:
            self.list_widget.addItem("地図ファイルが見つかりません (~/*.yaml)")
            return

        for path in yaml_files:
            name = os.path.basename(path)
            mtime = datetime.fromtimestamp(os.path.getmtime(path)).strftime("%Y-%m-%d %H:%M")
            item = QListWidgetItem(f"{name}  ({mtime})")
            item.setData(Qt.UserRole, path)
            self.list_widget.addItem(item)

        self.list_widget.setCurrentRow(0)

    def get_selected_path(self):
        item = self.list_widget.currentItem()
        if item:
            return item.data(Qt.UserRole)
        return None


def run(cmd, wait=False):
    if wait:
        return subprocess.run(["/bin/bash", "-c", cmd])
    else:
        return subprocess.Popen(["/bin/bash", "-c", cmd])


def main():
    app = QApplication(sys.argv)

    dialog = MapSelectorDialog()
    if dialog.exec() != QDialog.Accepted:
        sys.exit(0)

    map_path = dialog.get_selected_path()
    if not map_path:
        sys.exit(0)

    print(f"選択した地図: {map_path}")

    # map_server を起動
    map_server_proc = run(
        f"{ROS_SETUP} && {WS_SETUP} && "
        f"ros2 run nav2_map_server map_server --ros-args "
        f"-p yaml_filename:={map_path} -p use_sim_time:=false"
    )

    # map_server の起動を待ってライフサイクル遷移
    time.sleep(2)
    run(f"{ROS_SETUP} && ros2 lifecycle set /map_server configure", wait=True)
    time.sleep(0.5)
    run(f"{ROS_SETUP} && ros2 lifecycle set /map_server activate", wait=True)

    # RViz2 設定ファイルのパスを取得
    result = subprocess.run(
        ["/bin/bash", "-c", f"{ROS_SETUP} && {WS_SETUP} && ros2 pkg prefix susumu_robo"],
        capture_output=True, text=True
    )
    pkg_prefix = result.stdout.strip()
    rviz_config = os.path.join(pkg_prefix, "share", "susumu_robo", "config", "slam.rviz")

    # RViz2 を起動し、終了を待つ
    rviz_proc = run(f"{ROS_SETUP} && {WS_SETUP} && ros2 run rviz2 rviz2 -d {rviz_config}")
    rviz_proc.wait()  # RViz2 が閉じられるまで待機

    # RViz2 終了後に map_server を終了
    print("RViz2 closed. Stopping map_server...")
    map_server_proc.terminate()
    try:
        map_server_proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        map_server_proc.kill()

    sys.exit(0)


if __name__ == "__main__":
    main()
