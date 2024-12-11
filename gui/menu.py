import sys
import platform
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QTextEdit, QScrollArea, QSizePolicy, QScroller, QFrame
)
from PySide6.QtCore import QProcess
from PySide6.QtGui import QFont
from typing import Optional, Dict


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()

        self.setWindowTitle("ROS2 GUI App")
        self.setGeometry(100, 100, 800, 600)

        # アイテム管理
        self.processes: Dict[str, QProcess] = {}
        self.item_buttons: Dict[str, QPushButton] = {}
        self.current_selected_item: Optional[str] = None

        # UIセットアップ
        self.setup_ui()
        self.add_all_items()

    def setup_ui(self) -> None:
        """UIレイアウトを構築する関数"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        layout = QHBoxLayout(central_widget)
        layout.setSpacing(5)

        # 左側レイアウト設定
        left_main_layout = QVBoxLayout()
        left_main_layout.setContentsMargins(0, 0, 0, 0)
        layout.addLayout(left_main_layout)
        layout.setStretch(0, 1)

        # 左側: スクロールエリアと折りたたみカテゴリ
        scroll_area = QScrollArea()
        QScroller.grabGesture(scroll_area.viewport(), QScroller.LeftMouseButtonGesture)
        scroll_area.setWidgetResizable(True)
        scroll_area.setFixedWidth(180)
        scroll_widget = QWidget()
        self.left_layout = QVBoxLayout(scroll_widget)
        self.left_layout.setSpacing(5)
        self.left_layout.setContentsMargins(0, 0, 0, 0)
        scroll_area.setWidget(scroll_widget)
        left_main_layout.addWidget(scroll_area)

        # 左側: ストップボタン
        self.stop_button = QPushButton("Stop Selected Item")
        self.stop_button.setEnabled(False)
        self.stop_button.setStyleSheet(self.get_stop_button_style(False))
        self.stop_button.clicked.connect(self.stop_selected_item)
        left_main_layout.addWidget(self.stop_button)

        # 右側レイアウト設定
        self.right_layout = QVBoxLayout()
        self.right_layout.setContentsMargins(0, 0, 0, 0)
        self.right_layout.setSpacing(5)
        layout.addLayout(self.right_layout)
        layout.setStretch(1, 4)

        self.output_display = QTextEdit()
        self.output_display.setReadOnly(True)
        self.output_display.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.output_display.setStyleSheet("background-color: black; color: white;")
        self.set_output_font()

        self.right_layout.addWidget(self.output_display)

    def set_output_font(self) -> None:
        """OSに応じて出力エリアの等幅フォントを設定"""
        system_name: str = platform.system()
        if system_name == "Windows":
            font = QFont("MS Gothic")  # Windows向け
        elif system_name == "Linux":
            font = QFont("Noto Sans Mono CJK JP")  # Ubuntu向け
        else:
            font = QFont("Monospace")  # Fallback等幅フォント

        font.setStyleHint(QFont.Monospace)
        font.setFixedPitch(True)
        font.setPointSize(14)
        self.output_display.setFont(font)

    def add_all_items(self) -> None:
        """アイテムをカテゴリごとに追加する関数"""
        self.add_category("Launch", [
            ("fast_lio", "ros2 launch fast_lio mapping.launch.py"),
            ("avatar_ros", "ros2 run avatar_ros avatar_node"),
        ])

        self.add_category("OS Tools", [
            ("jstest-gtk", "jstest-gtk"),
            ("top", "gnome-terminal -- top"),
            ("nvidia-smi", "nvidia-smi"),
            ("nvtop", "gnome-terminal -- nvtop"),
            ("reboot", "sudo reboot"),
        ])

        self.add_category("ROS2", [
            ("ros2 bag record", "ros2 bag record --all -o output.bag"),
            ("ros2 node list", "ros2 node list"),
            ("ros2 topic list", "ros2 topic list"),
            ("ros2 action list", "ros2 action list"),
            ("ros2 doctor", "ros2 doctor"),
            ("ros2 topic pub", "ros2 topic pub /chatter std_msgs/msg/String \"data: 'Hello ROS 2'\" --rate 1"),
            ("ros2 topic echo", "ros2 topic echo /cmd_vel"),
            ("ros2 topic echo csv", "ros2 topic echo /cmd_vel --csv"),
            ("ros2 topic echo filter", "ros2 topic echo /cmd_vel --filter 'abs(m.linear.x)<0.2'"),
        ])

        self.add_category("RQT Tools", [
            ("rqt", "rqt"),
            ("rqt_graph", "rqt_graph"),
            ("rqt_plot", "ros2 run rqt_plot rqt_plot"),
            ("rqt_plot (cmd_vel)", "ros2 run rqt_plot rqt_plot /cmd_vel/linear/x /cmd_vel/angular/z"),
            ("rqt_console", "ros2 run rqt_console rqt_console"),
            ("rqt_tf_tree", "ros2 run rqt_tf_tree rqt_tf_tree"),
            ("rqt_robot_steering", "ros2 run rqt_robot_steering rqt_robot_steering"),
        ])

        self.add_category("TF", [
            ("rqt_tf_tree", "ros2 run rqt_tf_tree rqt_tf_tree"),
            ("tf2_tools", "ros2 run tf2_tools view_frames"),
            ("tf2_monitor", "ros2 run tf2_ros tf2_monitor"),
        ])

        self.add_category("Service", [
            ("service all log",
             "systemctl list-unit-files --type=service --no-pager | grep -o 'ros2[^ ]*.service' | xargs -I{} sudo systemctl status {} --no-pager"),
            ("service all status",
             "sudo systemctl list-units --type=service | grep -e ros2 -e UNIT; sudo systemctl list-unit-files --type=service | grep -e ros2 -e UNIT"),

            ("start ros2_bringup", "sudo systemctl start ros2_bringup"),
            ("start ros2_mid360", "sudo systemctl start ros2_mid360"),
            ("start ros2_ddsm115", "sudo systemctl start ros2_ddsm115"),

            ("stop ros2_bringup", "sudo systemctl stop ros2_bringup"),
            ("stop ros2_mid360", "sudo systemctl stop ros2_mid360"),
            ("stop ros2_ddsm115", "sudo systemctl stop ros2_ddsm115"),

            ("enable ros2_bringup", "sudo systemctl enable ros2_bringup"),
            ("enable ros2_mid360", "sudo systemctl enable ros2_mid360"),
            ("enable ros2_ddsm115", "sudo systemctl enable ros2_ddsm115"),

            ("disable ros2_bringup", "sudo systemctl disable ros2_bringup"),
            ("disable ros2_mid360", "sudo systemctl disable ros2_mid360"),
            ("disable ros2_ddsm115", "sudo systemctl disable ros2_ddsm115"),
        ])

    def add_category(self, category_name: str, items: list) -> None:
        """カテゴリごとにボタンを折りたたみメニューに追加"""
        category_button = QPushButton(category_name)
        category_button.setCheckable(True)
        category_button.setStyleSheet("background-color: lightgray; font-weight: bold;")

        # 初期状態で展開（チェック状態）にする
        category_button.setChecked(True)

        # 子アイテムのコンテナ（折りたたみ用）
        category_frame = QFrame()
        category_layout = QVBoxLayout(category_frame)
        category_layout.setContentsMargins(10, 5, 0, 5)

        # 初期状態は可視
        category_frame.setVisible(True)

        # 親ボタンのクリックで表示/非表示を切り替え
        category_button.toggled.connect(lambda checked: category_frame.setVisible(checked))

        self.left_layout.addWidget(category_button)
        self.left_layout.addWidget(category_frame)

        # 各アイテムを追加
        for name, command in items:
            self.add_item_button(category_layout, name, command)

    def add_item_button(self, layout, name: str, command: str) -> None:
        """アイテムボタンを指定レイアウトに追加する"""
        button = QPushButton(name)
        button.setMinimumHeight(40)
        button.setStyleSheet("background-color: white;")
        button.clicked.connect(lambda: self.select_item(name, command))
        layout.addWidget(button)
        self.item_buttons[name] = button

    def get_stop_button_style(self, enabled: bool) -> str:
        """ストップボタンのスタイルを返す"""
        if enabled:
            return "background-color: red; color: white; font-weight: bold;"
        else:
            return "background-color: lightgray; color: gray;"

    def select_item(self, name: str, command: str) -> None:
        """アイテムを選択し、起動状態にする"""
        self.current_selected_item = name
        self.update_button_styles()
        self.stop_button.setEnabled(name in self.processes)

        # 出力エリアの初期化
        self.output_display.clear()

        if name not in self.processes:  # 初回起動の場合
            self.output_display.append(f"Starting command: {command}\n")
            self.start_process(name, command)
        else:  # 既に起動済みの場合
            self.output_display.append(f"Showing output for {name}...\n")

        self.display_selected_item_output(name)

    def start_process(self, name: str, command: str) -> None:
        """アイテムを起動する"""
        process = QProcess()
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(lambda: self.display_selected_item_output(name))
        process.finished.connect(lambda: self.on_process_finished(name))
        process.start("/bin/bash", ["-c", f". /opt/ros/humble/setup.bash && {command}"])

        self.processes[name] = process
        self.item_buttons[name].setStyleSheet("background-color: lightgreen;")

    def display_selected_item_output(self, name: str) -> None:
        """選択中のアイテムの出力を表示"""
        if self.current_selected_item == name and name in self.processes:
            process = self.processes[name]
            output = process.readAllStandardOutput().data().decode()
            if output:
                self.output_display.append(output)

    def on_process_finished(self, name: str) -> None:
        """プロセス終了時に状態をリセットする"""
        if name in self.processes:
            del self.processes[name]
            self.item_buttons[name].setStyleSheet("background-color: white;")

        if self.current_selected_item == name:
            self.current_selected_item = None
            self.output_display.append(f"{name} has stopped.\n")
            self.stop_button.setEnabled(False)
            self.stop_button.setStyleSheet(self.get_stop_button_style(False))

    def stop_selected_item(self) -> None:
        """選択中のアイテムを停止する"""
        if self.current_selected_item and self.current_selected_item in self.processes:
            process = self.processes[self.current_selected_item]
            self.output_display.append(f"Stopping {self.current_selected_item}...\n")
            process.kill()
            self.on_process_finished(self.current_selected_item)

    def update_button_styles(self) -> None:
        """選択状態と起動状態のボタンを更新"""
        for name, button in self.item_buttons.items():
            if name == self.current_selected_item:
                button.setStyleSheet("background-color: lightblue;")
            elif name in self.processes:
                button.setStyleSheet("background-color: lightgreen;")
            else:
                button.setStyleSheet("background-color: white;")

        # ストップボタンの有効/無効状態とスタイルを更新
        enabled = self.current_selected_item in self.processes
        self.stop_button.setEnabled(enabled)
        self.stop_button.setStyleSheet(self.get_stop_button_style(enabled))


def main() -> None:
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
