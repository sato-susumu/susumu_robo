import sys
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QTextEdit
from PySide6.QtCore import QProcess


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("ROS2 GUI App")
        self.setGeometry(100, 100, 800, 600)

        # UIレイアウト構築
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        layout = QHBoxLayout(central_widget)
        self.left_layout = QVBoxLayout()
        self.right_layout = QVBoxLayout()
        layout.addLayout(self.left_layout)
        layout.addLayout(self.right_layout)

        # 出力表示領域
        self.output_display = QTextEdit()
        self.output_display.setReadOnly(True)
        self.output_display.setFixedHeight(300)

        # アイテム管理
        self.processes = {}
        self.item_buttons = {}
        self.current_selected_item = None

        # アイテムの追加
        self.add_item_button("fast_lio", "ros2 launch fast_lio mapping.launch.py")
        self.add_item_button("rosbag", "ros2 bag record --all -o output.bag")
        self.add_item_button("rqt", "rqt")
        self.add_item_button("avatar_ros", "ros2 run avatar_ros avatar_node")
        self.add_item_button("nvidia-smi", "nvidia-smi")
        self.add_item_button("ros2_node_list", "ros2 node list")
        self.add_item_button("ros2_topic_list", "ros2 topic list")
        self.add_item_button("rqt_graph", "rqt_graph")
        self.add_item_button("ros2_topic_pub_chatter", "ros2 topic pub /chatter std_msgs/msg/String \"data: 'Hello ROS 2'\" --rate 1")

        # ストップボタン
        self.stop_button = QPushButton("Stop Selected Item")
        self.stop_button.setEnabled(False)
        self.stop_button.setStyleSheet(self.get_stop_button_style(False))  # 初期スタイル
        self.stop_button.clicked.connect(self.stop_selected_item)
        self.left_layout.addWidget(self.stop_button)

        self.right_layout.addWidget(self.output_display)

    def get_stop_button_style(self, enabled):
        """ストップボタンのスタイルを返す"""
        if enabled:
            return "background-color: red; color: white; font-weight: bold;"
        else:
            return "background-color: lightgray; color: gray;"

    def add_item_button(self, name, command):
        """アイテムボタンを追加する"""
        button = QPushButton(name)
        button.setMinimumHeight(50)
        button.setStyleSheet("background-color: white;")
        button.clicked.connect(lambda: self.select_item(name, command))
        self.left_layout.addWidget(button)
        self.item_buttons[name] = button

    def select_item(self, name, command):
        """アイテムを選択し、起動状態にする"""
        self.current_selected_item = name
        self.update_button_styles()
        self.stop_button.setEnabled(name in self.processes)

        if name not in self.processes:  # プロセス未起動の場合は起動する
            self.start_process(name, command)

        # 選択したアイテムの出力を表示する
        self.output_display.clear()
        self.output_display.append(f"Showing output for {name}...\n")
        self.display_selected_item_output(name)

    def start_process(self, name, command):
        """アイテムを起動する"""
        process = QProcess()
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(lambda: self.display_selected_item_output(name))
        process.finished.connect(lambda: self.on_process_finished(name))
        process.start("/bin/bash", ["-c", f". /opt/ros/humble/setup.bash && {command}"])

        self.processes[name] = process
        self.item_buttons[name].setStyleSheet("background-color: lightgreen;")

    def display_selected_item_output(self, name):
        """選択中のアイテムの出力を表示"""
        if self.current_selected_item == name and name in self.processes:
            process = self.processes[name]
            output = process.readAllStandardOutput().data().decode()
            if output:
                self.output_display.append(output)

    def on_process_finished(self, name):
        """プロセス終了時に状態をリセットする"""
        if name in self.processes:
            del self.processes[name]
            self.item_buttons[name].setStyleSheet("background-color: white;")

        if self.current_selected_item == name:
            self.current_selected_item = None
            self.output_display.append(f"{name} has stopped.\n")
            self.stop_button.setEnabled(False)
            self.stop_button.setStyleSheet(self.get_stop_button_style(False))

    def stop_selected_item(self):
        """選択中のアイテムを停止する"""
        if self.current_selected_item and self.current_selected_item in self.processes:
            process = self.processes[self.current_selected_item]
            self.output_display.append(f"Stopping {self.current_selected_item}...\n")
            process.kill()  # 強制終了
            self.on_process_finished(self.current_selected_item)

    def update_button_styles(self):
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


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
