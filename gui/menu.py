import sys
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QTextEdit
from PySide6.QtCore import QProcess
import os
import datetime


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("ROS2 GUI App")
        self.setGeometry(100, 100, 800, 600)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        left_layout = QVBoxLayout()
        right_layout = QVBoxLayout()

        layout = QHBoxLayout(central_widget)
        layout.addLayout(left_layout)
        layout.addLayout(right_layout)

        # 左レイアウトにボタンを追加
        self._start_fast_lio_button = self._create_button("Start fast_lio launch", left_layout, self.start_fast_lio_launch)
        self._start_rosbag_button = self._create_button("Start recording rosbag", left_layout, self.toggle_rosbag_recording)
        self._create_button("", left_layout, self.start_dummy)

        # 右レイアウトにボタンを追加
        self._start_rqt_button = self._create_button("Start rqt", right_layout, self.start_rqt)
        self._create_button("", right_layout, self.start_dummy)
        self._create_button("", right_layout, self.start_dummy)

        self._recording = False
        self._bag_file = None
        self._process1 = QProcess()
        self._process2 = QProcess()
        self._process3 = QProcess()

    def _create_button(self, text, layout, on_click) -> QPushButton:
        button = QPushButton(text)
        button.setMinimumHeight(50)
        button.clicked.connect(on_click)
        layout.addWidget(button)
        return button

    def start_dummy(self):
        pass

    def start_fast_lio_launch(self):
        command = "ros2 launch fast_lio mapping_mid360.launch.py"
        self._process2.start("/bin/bash", ["-c", f". /opt/ros/humble/setup.bash && . /home/taro/ros2_ws/install/setup.bash && {command}"])

    def toggle_rosbag_recording(self):
        if not self._recording:
            self.start_rosbag_recording()
        else:
            self.stop_rosbag_recording()

    def start_rosbag_recording(self):
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self._bag_file = f'{timestamp}.bag'
        command = f"ros2 bag record --all -o {self._bag_file}"
        self._process3.start("/bin/bash", ["-c", f". /opt/ros/humble/setup.bash && . /home/taro/ros2_ws/install/setup.bash && {command}"])

        self._recording = True
        self._start_rosbag_button.setText("Stop recording rosbag")

    def stop_rosbag_recording(self):
        self._process3.terminate()
        self._recording = False
        self._start_rosbag_button.setText("Start recording rosbag")

    def start_rqt(self):
        command = "rqt"
        self._process1.start("/bin/bash", ["-c", f". /opt/ros/humble/setup.bash && . /home/taro/ros2_ws/install/setup.bash && {command}"])

def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
