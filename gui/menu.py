import sys
import PySide6
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton
import os
import subprocess
import datetime

dirname = os.path.dirname(PySide6.__file__)
plugin_path = os.path.join(dirname, 'plugins', 'platforms')
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = plugin_path


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

        # 右レイアウトにボタンを追加
        self._start_rqt_button = self._create_button("Start rqt", right_layout, self.start_rqt)
        self._start_dummy_button = self._create_button("", right_layout, self.start_dummy)

        self._recording = False
        self._bag_file = None
        self._process = None

    def _create_button(self, text, layout, on_click) -> QPushButton:
        button = QPushButton(text)
        button.setMinimumHeight(50)
        button.clicked.connect(on_click)
        layout.addWidget(button)
        return button

    def start_dummy(self):
        pass

    def start_fast_lio_launch(self):
        subprocess.run(
            ["/bin/bash", "-c", ". /opt/ros/humble/setup.bash && . /home/taro/ros2_ws/install/setup.bash && "
                                "ros2 launch fast_lio mapping_mid360.launch.py"])

    def toggle_rosbag_recording(self):
        if not self._recording:
            self.start_rosbag_recording()
        else:
            self.stop_rosbag_recording()

    def start_rosbag_recording(self):
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self._bag_file = f'{timestamp}.bag'
        self._process = subprocess.Popen(
            ["/bin/bash", "-c", ". /opt/ros/humble/setup.bash && . /home/taro/ros2_ws/install/setup.bash && "
                                f"ros2 bag record --all -o {self._bag_file}"])
        self._recording = True
        self._start_rosbag_button.setText("Stop recording rosbag")

    def stop_rosbag_recording(self):
        self._process.terminate()
        self._recording = False
        self._start_rosbag_button.setText("Start recording rosbag")

    def start_rqt(self):
        subprocess.run(
            ["/bin/bash", "-c", ". /opt/ros/humble/setup.bash && . /home/taro/ros2_ws/install/setup.bash && "
                                "rqt"])


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
