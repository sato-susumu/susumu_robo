import sys
import PySide6
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton
import os
import subprocess
import datetime

# 環境変数にPySide6を登録
dirname = os.path.dirname(PySide6.__file__)
plugin_path = os.path.join(dirname, 'plugins', 'platforms')
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = plugin_path

class MyMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("ROS2 GUI App")
        self.setGeometry(100, 100, 800, 600)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Layouts
        layout = QHBoxLayout(central_widget)
        left_layout = QVBoxLayout()
        right_layout = QVBoxLayout()
        layout.addLayout(left_layout)
        layout.addLayout(right_layout)

        # Creating buttons
        self.create_button("Start bbb.launch", left_layout, self.start_bbb_launch)
        self.create_button("Start Recording rosbag", left_layout, self.start_rosbag_recording)
        # Add more buttons as needed...

    def create_button(self, text, layout, on_click):
        button = QPushButton(text)
        button.clicked.connect(on_click)
        layout.addWidget(button)
    def start_bbb_launch(self):
        subprocess.run(
            ["/bin/bash", "-c", ". /opt/ros/humble/setup.bash && . /home/taro/ros2_ws/install/setup.bash && "
                                "ros2 launch fast_lio mapping_mid360.launch.py"])
        # subprocess.run(
        #     ["/bin/bash", "-c", ". /opt/ros/humble/setup.bash && . /home/taro/ros2_ws/install/setup.bash && "
        #                         "ros2 run turtlesim turtlesim_node"])

    def start_rosbag_recording(self):
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_file = f'{timestamp}.bag'
        subprocess.run(
            ["/bin/bash", "-c", ". /opt/ros/humble/setup.bash && . /home/taro/ros2_ws/install/setup.bash && "
                                f"ros2 bag record --all -o {bag_file}"])


def main():
    app = QApplication(sys.argv)
    window = MyMainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
