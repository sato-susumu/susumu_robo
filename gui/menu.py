import sys
import platform
import yaml
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
        self.load_and_add_items_from_yaml("menu.yaml")

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
        scroll_area.setFixedWidth(220)
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
        QScroller.grabGesture(self.output_display.viewport(), QScroller.LeftMouseButtonGesture)
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

    def load_and_add_items_from_yaml(self, yaml_file: str) -> None:
        """YAMLファイルからカテゴリ・アイテムを読み込み追加する関数"""
        with open(yaml_file, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)

        categories = data.get('categories', [])
        for category_data in categories:
            self.add_category(self.left_layout, category_data, level=0)

    def add_category(self, parent_layout: QVBoxLayout, category_data: dict, level: int = 0) -> None:
        """カテゴリまたはサブカテゴリを追加する関数"""
        category_name = category_data.get('name', 'Unknown')
        category_display_name = category_name.replace('&', '&&')  # エスケープ処理
        items = category_data.get('items', [])
        subcategories = category_data.get('subcategories', [])

        category_button = QPushButton(category_display_name)
        category_button.setCheckable(True)
        category_button.setChecked(True)  # 初期状態で展開

        # 階層レベルによって色分け＆左寄せ
        if level == 0:
            category_button.setStyleSheet(
                "background-color: lightblue; font-weight: bold; text-align: left; padding-left: 5px;"
            )
        else:
            category_button.setStyleSheet(
                "background-color: lightgray; font-weight: bold; text-align: left; padding-left: 5px;"
            )

        # 初期状態は展開中なので、▼を付ける
        category_button.setText(f"▼ {category_display_name}")

        category_frame = QFrame()
        category_layout = QVBoxLayout(category_frame)
        category_layout.setContentsMargins(10, 5, 0, 5)
        category_frame.setVisible(True)

        # トグル時にテキストを変更するスロットを定義
        def toggle_text(checked):
            if checked:
                category_button.setText(f"▼ {category_display_name}")
            else:
                category_button.setText(f"▶ {category_display_name}")
            category_frame.setVisible(checked)

        category_button.toggled.connect(toggle_text)

        parent_layout.addWidget(category_button)
        parent_layout.addWidget(category_frame)

        # アイテムを追加
        for item in items:
            name = item.get('name', 'NoName')
            command = item.get('command', '')
            self.add_item_button(category_layout, name, command)

        # サブカテゴリを再帰的に追加
        for subcat in subcategories:
            self.add_category(category_layout, subcat, level=level+1)

    def add_item_button(self, layout, name: str, command: str) -> None:
        """アイテムボタンを指定レイアウトに追加する"""
        button = QPushButton(name.replace('&', '&&'))  # エスケープ処理
        button.setMinimumHeight(40)
        button.setStyleSheet("background-color: white; text-align: left; padding-left: 5px;")
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
        self.item_buttons[name].setStyleSheet("background-color: lightgreen; text-align: left; padding-left: 5px;")

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
            self.item_buttons[name].setStyleSheet("background-color: white; text-align: left; padding-left: 5px;")

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
                button.setStyleSheet("background-color: lightblue; text-align: left; padding-left: 5px;")
            elif name in self.processes:
                button.setStyleSheet("background-color: lightgreen; text-align: left; padding-left: 5px;")
            else:
                button.setStyleSheet("background-color: white; text-align: left; padding-left: 5px;")

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
