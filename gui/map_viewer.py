#!/usr/bin/env python3
import os
import sys
import re
import shutil
import glob
import yaml
from datetime import datetime
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QListWidget, QListWidgetItem, QLabel, QMessageBox,
    QScrollArea, QSizePolicy, QSplitter, QTableWidget, QTableWidgetItem, QHeaderView
)
from PySide6.QtGui import QPixmap
from PySide6.QtCore import Qt


class MapViewer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("地図ビューア")
        self.resize(900, 600)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QHBoxLayout(central)

        # 左: ファイルリスト
        left = QVBoxLayout()
        left.addWidget(QLabel("地図ファイル (~/*.yaml):"))
        self.list_widget = QListWidget()
        self.list_widget.setFixedWidth(260)
        self.list_widget.currentItemChanged.connect(self._on_select)
        left.addWidget(self.list_widget)

        copy_btn = QPushButton("~/map.yaml, ~/map.pgm にコピー")
        copy_btn.setMinimumHeight(40)
        copy_btn.clicked.connect(self._copy_to_map_dir)
        left.addWidget(copy_btn)
        layout.addLayout(left)

        # 右: 画像とYAML表示を縦に分割
        right_splitter = QSplitter(Qt.Vertical)

        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.image_label = QLabel(alignment=Qt.AlignCenter)
        self.image_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.scroll.setWidget(self.image_label)
        right_splitter.addWidget(self.scroll)

        self.yaml_table = QTableWidget(0, 3)
        self.yaml_table.setHorizontalHeaderLabels(["フィールド", "値", "説明"])
        self.yaml_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.yaml_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.yaml_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        self.yaml_table.verticalHeader().setVisible(False)
        self.yaml_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.yaml_table.setFixedHeight(200)
        right_splitter.addWidget(self.yaml_table)

        layout.addWidget(right_splitter, stretch=1)

        self._load_map_files()

    def _load_map_files(self):
        home = os.path.expanduser("~")
        yaml_files = sorted(glob.glob(os.path.join(home, "map*.yaml")), reverse=True)
        for path in yaml_files:
            mtime = datetime.fromtimestamp(os.path.getmtime(path)).strftime("%Y-%m-%d %H:%M")
            item = QListWidgetItem(f"{os.path.basename(path)}  ({mtime})")
            item.setData(Qt.UserRole, path)
            self.list_widget.addItem(item)
        if self.list_widget.count():
            self.list_widget.setCurrentRow(0)

    def _on_select(self, item):
        if not item:
            return
        yaml_path = item.data(Qt.UserRole)

        # YAML内容を表で表示
        try:
            with open(yaml_path, encoding="utf-8") as f:
                data = yaml.safe_load(f)
            self._fill_yaml_table(data)
        except Exception as e:
            self.yaml_table.setRowCount(1)
            self.yaml_table.setItem(0, 0, QTableWidgetItem("エラー"))
            self.yaml_table.setItem(0, 2, QTableWidgetItem(str(e)))

        # PGM画像表示
        pgm_path = os.path.splitext(yaml_path)[0] + ".pgm"
        if not os.path.exists(pgm_path):
            self.image_label.setText(f".pgm が見つかりません:\n{pgm_path}")
            return
        pixmap = QPixmap(pgm_path)
        if pixmap.isNull():
            self.image_label.setText(f"画像を読み込めません:\n{pgm_path}")
            return
        self._pixmap = pixmap
        self.image_label.setPixmap(
            pixmap.scaled(self.scroll.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        )

    _YAML_DESCRIPTIONS = {
        "image":          "マップ画像ファイルのパス（PGM/PNG形式）",
        "resolution":     "1ピクセルが現実世界で何メートルか（m/px）",
        "origin":         "画像左下隅のワールド座標 [x(m), y(m), yaw(rad)]",
        "negate":         "色反転フラグ。1=白黒を反転して占有度を解釈",
        "occupied_thresh": "この値以上の占有度を「障害物あり」と判定（0〜1）",
        "free_thresh":    "この値以下の占有度を「自由空間」と判定（0〜1）",
        "mode":           "占有度の解釈モード（trinary/scale/raw）",
    }

    def _fill_yaml_table(self, data: dict) -> None:
        self.yaml_table.setRowCount(0)
        for key, value in data.items():
            row = self.yaml_table.rowCount()
            self.yaml_table.insertRow(row)
            self.yaml_table.setItem(row, 0, QTableWidgetItem(str(key)))
            self.yaml_table.setItem(row, 1, QTableWidgetItem(str(value)))
            desc = self._YAML_DESCRIPTIONS.get(key, "")
            self.yaml_table.setItem(row, 2, QTableWidgetItem(desc))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if getattr(self, "_pixmap", None):
            self.image_label.setPixmap(
                self._pixmap.scaled(self.scroll.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            )

    def _copy_to_map_dir(self):
        item = self.list_widget.currentItem()
        if not item:
            QMessageBox.warning(self, "警告", "地図ファイルを選択してください。")
            return
        yaml_path = item.data(Qt.UserRole)
        pgm_path = os.path.splitext(yaml_path)[0] + ".pgm"
        dest_yaml = os.path.expanduser("~/map.yaml")
        dest_pgm = os.path.expanduser("~/map.pgm")
        try:
            if os.path.exists(pgm_path):
                shutil.copy2(pgm_path, dest_pgm)
            # yamlのimage行をmap.pgmに書き換えてコピー
            with open(yaml_path, encoding="utf-8") as f:
                content = f.read()
            content = re.sub(r"^image:.*$", "image: map.pgm", content, flags=re.MULTILINE)
            with open(dest_yaml, "w", encoding="utf-8") as f:
                f.write(content)
            QMessageBox.information(self, "コピー完了", f"~/map.yaml, ~/map.pgm にコピーしました。")
        except Exception as e:
            QMessageBox.critical(self, "エラー", f"コピーに失敗しました:\n{e}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MapViewer()
    window.show()
    sys.exit(app.exec())
