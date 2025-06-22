import sys
import subprocess
from dataclasses import dataclass
from typing import Dict, List

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QFrame, QLabel,
    QPushButton, QVBoxLayout, QHBoxLayout, QGridLayout,
    QDialog, QTextEdit, QScrollArea, QMenuBar, QMessageBox
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QAction

# --- Data Model ---
@dataclass
class ServiceItem:
    name: str
    commands: Dict[str, str]
    active: bool = False
    enabled: bool = False

    def refresh_status(self) -> None:
        """
        Update 'active' and 'enabled' flags by querying systemctl.
        """
        self._refresh_active()
        self._refresh_enabled()

    def _refresh_active(self) -> None:
        try:
            out = subprocess.run(
                ["systemctl", "is-active", self.name],
                capture_output=True, text=True, check=False
            ).stdout.strip()
            self.active = (out == "active")
        except Exception:
            self.active = False

    def _refresh_enabled(self) -> None:
        try:
            out = subprocess.run(
                ["systemctl", "is-enabled", self.name],
                capture_output=True, text=True, check=False
            ).stdout.strip()
            self.enabled = (out == "enabled")
        except Exception:
            self.enabled = False

# --- UI Component ---
class ServiceTile(QFrame):
    def __init__(self, service: ServiceItem) -> None:
        super().__init__()
        self.service = service
        self._init_ui()
        self._connect_signals()
        self._update_ui()

    def _init_ui(self) -> None:
        self.setFrameShape(QFrame.StyledPanel)
        self.setLineWidth(1)
        # Layouts
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(4, 4, 4, 4)
        main_layout.setSpacing(4)

        # Labels
        self.name_label = QLabel(self.service.name, alignment=Qt.AlignCenter)
        self.name_label.setContentsMargins(0, 0, 0, 0)
        self.active_label = QLabel()
        self.enabled_label = QLabel()

        # Status layout
        status_layout = QHBoxLayout()
        status_layout.setContentsMargins(0, 0, 0, 0)
        status_layout.setSpacing(4)
        status_layout.addWidget(self.active_label)
        status_layout.addWidget(self.enabled_label)

        # Buttons
        self.start_btn = QPushButton('Start')
        self.stop_btn = QPushButton('Stop')
        self.enable_btn = QPushButton('Enable')
        self.disable_btn = QPushButton('Disable')
        self.log_btn = QPushButton('Log')
        btn_layout = QHBoxLayout()
        btn_layout.setContentsMargins(0, 0, 0, 0)
        btn_layout.setSpacing(2)
        for btn in [self.start_btn, self.stop_btn, self.enable_btn, self.disable_btn, self.log_btn]:
            btn.setFixedHeight(20)
            btn_layout.addWidget(btn)

        # Assemble
        main_layout.addWidget(self.name_label)
        main_layout.addLayout(status_layout)
        main_layout.addLayout(btn_layout)

    def _connect_signals(self) -> None:
        self.start_btn.clicked.connect(lambda: self._run_command('start'))
        self.stop_btn.clicked.connect(lambda: self._run_command('stop'))
        self.enable_btn.clicked.connect(lambda: self._run_command('enable'))
        self.disable_btn.clicked.connect(lambda: self._run_command('disable'))
        self.log_btn.clicked.connect(lambda: self._run_command('log'))

    def _run_command(self, action: str) -> None:
        cmd = self.service.commands.get(action)
        if not cmd:
            return
        proc = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, text=True)
        out, err = proc.communicate()
        if action == 'log':
            self._show_log_dialog(out or err)
        else:
            self.service.refresh_status()
            self._update_ui()

    def _show_log_dialog(self, text: str) -> None:
        dialog = QDialog(self)
        dialog.setWindowTitle(f'Logs: {self.service.name}')
        layout = QVBoxLayout(dialog)
        txt = QTextEdit(readOnly=True)
        txt.setPlainText(text)
        layout.addWidget(txt)
        dialog.resize(600, 400)
        dialog.exec()

    def _update_ui(self) -> None:
        # Active status
        if self.service.active:
            self.active_label.setText('Active')
            self.active_label.setStyleSheet('color: green; font-weight: bold;')
            self.start_btn.setVisible(False)
            self.stop_btn.setVisible(True)
        else:
            self.active_label.setText('Inactive')
            self.active_label.setStyleSheet('color: red; font-weight: bold;')
            self.start_btn.setVisible(True)
            self.stop_btn.setVisible(False)
        # Enabled status
        if self.service.enabled:
            self.enabled_label.setText('Enabled')
            self.enabled_label.setStyleSheet('color: green; font-weight: bold;')
            self.enable_btn.setVisible(False)
            self.disable_btn.setVisible(True)
        else:
            self.enabled_label.setText('Disabled')
            self.enabled_label.setStyleSheet('color: red; font-weight: bold;')
            self.enable_btn.setVisible(True)
            self.disable_btn.setVisible(False)
        # Log always available
        self.log_btn.setVisible(True)

# --- Main Dashboard ---
class DashboardWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle('Systemd Dashboard')
        self.setMinimumSize(800, 600)
        self._init_ui()
        self._load_services()

    def _init_ui(self) -> None:
        # メニューバーを作成
        self._create_menu_bar()
        
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)

        container = QWidget()
        self.grid = QGridLayout(container)
        self.grid.setContentsMargins(4, 4, 4, 4)
        self.grid.setSpacing(4)

        scroll.setWidget(container)
        layout.addWidget(scroll)

    def _create_menu_bar(self) -> None:
        """メニューバーを作成"""
        menubar = self.menuBar()
        
        # ツールメニュー
        tools_menu = menubar.addMenu('ツール')
        
        # ジャーナルログ削除アクション
        cleanup_action = QAction('直近7日間以外のジャーナルログ削除', self)
        cleanup_action.triggered.connect(self._cleanup_journal_logs)
        tools_menu.addAction(cleanup_action)
        
        # ディスク使用量確認アクション
        disk_usage_action = QAction('ジャーナルログの使用ディスク量確認', self)
        disk_usage_action.triggered.connect(self._check_journal_disk_usage)
        tools_menu.addAction(disk_usage_action)
        
        tools_menu.addSeparator()
        
        # 終了アクション
        exit_action = QAction('終了', self)
        exit_action.triggered.connect(self.close)
        tools_menu.addAction(exit_action)

    def _cleanup_journal_logs(self) -> None:
        """直近7日間以外のジャーナルログを削除"""
        reply = QMessageBox.question(
            self, 
            '確認',
            '直近7日間以外のジャーナルログを削除しますか？\nこの操作は元に戻せません。',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            try:
                # journalctl --vacuum-time=7d を実行
                proc = subprocess.run(
                    ['sudo', 'journalctl', '--vacuum-time=7d'],
                    capture_output=True, text=True, check=True
                )
                
                QMessageBox.information(
                    self, 
                    '完了',
                    f'ジャーナルログの削除が完了しました。\n\n{proc.stdout}'
                )
            except subprocess.CalledProcessError as e:
                QMessageBox.critical(
                    self,
                    'エラー', 
                    f'ジャーナルログの削除に失敗しました。\n\nエラー: {e.stderr}'
                )
            except Exception as e:
                QMessageBox.critical(
                    self,
                    'エラー',
                    f'予期しないエラーが発生しました。\n\nエラー: {str(e)}'
                )

    def _check_journal_disk_usage(self) -> None:
        """ジャーナルログの使用ディスク量を確認"""
        try:
            # journalctl --disk-usage を実行
            proc = subprocess.run(
                ['sudo', 'journalctl', '--disk-usage'],
                capture_output=True, text=True, check=True
            )
            
            QMessageBox.information(
                self,
                'ディスク使用量',
                f'ジャーナルログのディスク使用量:\n\n{proc.stdout}'
            )
        except subprocess.CalledProcessError as e:
            QMessageBox.critical(
                self,
                'エラー',
                f'ディスク使用量の確認に失敗しました。\n\nエラー: {e.stderr}'
            )
        except Exception as e:
            QMessageBox.critical(
                self,
                'エラー',
                f'予期しないエラーが発生しました。\n\nエラー: {str(e)}'
            )

    def _load_services(self) -> None:
        service_names = [
            'ros2_bringup', 'ros2_mid360', 'ros2_ddsm115',
            'ros2_realsense', 'ros2_led', 'ros2_nav2',
            'ros2_audio',
            'ros2_asr_with_vad',
            'ros2_asr_with_wake_word',
            'ros2_rai_whoami',
            'ros2_rai_voice_hmi',
            # ... more services ...
        ]
        self.services: List[ServiceItem] = []
        for name in service_names:
            cmds = {
                'start': f'sudo systemctl start {name}',
                'stop': f'sudo systemctl stop {name}',
                'enable': f'sudo systemctl enable {name}',
                'disable': f'sudo systemctl disable {name}',
                'log': f'sudo journalctl -u {name}.service --no-pager'
            }
            svc = ServiceItem(name, cmds)
            svc.refresh_status()
            self.services.append(svc)
        self._populate_grid()

    def _populate_grid(self) -> None:
        cols = 3
        for idx, svc in enumerate(self.services):
            tile = ServiceTile(svc)
            self.grid.addWidget(tile, idx // cols, idx % cols)

# --- Entry Point ---
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = DashboardWindow()
    window.show()
    sys.exit(app.exec())
