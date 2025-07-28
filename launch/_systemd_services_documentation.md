# Susumu Robot Systemd Services Documentation

このドキュメントでは、Susumu Robotで使用されているsystemdサービスの詳細について説明します。

## 目次

1. [概要](#概要)
2. [サービス一覧](#サービス一覧)
3. [サービス作成方法](#サービス作成方法)
4. [各サービスの詳細](#各サービスの詳細)
5. [サービスの依存関係](#サービスの依存関係)
6. [環境変数とパス設定](#環境変数とパス設定)
7. [サービス管理コマンド](#サービス管理コマンド)
8. [トラブルシューティング](#トラブルシューティング)

## 概要

Susumu Robotは、ROS2ノードをsystemdサービスとして管理することで、システムの起動時に自動的にロボットの機能を立ち上げることができます。サービスは主に2つの方法で作成されています：

1. **robot_upstart**パッケージを使用した自動生成
2. **手動作成**（RAI統合サービス用）

## サービス一覧

### コアサービス（robot_upstartで生成）

| サービス名 | 説明 | 依存関係 | 状態 |
|----------|------|---------|------|
| ros2_bringup.service | メインシステムランチャー | nss-lookup.target | disabled |
| ros2_mid360.service | Livox Mid-360 3D LiDAR | nss-lookup.target | disabled |
| ros2_ddsm115.service | DDSM115モーター制御 | nss-lookup.target | disabled |
| ros2_realsense.service | RealSense D435iカメラ | nss-lookup.target | disabled |
| ros2_led.service | BlinkStick LED制御 | nss-lookup.target | disabled |
| ros2_nav2.service | Navigation2スタック | nss-lookup.target | disabled |

### 音声処理サービス

| サービス名 | 説明 | 依存関係 | 特殊設定 |
|----------|------|---------|---------|
| ros2_audio.service | VoiceVox音声合成 | sound.target | PulseAudio、環境変数 |
| ros2_asr_with_vad.service | VAD付き音声認識 | sound.target | - |
| ros2_asr_with_wake_word.service | ウェイクワード付き音声認識 | sound.target | - |

### RAI統合サービス（手動作成）

| サービス名 | 説明 | 依存関係 | 特殊設定 |
|----------|------|---------|---------|
| ros2_rai_voice_hmi.service | 音声対話インターフェース | network.target | 対話シェル、自動再起動 |
| ros2_rai_whoami.service | ロボット識別サービス | network.target | 対話シェル、自動再起動 |

## サービス作成方法

### robot_upstartによる自動生成

robot_upstartは以下の構造でサービスを作成します：

```
/lib/systemd/system/ros2_xxx.service
    ↓ ExecStart
/usr/sbin/ros2_xxx-start (起動スクリプト)
    ↓ source & launch
/etc/ros/humble/ros2_xxx.d/ (ジョブディレクトリ)
    └── job.launch (実際のlaunchファイル)
```

#### 生成されるサービスファイルの例（ros2_bringup.service）

```ini
[Unit]
Description="bringup ros2_bringup"
After=nss-lookup.target

[Service]
Type=simple
ExecStart=/usr/sbin/ros2_bringup-start

[Install]
WantedBy=multi-user.target
```

#### 起動スクリプトの内容

```bash
#!/bin/bash
# RMWミドルウェアの設定
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# ワークスペースのセットアップ
source /home/taro/ros2_ws/install/setup.bash

# ログディレクトリの設定
export ROS_LOG_DIR=/tmp

# ユーザー権限で実行
exec /usr/bin/setpriv --reuid taro --regid taro --init-groups \
    /opt/ros/humble/bin/ros2 launch -a ... job.launch
```

### 手動作成（RAIサービス）

RAIサービスは特殊な要件のため手動で作成されています：

```ini
[Unit]
Description=ROS2 RAI HMI Voice Node
After=network.target

[Service]
User=taro
WorkingDirectory=/home/taro/rai
ExecStart=/bin/bash -i -c '/home/taro/ros2_ws/src/susumu_robo/launch/run_ros2_rai_voice_hmi.sh'
Restart=on-failure
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

## 各サービスの詳細

### ros2_bringup.service

**機能**: ロボットの基本システムを統合的に起動

**起動内容**:
- base.launch.py（twist_stamper）
- teleop_twist_joy.launch.py（ジョイパッド制御）
- collision_monitor.launch.py（障害物回避）
- foxglove_bridge（可視化）

### ros2_mid360.service

**機能**: Livox Mid-360 3D LiDARドライバー

**設定**:
- 発行頻度: 10Hz
- トピック: `/livox/lidar`
- 静的IP: 192.168.1.50

### ros2_ddsm115.service

**機能**: DDSM115ダイレクトドライブサーボモーター制御

**要件**:
- RS485-USB変換器
- dialoutグループへの所属

### ros2_realsense.service

**機能**: Intel RealSense D435i深度カメラ

**出力**:
- RGB画像
- 深度画像
- 点群データ

### ros2_audio.service

**機能**: VoiceVoxによる日本語音声合成

**特殊環境変数**:
```bash
PULSE_SERVER=unix:/run/user/1000/pulse/native
XDG_RUNTIME_DIR=/run/user/1000
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/taro/ros2_ws/src/voicevox_ros2/voicevox_core
JTALK_PATH=/home/taro/ros2_ws/src/voicevox_ros2/voicevox_core/open_jtalk_dic_utf_8-1.11
```

### ros2_led.service

**機能**: BlinkStick LED制御

**動作**:
- 通常時: 緑色
- 障害物検知時: 黄色点滅

### ros2_nav2.service

**機能**: Navigation2スタックとSLAM

**含まれる機能**:
- SLAM Toolbox（オンライン非同期）
- Nav2ナビゲーション
- 経路計画
- 障害物回避

### ros2_asr_with_vad.service / ros2_asr_with_wake_word.service

**機能**: 音声認識サービス

**特徴**:
- VAD版: 音声活動検出による自動認識開始
- ウェイクワード版: 特定のキーワードで認識開始

### ros2_rai_voice_hmi.service / ros2_rai_whoami.service

**機能**: RAI（Robot AI）統合サービス

**特徴**:
- 対話シェル（`/bin/bash -i -c`）を使用
- 完全な環境変数の継承
- 失敗時の自動再起動
- `~/rai`ディレクトリで実行

## サービスの依存関係

### 起動順序の依存関係

```
システム起動
    ↓
ネットワーク初期化 (network.target)
    ↓
名前解決サービス (nss-lookup.target)
    ├→ ros2_bringup.service
    ├→ ros2_mid360.service
    ├→ ros2_ddsm115.service
    ├→ ros2_realsense.service
    ├→ ros2_led.service
    └→ ros2_nav2.service
    
サウンドシステム (sound.target)
    ├→ ros2_audio.service
    ├→ ros2_asr_with_vad.service
    └→ ros2_asr_with_wake_word.service

ネットワーク (network.target)
    ├→ ros2_rai_voice_hmi.service
    └→ ros2_rai_whoami.service
```

### 機能的な依存関係

- **ros2_nav2**は**ros2_mid360**（LiDAR）のデータを使用
- **ros2_led**は**collision_monitor**の障害物検知情報を使用
- **ros2_audio**系サービスは音声デバイスへのアクセスが必要

## 環境変数とパス設定

### 共通環境変数

```bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp  # DDS実装
ROS_LOG_DIR=/tmp                     # ログディレクトリ
ROS_HOME=/home/taro/.ros             # ROS設定ディレクトリ
```

### VoiceVox関連（ros2_audio）

```bash
PULSE_SERVER=unix:/run/user/1000/pulse/native
LD_LIBRARY_PATH=/path/to/voicevox_core
JTALK_PATH=/path/to/open_jtalk_dic
```

## サービス管理コマンド

### 基本的な操作

```bash
# サービスの起動
sudo systemctl start ros2_bringup.service

# サービスの停止
sudo systemctl stop ros2_bringup.service

# サービスの状態確認
sudo systemctl status ros2_bringup.service

# サービスの有効化（自動起動）
sudo systemctl enable ros2_bringup.service

# サービスの無効化
sudo systemctl disable ros2_bringup.service

# サービスの再起動
sudo systemctl restart ros2_bringup.service
```

### ログの確認

```bash
# リアルタイムログ表示
journalctl -u ros2_bringup.service -f

# 過去のログ表示
journalctl -u ros2_bringup.service --since "1 hour ago"

# エラーのみ表示
journalctl -u ros2_bringup.service -p err
```

### 一括操作スクリプト

```bash
# 全サービス起動
./start_ros2_service.sh

# 全サービス停止
./stop_ros2_service.sh

# 全サービス状態確認
./status_ros2_service.sh
```

## トラブルシューティング

### よくある問題と解決方法

#### 1. サービスが起動しない

**確認事項**:
```bash
# 詳細なエラーを確認
journalctl -u ros2_xxx.service -n 50

# 権限の確認
ls -la /dev/ttyUSB*  # RS485デバイス
groups taro          # ユーザーのグループ確認
```

#### 2. "No module named 'xxx'" エラー

**解決方法**:
```bash
# ワークスペースの再ビルド
cd /home/taro/ros2_ws
colcon build
```

#### 3. デバイスアクセスエラー

**解決方法**:
```bash
# 必要なグループに追加
sudo usermod -a -G dialout taro  # シリアルポート
sudo usermod -a -G audio taro    # オーディオデバイス
sudo usermod -a -G input taro    # 入力デバイス
# ログアウト・ログインが必要
```

#### 4. ネットワーク関連エラー（LiDAR）

**確認事項**:
- ネットワークインターフェースのIP設定
- `ping 192.168.1.50`でLiDARへの接続確認
- ファイアウォール設定

#### 5. 音声サービスのエラー

**確認事項**:
- PulseAudioサーバーの起動状態
- 音声デバイスの認識: `aplay -l`
- 環境変数の設定確認

### サービスのデバッグ

```bash
# サービスを手動で実行してデバッグ
sudo -u taro /usr/sbin/ros2_bringup-start

# 環境変数を確認しながら実行
sudo -u taro env | grep ROS
```

### 設定のリセット

```bash
# サービスの完全削除と再インストール
./_uninstall_for_startup.sh
./_install_for_startup.sh
```

## 推奨される運用方法

1. **開発時**: 手動でlaunchファイルを実行してデバッグ
2. **テスト時**: サービスを個別に起動して動作確認
3. **本番運用時**: 必要なサービスのみ有効化して自動起動

## セキュリティ考慮事項

- サービスは`taro`ユーザー権限で実行（root権限ではない）
- ネットワークサービスは必要最小限のポートのみ使用
- ログは`/tmp`に出力されるため、定期的な削除が必要