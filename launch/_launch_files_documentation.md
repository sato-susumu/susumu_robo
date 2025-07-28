# Susumu Robot Launch Files Documentation

このドキュメントでは、`susumu_robo`パッケージの`launch`ディレクトリ内にある全てのlaunchファイルとスクリプトについて説明します。

## 目次

1. [概要](#概要)
2. [メインシステム起動](#メインシステム起動)
3. [ハードウェアインターフェース](#ハードウェアインターフェース)
4. [ナビゲーション](#ナビゲーション)
5. [音声処理](#音声処理)
6. [制御システム](#制御システム)
7. [サービス管理スクリプト](#サービス管理スクリプト)
8. [起動ファイルの階層構造](#起動ファイルの階層構造)

## 概要

Susumu Robotのlaunchシステムは、レガシー版（接頭辞なし）と新版（`ros2_`接頭辞付き）の2つのパターンで構成されています。新版は主にsystemdサービスとして使用され、レガシー版は実際のノード起動処理を含んでいます。

## メインシステム起動

### ros2_bringup.launch.py / bringup.launch.py

**目的**: ロボットの基本システムを起動するメインランチャー

**起動内容**:
- `base.launch.py`: モーター制御のためのtwist stamper
- `teleop_twist_joy.launch.py`: PS5 DualSenseゲームパッド制御
- `collision_monitor.launch.py`: 障害物回避システム
- Foxglove Bridge: Webベースの可視化（ポート8765）

**依存関係**:
- `susumu_robo`パッケージ内の各launchファイル
- `foxglove_bridge`パッケージ

## ハードウェアインターフェース

### ros2_mid360.launch.py / mid360.launch.py

**目的**: Livox Mid-360 3D LiDARの起動

**起動内容**:
- `livox_ros2_driver`パッケージのmsg_MID360_launch.py
- パラメータファイル: `MID360_config.json`

**主要パラメータ**:
- `publish_freq`: 10Hz
- `multi_topic`: 0（単一トピック）
- トピック名: `/livox/lidar`

### ros2_ddsm115.launch.py / ddsm115.launch.py

**目的**: DDSM115ダイレクトドライブサーボモーターの制御

**起動内容**:
- `ddsm115_ros2`パッケージのノード
- RS485通信インターフェース

**依存関係**:
- `ddsm115_ros2`パッケージ（別リポジトリ）

### ros2_realsense.launch.py / realsense.launch.py

**目的**: Intel RealSense D435i深度カメラの起動

**起動内容**:
- `realsense2_camera`パッケージのrs_launch.py
- RGB-Dカメラストリーム

**主要設定**:
- `enable_rgbd`: true
- `enable_sync`: true
- `align_depth.enable`: true
- `enable_color`: true
- `enable_depth`: true

### ros2_led.launch.py / led.launch.py

**目的**: BlinkStick LED制御

**起動内容**:
- `susumu_robo`パッケージの`led_controller_node`
- 障害物検知に基づくLEDパターン制御

**機能**:
- 障害物検知時: 黄色点滅
- 通常時: 緑色

## ナビゲーション

### ros2_nav2.launch.py / nav2.launch.py

**目的**: Nav2ナビゲーションスタックの起動

**起動内容**:
1. SLAM Toolbox（オンラインasync）
2. Nav2 bringup（navigation_launch.py）

**設定**:
- SLAMモード: オンライン非同期
- 自動起動: true
- Nav2パラメータ: カスタム設定ファイル使用

## 音声処理

### ros2_audio.launch.py / audio.launch.py

**目的**: 基本的な音声合成機能

**起動内容**:
- `ros2_voicevox`パッケージのvoicevox_launch.py
- VoiceVoxテキスト音声合成エンジン

### ros2_asr_with_vad.launch.py / asr_with_vad.launch.py

**目的**: Voice Activity Detection (VAD)付き音声認識

**起動内容**:
- `respeaker_ros`パッケージ
- `ros2_whisper`パッケージ（VAD統合）

**設定**:
- VADモデル: Silero
- 言語: 日本語（ja）
- マイクデバイス: Anker PowerConf S3

### ros2_asr_with_wake_word.launch.py / asr_with_wake_word.launch.py

**目的**: ウェイクワード検出付き音声認識

**起動内容**:
- `respeaker_ros`パッケージ
- `ros2_whisper`パッケージ（ウェイクワード統合）

**ウェイクワード**:
- alexa
- hey_jarvis
- hey_mycroft

**依存モデル**:
- ONNX形式のウェイクワードモデル（`models/`ディレクトリ）

## 制御システム

### base.launch.py

**目的**: twist_stamperノードによる速度コマンドのタイムスタンプ付与

**機能**:
- `/cmd_vel`を`/cmd_vel_stamped`に変換
- ナビゲーションスタックとの互換性確保

### teleop_twist_joy.launch.py

**目的**: PS5 DualSenseコントローラーによる手動操作

**起動内容**:
- `joy`ノード: ジョイスティック入力
- `teleop_twist_joy`ノード: twist変換

**設定**:
- デバイス: `/dev/input/js0`
- デッドマンスイッチ: 必須
- 自動リピート: 有効

### collision_monitor.launch.py

**目的**: LiDARベースの障害物回避

**起動内容**:
- `laserscan_filter_node`: LiDARデータのフィルタリング
- `twist_filter_node`: 安全な速度制御

**機能**:
- 進行方向の障害物検知
- 自動停止（前進時のみ）
- 回転は障害物があっても許可

### tenkey_controller.launch.py

**目的**: 10キー入力デバイスによる制御

**起動内容**:
- `tenkey_controller`ノード
- シンプルな速度コマンド生成

### twist_mux.launch.py

**目的**: 複数の速度コマンドソースの管理（現在は実験的）

**機能**:
- 優先度ベースの入力切り替え
- タイムアウト管理

## サービス管理スクリプト

### 起動・停止・状態確認

#### start_ros2_service.sh
全てのROS2サービスを起動します：
```bash
sudo systemctl start ros2_bringup.service
sudo systemctl start ros2_mid360.service
sudo systemctl start ros2_ddsm115.service
sudo systemctl start ros2_realsense.service
sudo systemctl start ros2_led.service
sudo systemctl start ros2_nav2.service
```

#### stop_ros2_service.sh
全てのROS2サービスを停止します。

#### status_ros2_service.sh
サービスの状態を確認します：
- アクティブなサービス一覧
- インストール済みサービス一覧

### インストール・アンインストール

#### _install_for_startup.sh
`robot_upstart`を使用してROS2 launchファイルをsystemdサービスとして登録します。

**特徴**:
- 自動的にROS2環境をセットアップ
- ネットワーク依存性の設定
- 音声サービスはサウンドシステム依存

#### _uninstall_for_startup.sh
インストールされた全てのサービスを削除します。

#### _install_ros2_rai_services.sh / _uninstall_ros2_rai_services.sh
RAI（Robot AI）統合サービスの手動インストール/アンインストール：
- `ros2_rai_whoami`: ロボット識別サービス
- `ros2_rai_voice_hmi`: 音声対話インターフェース

**特徴**:
- 手動でsystemdサービスファイルを作成
- インタラクティブシェル環境を使用
- `~/rai`ディレクトリで実行

### 実行スクリプト

#### run_ros2_rai_voice_hmi.sh
RAI音声HMIを直接実行（デバッグ用）

#### run_ros2_rai_whoami.sh
RAI whoamiサービスを直接実行（デバッグ用）

## 起動ファイルの階層構造

```
ros2_bringup.launch.py
    └── bringup.launch.py
        ├── base.launch.py (twist_stamper)
        ├── teleop_twist_joy.launch.py
        │   ├── joy_node
        │   └── teleop_twist_joy_node
        ├── collision_monitor.launch.py
        │   ├── laserscan_filter_node
        │   └── twist_filter_node
        └── foxglove_bridge_launch.xml

ros2_nav2.launch.py
    └── nav2.launch.py
        ├── slam_toolbox (online_async)
        └── nav2_bringup/navigation_launch.py

ros2_audio.launch.py / ros2_asr_with_*.launch.py
    └── 各音声処理パッケージ
```

## 使用上の注意

1. **サービス起動順序**: 基本的に依存関係は自動解決されますが、`ros2_bringup`を最初に起動することを推奨

2. **デバイス権限**: 
   - RS485（モーター）: dialoutグループへの追加が必要
   - オーディオデバイス: audioグループへの追加が必要
   - ジョイスティック: inputグループへの追加が必要

3. **ネットワーク設定**:
   - Livox Mid-360: 静的IP（192.168.1.50）が必要
   - Foxglove Bridge: ポート8765を開放

4. **モデルファイル**:
   - ウェイクワードモデルは`models/`ディレクトリに配置
   - ONNX形式（.onnx）およびTensorFlow Lite形式（.tflite）をサポート