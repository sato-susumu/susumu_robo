# susumu_robo

## プロジェクト概要
ROS2搭載でパワフルながら静かに動く小型移動ロボット。
永遠に完成しない適当プロジェクト？

## 主要機能

### 現在アクティブな機能
- [x] タッチ対応GUI メニュー
- [x] 3D LiDAR（Livox Mid-360）
- [x] ゲームパッド操作
- [x] 衝突回避システム
- [x] SLAM & ナビゲーション（slam_toolbox, Nav2）
- [x] デプスカメラ（RealSense D435i）
- [x] 音声合成（AivisSpeech + speak_ros）
- [x] 音声認識（Google Cloud ASR）
- [x] Gazebo Classic シミュレーション（TurtleBot3 world 流用）

## シミュレータ起動

実機ハードウェアなしで全スタックを動かす場合:

```bash
# Gazebo + ロボット + 衝突回避 + twist_mux + foxglove + dummy GNSS
ros2 launch susumu_robo robo_indoor_sim.launch.py

# Nav2 (事前にマップが必要)
ros2 launch susumu_robo slam_toolbox_sim.launch.py    # 1回目はSLAMでマップ作成
ros2 run nav2_map_server map_saver_cli -f ~/sim_map
ros2 launch susumu_robo nav2_sim.launch.py             # 以降はNav2で自律走行

# オフライン音声 (ダミー speak + STT debug + agent)
ros2 launch susumu_robo audio_option_sim.launch.py
```

### 一度開発したもののほったらかし
- [x] FAST-LIO
- [x] GLIM
- [x] GNSS
- [x] 音声アクティビティ検出（Silero VAD）
- [x] LED制御（BlinkStick Strip）
- [x] Foxgloveによる可視化
- [x] アバター

### 問題あり
- [x] ウェイクワード検出（Alexa, Hey Jarvis, Hey Mycroft）

## ドキュメント

- [Launchファイル一覧・依存関係図・トピックフロー](launch/README_launch.md)
- [ノード・スクリプト仕様一覧](susumu_robo/README_scripts.md)


