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

### 一度開発したもののほったらかし
- [x] 音声合成（VoiceVox）
- [x] GLIM
- [x] GNSS
- [x] 音声認識（ASR）
- [x] 音声アクティビティ検出（Silero VAD）
- [x] LED制御（BlinkStick Strip）
- [x] Foxgloveによる可視化
- [x] アバター

### 問題あり
- [x] ウェイクワード検出（Alexa, Hey Jarvis, Hey Mycroft）

## ドキュメント

- [Launchファイル一覧・依存関係図・トピックフロー](launch/README_launch.md)
- [ノード・スクリプト仕様一覧](susumu_robo/README_scripts.md)


