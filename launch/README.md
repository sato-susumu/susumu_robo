# _install_for_startup.shの注意点
/home/taro/ros2_ws/install/setup.bashに依存しています。いずれ、修正が必要

# 問題発生時の対処法
## systemdで登録したサービスで音声が再生できない
```
sudo vi /lib/systemd/system/ros2_audio.service
```
ファイルの末尾に次の内容を追記します

例：Userがtaro、uidが1000、ワークスペースが/home/taro/ros2_wsの場合
```
[Service]
Environment="PULSE_SERVER=unix:/run/user/1000/pulse/native"
Environment="XDG_RUNTIME_DIR=/run/user/1000"
Environment="LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/taro/ros2_ws/src/voicevox_ros2/voicevox_core"
Environment="JTALK_PATH=/home/taro/ros2_ws/src/voicevox_ros2/voicevox_core/open_jtalk_dic_utf_8-1.11"
Environment="KANAENG_PATH=/home/taro/ros2_ws/src/voicevox_ros2/voicevox_core/bep-eng.dic"
User=taro
```

修正後は再起動し、オーディオを使った処理が動くことを確認する

## systemd実行時、ディレクトリ作成権限がないため実行に失敗する
ros2_asr_with_vad および ros2_asr_with_wake_word で発生するため修正
ros2_asr_with_vadの修正例
```
sudo vi /lib/systemd/system/ros2_asr_with_vad.service
```

修正前：
```
[Service]
Type=simple
ExecStart=/usr/sbin/ros2_asr_with_vad-start
```

修正後：Userがtaro、uidが1000、ワークスペースが/home/taro/ros2_wsの場合
```
[Service]
Type=simple
WorkingDirectory=/home/taro/ros2_ws/install/susumu_asr_ros/share/susumu_asr_ros
Environment="PULSE_SERVER=unix:/run/user/1000/pulse/native"
User=taro
ProtectHome=no
ExecStart=/usr/sbin/ros2_asr_with_vad-start
```
色々試行錯誤して上記のような設定になったため、もう少し減らす余地がありそう

