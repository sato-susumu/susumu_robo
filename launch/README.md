# _install_for_startup.shの注意点
/home/taro/ros2_ws/install/setup.bashに依存しています。いずれ、修正が必要

# 問題発生時の対処法
## systemdで登録したサービスで音声が再生できない
```sudo vi /lib/systemd/system/ros2_audio.service
```ファイルの末尾に次の内容を追記します

例：Userがtaro、uidが1000の場合
```[Service]
Environment="PULSE_SERVER=unix:/run/user/1000/pulse/native"
Environment="XDG_RUNTIME_DIR=/run/user/1000"
User=taro
```
修正後は再起動し、オーディオを使った処理が動くことを確認する
