# led_controller_node
## led_controller_node 起動方法
```bash
ros2 run susumu_robo led_controller_node
```

## blinkstick_node 起動方法
```bash
ros2 run susumu_blinkstick_ros2 blinkstick_node
```

## トピックの接続確認
```bash
ros2 topic info /scan_in_range
ros2 topic info /led
```

## LEDコマンドの手動テスト
```bash
ros2 topic pub /led susumu_ros2_interfaces/msg/LED "pattern: 'SOLID'
color1: 'green'
color2: 'black'
duration: 5.0
priority: 1
decay_rate: 0.0
speed: 1.0" --once
```

## 動作確認
### /scan_in_rangeに false を設定
```bash
ros2 topic pub /scan_in_range std_msgs/Bool "data: false" --once
```

### /scan_in_rangeに false を設定
```bash
ros2 topic pub /scan_in_range std_msgs/Bool "data: true" --once
```
