# Susumu Robo スクリプトドキュメント

このドキュメントは、`susumu_robo`パッケージ内の各Pythonスクリプトの詳細な説明を提供します。

## 目次

1. [laserscan_filter_node.py](#laserscan_filter_nodepy) - LiDARデータのフィルタリング
2. [twist_filter_node.py](#twist_filter_nodepy) - 速度コマンドの安全フィルタ
3. [led_controller_node.py](#led_controller_nodepy) - LED制御
4. [tenkey_publisher.py](#tenkey_publisherpy) - テンキー入力制御
5. [laser_scan_detect_test.py](#laser_scan_detect_testpy) - 物体検出テスト
6. [laser_scan_test.py](#laser_scan_testpy) - LaserScan表示テスト
7. [key_event_handler.py](#key_event_handlerpy) - キーイベント処理
8. [livox_imu_converter.py](#livox_imu_converterpy) - IMU単位変換
9. [ntrip_str2str_node.py](#ntrip_str2str_nodepy) - NTRIP補正データ配信
10. [number_key_publisher.py](#number_key_publisherpy) - 数字キー入力配信
11. [robo_doctor_node.py](#robo_doctor_nodepy) - システム診断
12. [dummy_navsatfix_publisher.py](#dummy_navsatfix_publisherpy) - ダミーGNSS配信
13. [imu_visualizer.py](#imu_visualizerpy) - IMUデータ可視化
14. [to_human_2_speak_ros_node.py](#to_human_2_speak_ros_nodepy) - `/to_human` を speak_ros アクションへ橋渡し

---

## laserscan_filter_node.py

### 概要
LiDARからのスキャンデータをフィルタリングし、ロボットの周囲の指定領域内に障害物があるかを検出するノードです。前進・後進の移動方向に応じて検出領域を動的に切り替えます。

### 機能
- LiDARデータの座標変換（laser_frame → livox_frame）
- 前進/後進に応じた検出範囲の切り替え
- 検出範囲の速度に応じた動的調整
- 検出結果の可視化用データ生成

### ROS2インターフェース

#### 購読トピック
| トピック名 | 型 | 説明 |
|-----------|-----|------|
| `/scan` | `sensor_msgs/LaserScan` | LiDARからのスキャンデータ |
| `/input_twist` | `geometry_msgs/TwistStamped` | 速度コマンド（移動方向判定用） |

#### 公開トピック
| トピック名 | 型 | 説明 |
|-----------|-----|------|
| `/scan_in_range` | `std_msgs/Bool` | 指定範囲内に障害物があるかのフラグ |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | フィルタリング後の速度コマンド |
| `/detected_points` | `sensor_msgs/LaserScan` | 検出された点群のみを含むLaserScan |
| `/scan_range_polygon` | `geometry_msgs/PolygonStamped` | 検出範囲の可視化用ポリゴン |

### パラメータ

```yaml
reference_link: 'livox_frame'  # 座標変換の基準フレーム

forward:  # 前進時の検出範囲
  x_min: -0.22
  x_max: 0.22
  y_min: -0.22
  y_max: 0.22

backward:  # 後進時の検出範囲
  x_min: -0.22
  x_max: 0.22
  y_min: -0.22
  y_max: 0.22
```

### 処理フロー

```mermaid
graph TD
    A[LaserScanデータ受信] --> B[cmd_velから移動方向判定]
    B --> C{前進/後進?}
    C -->|前進| D[前進用検出範囲選択]
    C -->|後進| E[後進用検出範囲選択]
    D --> F[速度に応じて範囲調整]
    E --> F
    F --> G[各点をlivox_frame座標系に変換]
    G --> H[範囲内チェック]
    H --> I[結果をBoolで公開]
    H --> J[検出点群を公開]
    H --> K[検出範囲ポリゴンを公開]
```

### 使用例
```bash
ros2 run susumu_robo laserscan_filter_node
```

---

## twist_filter_node.py

### 概要
障害物が検出された際に、ロボットの直進移動（前進・後進）を停止させる安全フィルタです。回転動作は許可されるため、その場での方向転換は可能です。

### 機能
- 障害物検出時の自動停止
- 回転動作の維持
- フィルタリングの有効/無効切り替え

### ROS2インターフェース

#### 購読トピック
| トピック名 | 型 | 説明 |
|-----------|-----|------|
| `/input_twist` | `geometry_msgs/Twist` | フィルタリング前の速度コマンド |
| `/enable` | `std_msgs/Bool` | フィルタリングの有効/無効フラグ |

#### 公開トピック
| トピック名 | 型 | 説明 |
|-----------|-----|------|
| `/output_twist` | `geometry_msgs/Twist` | フィルタリング後の速度コマンド |

### 処理フロー

```mermaid
graph TD
    A[速度コマンド受信] --> B{フィルタ有効?}
    B -->|Yes| C[linear.x/y/z = 0.0]
    B -->|No| D[入力をそのまま使用]
    C --> E[angular保持]
    E --> F[output_twistに公開]
    D --> F
```

### 動作詳細
- `enable = True`: 直進成分（linear）を0に設定、回転成分（angular）は維持
- `enable = False`: 入力をそのまま出力

### 使用例
```bash
ros2 run susumu_robo twist_filter_node
```

---

## led_controller_node.py

### 概要
障害物の検出状態に応じてLEDの点灯パターンを制御します。障害物検出時は黄色で点滅、非検出時は消灯します。

### 機能
- 障害物検出状態の監視
- LED点滅パターンの制御
- 状態変化時の即座な反応

### ROS2インターフェース

#### 購読トピック
| トピック名 | 型 | 説明 |
|-----------|-----|------|
| `/scan_in_range` | `std_msgs/Bool` | 障害物検出フラグ |

#### 公開トピック
| トピック名 | 型 | 説明 |
|-----------|-----|------|
| `/led` | `susumu_ros2_interfaces/LED` | LED制御コマンド |

### LED制御パターン

| 状態 | 動作 | 色 | 点滅速度 | 継続時間 | 優先度 |
|------|------|-----|----------|----------|--------|
| 障害物検出 | 点滅 | 黄色 | 3.0 | 60秒 | 通常 |
| 障害物なし | 消灯 | - | - | - | 99999（最高） |

### 処理フロー

```mermaid
graph TD
    A[scan_in_range受信] --> B{状態変化?}
    B -->|変化なし| A
    B -->|False→True| C[黄色点滅開始]
    B -->|True→False| D[LED消灯]
    C --> E[60秒間点滅]
    D --> F[即座に消灯]
```

### 使用例
```bash
ros2 run susumu_robo led_controller_node
```

---

## tenkey_publisher.py

### 概要
USBテンキーパッドを使用してロボットを手動操作するためのコントローラです。矢印キーで前後左右の移動、5キーで停止が可能です。

### 機能
- USBテンキーからの入力読み取り
- キー入力に応じた速度コマンド生成
- 同時押し防止（最新のキー入力のみ有効）

### ROS2インターフェース

#### 公開トピック
| トピック名 | 型 | 説明 |
|-----------|-----|------|
| `/turtle1/cmd_vel` | `geometry_msgs/Twist` | 速度コマンド |

### パラメータ

```yaml
keyboard_device_path: '/dev/input/by-id/usb-05a4_Tenkey_Keyboard-event-kbd'
cmd_vel_topic: '/turtle1/cmd_vel'
linear_speed: 1.0  # 直進速度 [m/s]
angular_speed: 1.0  # 回転速度 [rad/s]
```

### キーマッピング

| キー | 動作 | linear.x | angular.z |
|------|------|----------|-----------|
| 8 (↑) | 前進 | +1.0 | 0.0 |
| 2 (↓) | 後進 | -1.0 | 0.0 |
| 4 (←) | 左回転 | 0.0 | +1.0 |
| 6 (→) | 右回転 | 0.0 | -1.0 |
| 5 | 停止 | 0.0 | 0.0 |

### 必要な権限
```bash
# ユーザーをinputグループに追加
sudo usermod -a -G input $USER
# 再ログインが必要
```

### 使用例
```bash
ros2 run susumu_robo tenkey_publisher
```

---

## laser_scan_detect_test.py

### 概要
LaserScanデータから前方、後方、近距離の物体を検出し、結果をログに出力するテストツールです。

### 機能
- 前方物体検出（±90度、0.5m以内）
- 後方物体検出（±90度より外側、0.5m以内）
- 近距離物体検出（全方向、0.3m以内）

### ROS2インターフェース

#### 購読トピック
| トピック名 | 型 | 説明 |
|-----------|-----|------|
| `/scan` | `sensor_msgs/LaserScan` | LiDARデータ |

### 検出条件

| 検出タイプ | 距離条件 | 角度条件 |
|-----------|---------|---------|
| 前方物体 | < 0.5m | -90° ≤ θ ≤ 90° |
| 後方物体 | < 0.5m | θ < -90° または θ > 90° |
| 近距離物体 | < 0.3m | 全方向 |

### 出力形式
```
前方に物体を検出: [(距離1, 角度1), (距離2, 角度2), ...]
後方に物体を検出: [(距離1, 角度1), (距離2, 角度2), ...]
近距離に物体を検出: [(距離1, 角度1), (距離2, 角度2), ...]
```

### 使用例
```bash
ros2 run susumu_robo laser_scan_detect_test
```

---

## laser_scan_test.py

### 概要
LaserScanデータの一部を段階的に表示するテストツールです。表示範囲を10%ずつ増減させることで、LiDARデータの動作確認ができます。

### 機能
- LaserScanデータの部分表示
- 表示範囲の自動循環（0% → 100% → 0%）
- 非表示部分は無限大として処理

### ROS2インターフェース

#### 購読トピック
| トピック名 | 型 | 説明 |
|-----------|-----|------|
| `/scan` | `sensor_msgs/LaserScan` | 元のLiDARデータ |

#### 公開トピック
| トピック名 | 型 | 説明 |
|-----------|-----|------|
| `/modified_scan` | `sensor_msgs/LaserScan` | 加工されたLiDARデータ |

### 動作パターン

```
時刻 0s: 0%表示（全データ無限大）
時刻 1s: 10%表示（最初の10%のみ有効）
時刻 2s: 20%表示
...
時刻 10s: 100%表示（全データ有効）
時刻 11s: 0%表示（サイクル再開）
```

### 使用例
```bash
ros2 run susumu_robo laser_scan_test

# 別ターミナルで確認
ros2 topic echo /modified_scan
```

---

## システム連携図

```mermaid
graph TD
    subgraph Input
        L[LiDAR /scan]
        V[Velocity /cmd_vel]
        K[Tenkey Input]
    end
    
    subgraph Processing
        LF[laserscan_filter_node]
        TF[twist_filter_node]
        TC[tenkey_publisher]
    end
    
    subgraph Output
        LED[led_controller_node]
        M[Motors]
    end
    
    L --> LF
    V --> LF
    LF -->|/scan_in_range| TF
    LF -->|/scan_in_range| LED
    K --> TC
    TC -->|/cmd_vel| TF
    TF -->|/output_twist| M
```

## トラブルシューティング

### laserscan_filter_node
- **問題**: TFエラーが発生する
  - **解決**: `ros2 run tf2_tools view_frames`でTFツリーを確認
  - livox_frameとlaser_link間の変換が存在することを確認

### twist_filter_node
- **問題**: 速度コマンドが通らない
  - **解決**: `/enable`トピックの値を確認（`ros2 topic echo /enable`）

### led_controller_node
- **問題**: LEDが点灯しない
  - **解決**: BlinkStickの接続とUSB権限を確認

### tenkey_publisher
- **問題**: デバイスが見つからない
  - **解決**: `ls /dev/input/by-id/`でデバイスパスを確認
  - パラメータで正しいパスを設定

## key_event_handler.py

### 概要
テンキー等からのキーイベントを受け取り、rosbag録画の開始/停止やシステム診断を実行するノードです。結果はTTSで音声通知されます。

### 機能
- キー1: rosbag録画のトグル（開始/停止）
- キー2: システム診断の実行
- 録画結果・診断結果のTTS通知

### ROS2インターフェース

#### 購読トピック
| トピック名 | 型 | 説明 |
|-----------|-----|------|
| `key_event` | `std_msgs/String` | キーイベント（"1", "2"等） |

#### 公開トピック
| トピック名 | 型 | 説明 |
|-----------|-----|------|
| `to_human` | `std_msgs/String` | TTS音声合成用メッセージ |

### パラメータ

```yaml
key_event_topic: 'key_event'
robo_doctor_path: '/home/taro/ros2_ws/src/susumu_robo/launch/robo_doctor.py'
rosbag_base_dir: '~/ros2_bags'
```

### 使用例
```bash
ros2 run susumu_robo key_event_handler
```

---

## livox_imu_converter.py

### 概要
Livox Mid-360のIMUデータの加速度単位をG単位からm/s²に変換するノードです。

### 機能
- 加速度データのG単位 → m/s²変換（×9.81）
- 角速度・姿勢データはそのままコピー

### ROS2インターフェース

#### 購読トピック
| トピック名 | 型 | 説明 |
|-----------|-----|------|
| `/livox/imu` | `sensor_msgs/Imu` | Livox IMUデータ（G単位） |

#### 公開トピック
| トピック名 | 型 | 説明 |
|-----------|-----|------|
| `/livox/imu_ms2` | `sensor_msgs/Imu` | 変換後のIMUデータ（m/s²） |

### 使用例
```bash
ros2 run susumu_robo livox_imu_converter
```

---

## ntrip_str2str_node.py

### 概要
RTKLIBの`str2str`コマンドをROS2ノードとしてラップし、NTRIPサーバーからGNSS補正データを取得してGNSSレシーバーに転送するノードです。

### 機能
- str2strプロセスの起動・管理
- 接続状態の監視（C=接続/W=待機/E=エラー）
- ステータスのサービス公開

### ROS2インターフェース

#### サービス
| サービス名 | 型 | 説明 |
|-----------|-----|------|
| `~/get_status` | `susumu_ros2_interfaces/NtripStatus` | NTRIP接続状態の取得 |

### パラメータ

```yaml
ntrip_server: 'ntrip://ntrip1.bizstation.jp:2101/0C8BD4BE'
output_dest: 'tcpcli://192.168.3.1:28785'
```

### 使用例
```bash
ros2 run susumu_robo ntrip_str2str_node
```

---

## number_key_publisher.py

### 概要
USBキーボード（ゲーミングマウス等の追加ボタン）のテンキー入力を読み取り、キーイベントとして配信するノードです。`key_event_handler`と組み合わせて使用します。

### 機能
- evdevによるキーボードデバイスの読み取り
- KP1〜KP4のキーをイベントとして配信

### ROS2インターフェース

#### 公開トピック
| トピック名 | 型 | 説明 |
|-----------|-----|------|
| `key_event` | `std_msgs/String` | キーイベント（"1"〜"4"） |

### パラメータ

```yaml
keyboard_device_path: '/dev/input/by-id/usb-INSTANT_USB_GAMING_MOUSE-if01-event-kbd'
key_event_topic: 'key_event'
```

### キーマッピング

| キー | 配信値 |
|------|--------|
| KP1 | "1" |
| KP2 | "2" |
| KP3 | "3" |
| KP4 | "4" |

### 使用例
```bash
ros2 run susumu_robo number_key_publisher
```

---

## robo_doctor_node.py

### 概要
ロボットのシステム状態を定期的に診断し、`/diagnostics`トピックに結果を配信するノードです。バッテリー、ネットワーク、ROS2ノード/トピック、デバイス等を監視します。

### 機能
- バッテリー残量チェック
- ネットワーク疎通確認（LiDAR、GNSS）
- ROS2ノード・トピックの存在確認
- デバイス存在確認（IMU、ジョイスティック、CAN）
- PTP時刻同期状態確認
- Livox設定ファイル検証

### ROS2インターフェース

#### 公開トピック
| トピック名 | 型 | 説明 |
|-----------|-----|------|
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 診断結果（3秒毎に配信） |

### パラメータ

```yaml
enable_gnss_checks: true   # GNSS関連チェックの有効/無効
enable_ptp_checks: true    # PTP関連チェックの有効/無効
```

### 診断項目

| カテゴリ | 項目 | 間隔 |
|---------|------|------|
| System | Battery | 60s |
| PTP | Service, Process, Slave, Master | 60s |
| Network | LiDAR, GNSS, HostInterface | 60s |
| Device | IMU, Joystick, CAN | 60s |
| Nodes | 各期待ノード | 60s |
| Topics | 各期待トピック | 60s |
| DataFlow | LiDAR, GNSS | 60s |
| Config | Livox設定 | 60s |

### 使用例
```bash
ros2 run susumu_robo robo_doctor_node
```

---

## dummy_navsatfix_publisher.py

### 概要
GNSSノードが起動していない場合に、無効値（NaN）の`NavSatFix`メッセージを`/fix`トピックに配信するノードです。実際のGNSSノードが起動したら自動的に終了します。

### 機能
- 5秒毎にNaN座標のNavSatFixを配信
- 他のNavSatFixパブリッシャー検出時に自動終了

### ROS2インターフェース

#### 公開トピック
| トピック名 | 型 | 説明 |
|-----------|-----|------|
| `/fix` | `sensor_msgs/NavSatFix` | ダミーGNSS位置（NaN） |

### 使用例
```bash
ros2 run susumu_robo dummy_navsatfix_publisher
```

---

## imu_visualizer.py

### 概要
IMUデータをmatplotlibでリアルタイムにグラフ表示するデバッグツールです。1つまたは2つのIMUトピックを同時に表示できます。

### 機能
- 加速度（m/s²）のリアルタイムグラフ表示
- 角速度（rad/s）のリアルタイムグラフ表示
- 姿勢（Roll/Pitch/Yaw、度）のリアルタイムグラフ表示
- 2トピックの同時比較表示（デュアルモード）

### ROS2インターフェース

#### 購読トピック
| トピック名 | 型 | 説明 |
|-----------|-----|------|
| TOPIC1（引数指定） | `sensor_msgs/Imu` | 1つ目のIMUデータ |
| TOPIC2（引数指定） | `sensor_msgs/Imu` | 2つ目のIMUデータ（省略可） |

### 使用例
```bash
# デフォルト（/imu と /livox/imu_ms2 を比較）
python3 imu_visualizer.py

# 単一トピック
python3 imu_visualizer.py /my_imu

# 2トピック比較
python3 imu_visualizer.py /imu1 /imu2
```

---

## to_human_2_speak_ros_node.py

### 概要
`/to_human` トピックで届いたテキストを句読点で文ごとに分割し、`speak_ros` の `Speak` アクションへ順次送って音声合成（AivisSpeech）させるブリッジノードです。

### 機能
- `、。！？` を区切り文字とした文単位の分割
- アクション完了を待ってから次の文を送るキュー処理
- `speak_ros` のアクションサーバ起動を待ってからサブスクライブ開始

### ROS2インターフェース

#### 購読トピック
| トピック名 | 型 | 説明 |
|-----------|-----|------|
| `/to_human` | `std_msgs/String` | 読み上げ対象テキスト |

#### アクションクライアント
| アクション名 | 型 | 説明 |
|-----------|-----|------|
| `/speak` | `speak_ros_interfaces/action/Speak` | 1 文ずつ送信して逐次発話 |

### パラメータ

```yaml
input_topic: '/to_human'
```

### 使用例
```bash
ros2 run susumu_robo to_human_2_speak_ros
```

---

## 関連ファイル

- **設定ファイル**: `config/`ディレクトリ内のYAMLファイル
- **起動ファイル**: `launch/`ディレクトリ内の各launch.pyファイル