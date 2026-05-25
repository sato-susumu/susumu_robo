# ノード・トピック関係図（robo_indoor 構成）

`robo_indoor.launch.py` で起動される全ノードとトピックの接続関係。

## 全体フロー

```mermaid
flowchart TD
    classDef hw fill:#4a4a6a,stroke:#9090c0,color:#fff
    classDef lidar fill:#1a4a1a,stroke:#50a050,color:#fff
    classDef filter fill:#1a3a4a,stroke:#4090b0,color:#fff
    classDef safety fill:#4a1a1a,stroke:#c05050,color:#fff
    classDef drive fill:#3a2a1a,stroke:#c09050,color:#fff
    classDef imu fill:#2a1a4a,stroke:#8050c0,color:#fff
    classDef nav fill:#1a3a3a,stroke:#40a0a0,color:#fff

    subgraph HW["ハードウェア"]
        LIDAR["Livox Mid-360"]:::hw
        IMU_HW["IMU (WT901)"]:::hw
        MOTOR_HW["BotWheel モーター"]:::hw
        JOY_HW["Logicool F710"]:::hw
    end

    subgraph mid360["mid360.launch.py (1秒後)"]
        livox_driver["livox_lidar_publisher\n(livox_ros_driver2)"]:::lidar
        livox_conv["livox_to_pointcloud2_node"]:::lidar
        pc2scan["pointcloud_to_laserscan"]:::lidar
        imu_conv["livox_imu_converter"]:::imu
    end

    subgraph laser_filter["laser_filter.launch.py (2秒後)"]
        lf["scan_to_scan_filter_chain\n(laser_filters)\n\n後方右側: -180 〜 -135 deg → NaN\n後方左側: +163 〜 +180 deg → NaN"]:::filter
    end

    subgraph collision["collision_monitor.launch.py (4秒後)"]
        lsf["laserscan_filter_node\n障害物検出・速度フィルタ統合\n前方障害物→前進ブロック\n後方障害物→後退ブロック"]:::safety
    end

    subgraph base["base.launch.py (4秒後)"]
        ts1["twist_stamper\n(/cmd_vel → /botwheel_explorer/cmd_vel)"]:::drive
    end

    subgraph botwheel["botwheel_teleop.launch.py (6秒後)"]
        joy["joy_node"]:::drive
        teleop["teleop_twist_joy_node\n(TwistStamped出力)"]:::drive
        botwheel_ctrl["botwheel_explorer\nモータードライバ"]:::drive
    end

    subgraph imu["imu_wt901.launch.py (5秒後)"]
        imu_node["witmotion IMU node"]:::imu
    end

    subgraph key["key_event_system.launch.py (6秒後)"]
        key_node["key_event_handler"]:::drive
    end

    nav2["Nav2"]:::nav
    odom_relay["odom_topic_relay"]:::nav

    %% ハードウェア接続
    LIDAR -->|"USB/LAN"| livox_driver
    IMU_HW -->|"USB"| imu_node
    JOY_HW -->|"USB レシーバー"| joy

    %% LiDAR パイプライン
    livox_driver -->|"/livox/lidar\n(CustomMsg)"| livox_conv
    livox_conv -->|"/converted_pointcloud2\n(PointCloud2)"| pc2scan
    pc2scan -->|"/scan_raw\n(LaserScan)"| lf
    lf -->|"/scan\n(LaserScan, フィルター済み)"| lsf

    %% 障害物検出
    lsf -->|"/scan_in_range\n(Bool)"| lsf
    lsf -->|"/scan_range_polygon\n(PolygonStamped)"| lsf

    %% コマンド速度フロー
    joy -->|"/joy"| teleop
    teleop -->|"/input_twist\n(TwistStamped)"| lsf
    lsf -->|"/botwheel_explorer/cmd_vel\n(TwistStamped, 障害物時ゼロ化)"| botwheel_ctrl

    %% モーター制御
    botwheel_ctrl -->|"RS485"| MOTOR_HW

    %% オドメトリ
    botwheel_ctrl -->|"/botwheel_explorer/odom"| odom_relay
    odom_relay -->|"/odom"| nav2

    %% IMU
    imu_node -->|"/imu\n(Imu)"| nav2
    livox_driver -->|"/livox/imu\n(Imu raw)"| imu_conv
```

## トピック一覧

| トピック | 型 | Publisher | Subscriber | 備考 |
|---------|-----|-----------|------------|------|
| `/livox/lidar` | `CustomMsg` | livox_lidar_publisher | livox_to_pointcloud2_node | Livox独自形式 |
| `/converted_pointcloud2` | `PointCloud2` | livox_to_pointcloud2_node | pointcloud_to_laserscan | |
| `/scan_raw` | `LaserScan` | pointcloud_to_laserscan | scan_to_scan_filter_chain | フィルター前の生データ |
| `/scan` | `LaserScan` | scan_to_scan_filter_chain | laserscan_filter_node, amcl, costmap 等 | **フィルター済み** |
| `/scan_in_range` | `Bool` | laserscan_filter_node | — | 障害物検出フラグ（内部状態管理に使用） |
| `/scan_range_polygon` | `PolygonStamped` | laserscan_filter_node | — | 検出範囲の可視化用 |
| `/joy` | `Joy` | joy_node | teleop_twist_joy_node | |
| `/input_twist` | `TwistStamped` | teleop_twist_joy_node | laserscan_filter_node | ゲームパッド入力 |
| `/botwheel_explorer/cmd_vel` | `TwistStamped` | laserscan_filter_node | botwheel_explorer | 障害物時に linear をゼロ化済み |
| `/botwheel_explorer/odom` | `Odometry` | botwheel_explorer | odom_topic_relay | |
| `/odom` | `Odometry` | odom_topic_relay | controller_server, bt_navigator | |
| `/imu` | `Imu` | witmotion | Nav2 | |
| `/livox/imu` | `Imu` | livox_lidar_publisher | livox_imu_converter | G単位 |
| `/livox/imu_ms2` | `Imu` | livox_imu_converter | — | m/s²変換済み（outdoor用） |

## laser_filter の設定

`config/laser_filter.yaml` で定義。

| フィルター | パラメータ | 値 | 意味 |
|-----------|-----------|-----|------|
| filter1（後方右） | `lower_angle` / `upper_angle` | -3.1416 / -2.3562 rad（-180° 〜 -135°） | 後方右側ボディノイズをNaN置換 |
| filter2（後方左） | `lower_angle` / `upper_angle` | 2.8448 / 3.1416 rad（+163° 〜 +180°） | 後方左側ボディノイズをNaN置換 |
| filter3（スペックル） | `filter_type` / `max_range_difference` / `filter_window` | 1 / 0.3 / 2 | 散発的な外れ値を除去 |

ロボット後方両側に出るボディ由来の近接ノイズ（0.1〜0.2m）を除去する。

## 起動タイミング

```mermaid
gantt
    title robo_indoor.launch.py 起動シーケンス
    dateFormat s
    axisFormat %Ss

    mid360              : 1, 2s
    laser_filter        : 2, 3s
    bringup_diagnostic  : 2, 3s
    base                : 4, 5s
    collision_monitor   : 4, 5s
    foxglove_bridge     : 4, 5s
    imu_wt901           : 5, 6s
    botwheel_teleop     : 6, 7s
    key_event_system    : 6, 7s
```
