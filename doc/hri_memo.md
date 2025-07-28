# ros4hri
https://github.com/OscarMrZ/ros4hri-tutorials
https://github.com/OscarMrZ/ros4hri_ws_exercises

# hri_face_detect
## インストール方法
sudo apt install  ros-humble-hri ros-humble-hri-msgs ros-humble-hri-privacy-msgs -y
cd ~/ros2_ws/src
git clone https://github.com/ros4hri/hri_face_detect.git
git clone https://github.com/pal-robotics/launch_pal.git
cd ~/ros2_ws
colcon build
source ./install/setup.bash
 # hri_face_detect の requirements.txt を元にインストールした方がいいかも
pip install mediapipe

## 起動方法
ros2 run image_transport republish raw \
  in:=/camera/color/image_raw \
  out:=/image

ros2 run image_transport republish raw \
  in:=/camera/color/camera_info \
  out:=/camera_info

ros2 run tf2_ros static_transform_publisher \
  0 0 0   0 0 0   base_link map

ros2 launch hri_face_detect face_detect.launch.py

## topic (LLM結果)
下記のトピックを購読すれば、顔検出の 領域（ROI）, 特徴点（Landmarks）, 切り出し画像（Cropped/Aligned）, 追跡状況（Tracked）, ノード診断情報（Diagnostics） を一通り取得できます。これらを組み合わせることで、顔認識やヒューマン–ロボットインタラクションの高度な処理を実装できます。

### /humans/faces/<faceID>/roi
メッセージ型：hri_msgs/NormalizedRegionOfInterest2D
内容：
header：元画像の取得時刻とフレームID
xmin, ymin：ROI の左上頂点の正規化座標（［0,1］）
xmax, ymax：ROI の右下頂点の正規化座標（［0,1］）
c：検出信頼度（0.0～1.0）
意味：検出または追跡された顔領域を、元画像サイズに依存しない［0,1］正規化座標系で表現します。
更新頻度：画像処理レート（デフォルト30 Hz）に合わせて毎フレームまたは検出更新時に配信されます。
利用例：ros2 topic echo /humans/faces/f00001/roi で ROI 情報を取得できます。 
docs.ros.org
raw.githubusercontent.com

### /humans/faces/<faceID>/landmarks
メッセージ型：hri_msgs/FacialLandmarks
内容：
header：ROI と同じタイムスタンプ・フレームID
landmarks：長さ 70 の配列。各要素は NormalizedPointOfInterest2D（x, y が正規化座標、c が信頼度）
width, height：元画像の幅・高さ（ピクセル）
意味：顔上の特徴点（目尻や口角など ROS4HRI 標準の 70 点）を、正規化座標で提供します。
更新頻度：ROI と同様、検出・メッシュ処理が行われるフレームで配信。
利用例：ros2 topic echo /humans/faces/f00001/landmarks で顔ランドマークを参照可能です。 
docs.ros.org
raw.githubusercontent.com

### /humans/faces/<faceID>/cropped
メッセージ型：sensor_msgs/Image
内容：
顔領域を切り出し、指定サイズ（デフォルト 128×128）にリサイズしたカラー画像（encoding="bgr8"）
header：ROI と同じタイムスタンプ・フレームID
意味：顔認識や可視化用途に適した、切り出し＆縮小後の顔画像を提供します。
更新頻度：購読ノードがいる場合にのみ配信。購読数０ならスキップされます。
利用例：RViz の Image プラグインや rqt_image_view で可視化できます。 
docs.ros.org
raw.githubusercontent.com

### /humans/faces/<faceID>/aligned
メッセージ型：sensor_msgs/Image
内容：
顔を左右の目位置で正規化回転・スケーリングし、指定サイズ（デフォルト 128×128）に出力したカラー画像（encoding="bgr8"）
header：ROI と同じタイムスタンプ・フレームID
意味：顔の向きを揃え（DeepFace ベース）、認証モデル入力などに適した整列済み顔画像を提供します。
更新頻度：購読ノードがいる場合にのみ配信。購読数０ならスキップ。
利用例：顔認識パイプラインの前処理として利用。 
docs.ros.org
raw.githubusercontent.com

### /humans/faces/tracked
メッセージ型：hri_msgs/IdsList
内容：
header：タイムスタンプ・フレームID
ids：現在検出・追跡中のすべての顔ID（文字列配列）
意味：どの顔IDがアクティブかを一覧化。複数人検出時の管理に便利です。
更新頻度：顔検出／追跡結果の変化時に配信。
利用例：ros2 topic echo /humans/faces/tracked で現在の追跡顔一覧を取得。 
docs.ros.org
raw.githubusercontent.com

### /diagnostics
メッセージ型：diagnostic_msgs/DiagnosticArray
内容：
header：メッセージ生成時刻
status[]：DiagnosticStatus の配列。各要素は
name：コンポーネント名（例 "hri_face_detect/performance"）
level：OK(0), WARN(1), ERROR(2), STALE(3)
message：状態説明（例 "detection time: 120 ms"）
hardware_id：識別子
values[]：KeyValue 型配列。処理時間やフレームレートなどの詳細パラメータを格納
意味：顔検出ノードの健全性や処理レイテンシをモニタリング。閾値超過時には WARN/ERROR レベルで警告を発信します。
更新頻度：既定で 1 Hz（DIAG_PUB_RATE=1）。
利用例：rviz の Diagnostics プラグインや ros2 topic echo /diagnostics で確認可能。 
docs.ros2.org
raw.githubusercontent.com

# hri_person_manager
## インストール
hri_face_detect, hri_face_identificationが動いているのが前提

sudo apt install libdlib-dev -y
cd ~/ros2_ws/src
git clone https://github.com/ros4hri/hri_person_manager
cd ~/ros2_ws
colcon build
source ./install/setup.bash

## 実行
ros2 launch hri_person_manager person_manager.launch.py

■[ERROR] [launch_ros.actions.lifecycle_node]: Failed to make transition 'TRANSITION_CONFIGURE' for LifecycleNode '/hri_person_manager'と表示され起動できない
プロセスが残ったままなので、プロセスを探してkillする

## ライフサイクルの確認
ros2 lifecycle get /hri_person_manager

# hri_face_identification
## インストール
hri_face_detectが動いているのが前提

sudo apt install -y libtclap-dev
cd ~/ros2_ws/src
git clone https://github.com/ros4hri/hri_face_identification.git
cd ~/ros2_ws
colcon build
source ./install/setup.bash
## 起動方法
ros2 launch hri_face_identification hri_face_identification.launch.py


## 起動方法？ 未確認
HRIプラグイン付きRViz:
apt install ros-humble-rviz2
apt install ros-humble-hri-rviz
rviz2

# hri_engagement
## インストール
hri_person_manager が動いているのが前提

sudo apt install ros-humble-pyhri ros-humble-hri-actions-msgs -y

cd ~/ros2_ws/src
git clone https://github.com/ros4hri/hri_engagement.git
cd ~/ros2_ws
colcon build
source ./install/setup.bash

## 実行
ros2 launch hri_engagement hri_engagement.launch.py

## 解析結果(LLM)
hri_engagement は、ROS4HRI に準拠したノードで、カメラの視野内にいる人物とロボットとの「視覚的社会的エンゲージメント」をリアルタイムに算出し、2 秒間の時系列フィルタリングを経てそのレベルを判定します
github.com
.

公開トピック（Published topics）
/humans/persons/<person_id>/engagement_status

メッセージ型：hri_msgs/EngagementLevel

内容：各人物のエンゲージメントレベル（UNKNOWN, DISENGAGED, ENGAGING, ENGAGED, DISENGAGING のいずれか）を公開します
github.com
.

/intents

メッセージ型：hri_actions_msgs/Intent

内容：人物がエンゲージ（注視）したと判定された際に ENGAGE_WITH インテントを送出します

##  ros2 node info /hri_engagement_listener
$ ros2 node info /hri_engagement_listener
/hri_engagement_listener
  Subscribers:
    /humans/bodies/tracked: hri_msgs/msg/IdsList
    /humans/faces/tracked: hri_msgs/msg/IdsList
    /humans/persons/aifku/alias: std_msgs/msg/String
    /humans/persons/aifku/anonymous: std_msgs/msg/Bool
    /humans/persons/aifku/body_id: std_msgs/msg/String
    /humans/persons/aifku/engagement_status: hri_msgs/msg/EngagementLevel
    /humans/persons/aifku/face_id: std_msgs/msg/String
    /humans/persons/aifku/location_confidence: std_msgs/msg/Float32
    /humans/persons/aifku/voice_id: std_msgs/msg/String
省略
    /humans/voices/tracked: hri_msgs/msg/IdsList
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /tf: tf2_msgs/msg/TFMessage
    /tf_static: tf2_msgs/msg/TFMessage
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /hri_engagement_listener/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /hri_engagement_listener/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /hri_engagement_listener/get_parameters: rcl_interfaces/srv/GetParameters
    /hri_engagement_listener/list_parameters: rcl_interfaces/srv/ListParameters
    /hri_engagement_listener/set_parameters: rcl_interfaces/srv/SetParameters
    /hri_engagement_listener/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically

# ROS4HRI tutorials
## web
https://ros4hri.github.io/ros4hri-tutorials/

## docker hub
https://hub.docker.com/layers/palrobotics/public-tutorials-alum-devel/hri25/images/sha256-4261286ffc12f4bd0e1a98452eb80c21625e6169ae5fc5ee7d8034ef90442b9d

## Using the PAL public Docker image
https://docs.pal-robotics.com/edge/development/docker-public

# ROS4HRI Complete walk-through (古い)
https://wiki.ros.org/hri/Tutorials/ROS4HRI-walk-through?utm_source=chatgpt.com

# 関連
## TIAGo 
https://github.com/pal-robotics/tiago_simulation
