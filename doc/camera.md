
sudo apt install ros-humble-realsense2-camera

ros2 topic echo /camera/camera/accel/sample
ros2 topic echo /camera/camera/gyro/sample

ros2 run rqt_image_view rqt_image_view --force-discover /camera/color/image_raw
ros2 run rqt_image_view rqt_image_view --force-discover /camera/aligned_depth_to_color/image_raw

表示するトピック:
カラー画像: /camera/color/image_raw
深度画像: /camera/depth/image_rect_raw

pointcloud
  frame_id: camera_depth_optical_frame

