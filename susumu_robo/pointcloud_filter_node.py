import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np


POINT_STEP = 20  # x(4) + y(4) + z(4) + padding(4) + rgb(4)


class PointcloudFilterNode(Node):
    def __init__(self):
        super().__init__('pointcloud_filter_node')

        self.declare_parameter('voxel_size', 0.05)  # ダウンサンプリングの格子サイズ(m)
        self.declare_parameter('z_min', 0.1)        # 床面ノイズを除去する最低高さ(m)
        self.declare_parameter('z_max', 2.0)        # 天井など不要な点を除去する最大高さ(m)
        self.declare_parameter('max_range', 3.0)    # D435iの信頼できる最大距離(m)

        self.voxel_size = self.get_parameter('voxel_size').value
        self.z_min = self.get_parameter('z_min').value
        self.z_max = self.get_parameter('z_max').value
        self.max_range = self.get_parameter('max_range').value

        self.sub = self.create_subscription(
            PointCloud2,
            '/depth_camera/depth/color/points',
            self.callback,
            10,
        )
        self.pub = self.create_publisher(
            PointCloud2,
            '/depth_camera/points/downsampled',
            10,
        )

    def callback(self, msg):
        # バイト列を直接 numpy で読む（read_points のPythonループを回避）
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
        xyz = np.frombuffer(raw[:, :12].tobytes(), dtype=np.float32).reshape(-1, 3)

        # NaN除去
        valid = np.isfinite(xyz).all(axis=1)
        xyz = xyz[valid]
        raw = raw[valid]
        if xyz.shape[0] == 0:
            return

        x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]

        # 距離・高さフィルタ
        mask = (
            (x * x + y * y + z * z <= self.max_range ** 2) &  # sqrtを避けて二乗比較
            (z >= self.z_min) &
            (z <= self.z_max)
        )
        xyz = xyz[mask]
        raw = raw[mask]
        if xyz.shape[0] == 0:
            return

        # voxelダウンサンプリング: 格子ごとに1点だけ残す
        voxel_idx = np.floor(xyz / self.voxel_size).astype(np.int32)
        # ユニーク判定をハッシュで高速化
        keys = (voxel_idx[:, 0].astype(np.int64) * 1000003
                + voxel_idx[:, 1].astype(np.int64) * 1009
                + voxel_idx[:, 2].astype(np.int64))
        _, unique_idx = np.unique(keys, return_index=True)
        raw = raw[unique_idx]

        # バイト列を直接出力（tolist変換を回避）
        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = len(raw)
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.point_step * len(raw)
        out.data = raw.tobytes()
        out.is_dense = False
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PointcloudFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
