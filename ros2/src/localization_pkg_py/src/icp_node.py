#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

# ✅ Import your ICPTester from the module
from icp_module import ICPTester


class ICPLocalizationNode(Node):
    def __init__(self):
        super().__init__('icp_localization_node')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar_points',  # <-- Update this to your LiDAR topic name if different
            self.lidar_callback,
            10
        )

        self.icp = ICPTester(voxel_size=0.2, logger=self.get_logger())
        self.get_logger().info(" ICP Localization Node Started")

    def lidar_callback(self, msg):
        points = self.pointcloud2_to_xyz(msg)
        if hasattr(self, 'last_points'):
            delta = np.mean(np.linalg.norm(points - self.last_points, axis=1))
            print(f"[DEBUG] Point delta from last frame: {delta:.4f}")
        self.last_points = points

        if points is not None:
            print(f"[DEBUG] First point: {points[0]}")
            print(f"[DEBUG] Mean distance: {np.mean(np.linalg.norm(points, axis=1)):.2f}")
            self.icp.register(points)

    @staticmethod
    def pointcloud2_to_xyz(msg):
        points = []
        for pt in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")):
            points.append([pt[0], pt[1], pt[2]])
        return np.array(points)


def main(args=None):
    rclpy.init(args=args)
    node = ICPLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down ICP node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()