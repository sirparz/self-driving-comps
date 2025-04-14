#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
import numpy as np

class LaserScanToPointCloud(Node):
    def __init__(self):
        super().__init__('scan_to_pointcloud')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.publisher = self.create_publisher(PointCloud2, '/pointcloud', 10)
        self.get_logger().info("Listening to /scan...")

    def scan_callback(self, msg):
        # Convert LaserScan to 2D point cloud (x, y)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)

        # Filter out invalid ranges
        valid = np.logical_and(ranges > msg.range_min, ranges < msg.range_max)
        if not np.any(valid):
            self.get_logger().warn("No valid points in scan")
            return

        ranges = ranges[valid]
        angles = angles[valid]

        # Polar to Cartesian
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        points = np.stack((xs, ys, np.zeros_like(xs)), axis=-1)  # Shape: (N, 3)

        self.get_logger().info(f"Received scan with {points.shape[0]} valid points")
        self.get_logger().debug(f"First few points: {points[:5]}")

        # Publish the point cloud
        self.publish_pointcloud(points)

    def publish_pointcloud(self, points):
        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header.frame_id = "laser_frame"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.height = 1
        msg.width = points.shape[0]
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * points.shape[0]
        msg.is_dense = True
        msg.data = np.asarray(points, dtype=np.float32).tobytes()

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanToPointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()