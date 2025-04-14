#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
import math
import struct

class LaserToPointCloud(Node):
    def __init__(self):
        super().__init__('laser_to_pointcloud')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        self.publisher = self.create_publisher(PointCloud2, '/lidar_points', 10)
        self.get_logger().info("✅ LaserScan to PointCloud2 node started")

    def laser_callback(self, scan):
        points = []
        angle = scan.angle_min

        for r in scan.ranges:
            if math.isfinite(r):
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0.0
                points.append((x, y, z))
            angle += scan.angle_increment

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_data = b''.join([struct.pack('fff', *point) for point in points])

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = scan.header.frame_id

        pointcloud = PointCloud2()
        pointcloud.header = header
        pointcloud.height = 1
        pointcloud.width = len(points)
        pointcloud.fields = fields
        pointcloud.is_bigendian = False
        pointcloud.point_step = 12
        pointcloud.row_step = 12 * len(points)
        pointcloud.data = cloud_data
        pointcloud.is_dense = True

        self.publisher.publish(pointcloud)

def main(args=None):
    rclpy.init(args=args)
    node = LaserToPointCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
