from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. LaserScan → PointCloud2
        Node(
            package='localization_pkg_py',
            executable='convert2pointcloud2.py',
            name='laser_to_pointcloud_node',
            output='screen'
        ),

        # 2. ICP Node
        Node(
            package='localization_pkg_py',
            executable='icp_node.py',
            name='icp_localization_node',
            output='screen'
        ),

        # 3. Optionally add ground truth or visualization later
    ])
