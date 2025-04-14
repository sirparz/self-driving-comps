import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R


class ICPTester:
    def __init__(self, voxel_size=0.08):
        self.voxel_size = voxel_size
        self.last_pcd = None
        self.current_transform = np.eye(4)

        # ICP Configuration
        self.icp_config = {
            "max_correspondence_distance": 0.5,
            "estimation_method": o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            "criteria": o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=50,
                relative_fitness=1e-6,
                relative_rmse=1e-6,
            ),
        }

    def preprocess(self, points):
        """Convert numpy point array into a downsampled and filtered point cloud"""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Remove noise
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=30, std_ratio=1.5)

        # Downsample
        pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)

        # Estimate normals for point-to-plane ICP
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=50),
            fast_normal_computation=True
        )

        return pcd

    def register(self, points, gt_matrix=None):
        """Register new LiDAR scan with previous one and optionally compare with ground truth"""
        if len(points) < 1000:
            print("[ICP] Not enough points to run.")
            return None

        current_pcd = self.preprocess(points)

        if self.last_pcd is None:
            self.last_pcd = current_pcd
            print("[ICP] First scan stored.")
            return np.eye(4)

        result = o3d.pipelines.registration.registration_icp(
            current_pcd, self.last_pcd, **self.icp_config
        )

        self.last_pcd = current_pcd
        self.current_transform = self.current_transform @ result.transformation

        self._print_pose(result.transformation , gt_matrix)

        return self.current_transform

    def _print_pose(self, result, gt_matrix=None):
        trans = self.result[:3, 3]
        rot = self.result[:3, :3]
        euler = R.from_matrix(rot).as_euler('xyz', degrees=True)

        print("\n─── Estimated Pose ───")
        print(f"Position → x: {trans[0]:.2f}, y: {trans[1]:.2f}, z: {trans[2]:.2f}")
        print(f"Orientation → Roll: {euler[0]:.2f}, Pitch: {euler[1]:.2f}, Yaw: {euler[2]:.2f}")
        print(f"ICP Fitness: {result.fitness:.4f}, RMSE: {result.inlier_rmse:.4f}")

        if gt_matrix is not None:
            gt_trans = gt_matrix[:3, 3]
            gt_rot = gt_matrix[:3, :3]
            gt_euler = R.from_matrix(gt_rot).as_euler('xyz', degrees=True)

            trans_error = np.linalg.norm(trans - gt_trans)
            rot_error = np.abs(euler - gt_euler)

            print("\n─── Ground Truth ───")
            print(f"GT Position → x: {gt_trans[0]:.2f}, y: {gt_trans[1]:.2f}, z: {gt_trans[2]:.2f}")
            print(f"GT Orientation → Roll: {gt_euler[0]:.2f}, Pitch: {gt_euler[1]:.2f}, Yaw: {gt_euler[2]:.2f}")

            print("\n─── Error Metrics ───")
            print(f"Translation Error: {trans_error:.3f} meters")
            print(f"Rotation Error → Roll: {rot_error[0]:.2f}, Pitch: {rot_error[1]:.2f}, Yaw: {rot_error[2]:.2f}")