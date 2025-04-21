import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

class ICPTester:
    def __init__(self, voxel_size=0.05, logger=None):
        self.voxel_size = voxel_size
        self.last_pcd = None
        self.current_transform = np.eye(4)
        self.logger = logger  # ROS2 logger

        self.icp_config = {
            "max_correspondence_distance": 1.0,
            "estimation_method": o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            "criteria": o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=50,
                relative_fitness=1e-6,
                relative_rmse=1e-6,
            ),
        }

    def preprocess(self, points):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=30, std_ratio=1.5)
        pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)

        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=50),
            fast_normal_computation=True
        )

        return pcd

    def register(self, points):
        if len(points) < 1000:
            if self.logger:
                self.logger.warn("[ICP] Not enough points to run.")
            else:
                print("[ICP] Not enough points to run.")
            return None

        current_pcd = self.preprocess(points)

        if self.last_pcd is None:
            self.last_pcd = current_pcd
            if self.logger:
                self.logger.info("[ICP] First scan stored.")
            else:
                print("[ICP] First scan stored.")

            return np.eye(4)

        result = o3d.pipelines.registration.registration_icp(
            self.last_pcd, current_pcd, **self.icp_config
            )


        self.last_pcd = current_pcd
        self.current_transform = self.current_transform @ result.transformation

        self._print_pose(result, self.current_transform)
        return self.current_transform

    def _print_pose(self, result, current_transform):
        trans = current_transform[:3, 3]
        rot = current_transform[:3, :3]
        euler = R.from_matrix(rot).as_euler('xyz', degrees=True)

        msg = (
            f"\n─── Estimated Pose ───\n"
            f"Position → x: {trans[0]:.2f}, y: {trans[1]:.2f}, z: {trans[2]:.2f}\n"
            f"Orientation → Roll: {euler[0]:.2f}, Pitch: {euler[1]:.2f}, Yaw: {euler[2]:.2f}\n"
            f"ICP Fitness: {result.fitness:.4f}, RMSE: {result.inlier_rmse:.4f}\n"
            f"[DEBUG] ΔTransform:\n{result.transformation}\n"
            f"[DEBUG] Accumulated Pose:\n{current_transform}"
        )

        if self.logger:
            self.logger.info(msg)
        else:
            print(msg)
