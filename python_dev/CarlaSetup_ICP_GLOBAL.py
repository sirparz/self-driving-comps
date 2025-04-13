import cv2
import numpy as np
import open3d as o3d
import pygame

from scipy.spatial.transform import Rotation as R
import random
import sys

# ─── CARLA ────────────────────────────────────────────────────────────────────────────────
CARLA_WHL_PATH = (
    "C:/Users/hamad/OneDrive/Desktop/ROBOTICS GA/Carla-0.10.0-Win64-Shipping/"
    "PythonAPI/carla/dist/carla-0.10.0-cp38-cp38-win_amd64.whl"
)
sys.path.append(CARLA_WHL_PATH)
import carla
# Constants
SYNC_DT = 0.05  # seconds
LIDAR_CONFIG = {
    "range": "100",
    "rotation_frequency": "10",
    "channels": "64",
    "upper_fov": "10",
    "lower_fov": "-30",
    "points_per_second": "100000",
}
CAMERA_CONFIG = {
    "image_size_x": "800",
    "image_size_y": "600",
    "fov": "90",
}

class ICPTester:
    def __init__(self, voxel_size=0.08):
        """Simple ICP testing class that just prints odometry values"""
        self.voxel_size = voxel_size
        self.last_pcd = None
        self.current_transform = np.eye(4)
        
        # Initialize Open3D visualization (optional)
        self.pcd = o3d.geometry.PointCloud()
        
        # ICP configuration
        self.icp_config = {
            "max_correspondence_distance": 0.5,
            "estimation_method": o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            "criteria": o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=50,
                relative_fitness=1e-6,
                relative_rmse=1e-6
            )
        }

    def preprocess_pointcloud(self, points):
        """Convert numpy array to Open3D point cloud with preprocessing"""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        # Remove statistical outliers
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=30, std_ratio=1.5)
        # Downsample
        pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)
        
        # Estimate normals for point-to-plane ICP
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=50),
            fast_normal_computation=True  # Enable approximate but faster normals
            )
        
        return pcd

    def update_and_visualize(self, new_points, gt_matrix=None):
        """Process new LiDAR scan and print ICP results"""
        if len(new_points) < 1000:
            print("Not enough points for ICP")
            return None
            
        current_pcd = self.preprocess_pointcloud(new_points)
        
        # First frame case
        if self.last_pcd is None:
            self.last_pcd = current_pcd
            print("Initialized first point cloud")
            return np.eye(4)
        
        # Run ICP
        reg_result = o3d.pipelines.registration.registration_icp(
            current_pcd, self.last_pcd, **self.icp_config
        )
        
        # Update state
        self.last_pcd = current_pcd
        self.transform = np.copy(reg_result.transformation)
        self.current_transform = self.current_transform @ self.transform 
        
        # Print the results
        self._print_transform(self.current_transform, reg_result, gt_matrix)
        
        return self.transform

    def _print_transform(self, transform, reg_result, gt_matrix=None):
        """Print the ICP result and optional ground truth comparison"""
        rotation_matrix = transform[:3, :3]
        translation = transform[:3, 3]

        try:
            estimated_euler = R.from_matrix(rotation_matrix).as_euler('xyz', degrees=True)
        except ValueError:
            estimated_euler = np.zeros(3)
            print("Invalid rotation matrix, setting to zeros")

        print("\n─── ICP Results ───")
        print(f"Translation (meters): X: {translation[0]:.3f}, Y: {translation[1]:.3f}, Z: {translation[2]:.3f}")
        print(f"Rotation (degrees): Roll: {estimated_euler[0]:.2f}, Pitch: {estimated_euler[1]:.2f}, Yaw: {estimated_euler[2]:.2f}")
        print(f"Fitness: {reg_result.fitness:.4f}, RMSE: {reg_result.inlier_rmse:.4f}")

        if gt_matrix is not None:
            gt_translation = gt_matrix[:3, 3]
            gt_euler = R.from_matrix(gt_matrix[:3, :3]).as_euler('xyz', degrees=True)

            translation_error = np.linalg.norm(translation - gt_translation)
            rotation_error = np.abs(estimated_euler - gt_euler)

            print("\n─── Ground Truth Pose ───")
            print(f"GT Translation (meters): X: {gt_translation[0]:.3f}, Y: {gt_translation[1]:.3f}, Z: {gt_translation[2]:.3f}")
            print(f"GT Rotation (degrees): Roll: {gt_euler[0]:.2f}, Pitch: {gt_euler[1]:.2f}, Yaw: {gt_euler[2]:.2f}")

            print("\n─── Error Metrics ───")
            print(f"Translation Error (meters): {translation_error:.4f}")
            print(f"Rotation Error (degrees): Roll: {rotation_error[0]:.2f}, Pitch: {rotation_error[1]:.2f}, Yaw: {rotation_error[2]:.2f}")



class CarlaICPDemo:
    def __init__(self):
        pygame.init()
        self.display = pygame.display.set_mode((800, 600))
        self.clock = pygame.time.Clock()
        
        # Connect to CARLA
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        
        # Setup ICP tester
        self.icp_tester = ICPTester(voxel_size=0.2)
        
        
        # Spawn vehicle and sensors
        self._spawn_vehicle_and_sensors()
    # transform to a 4x4 matrix for Open3D
    @staticmethod
    def carla_transform_to_matrix(transform):
        # Extract translation
        x = transform.location.x
        y = transform.location.y
        z = transform.location.z

        # Extract rotation
        roll = np.radians(transform.rotation.roll)
        pitch = np.radians(transform.rotation.pitch)
        yaw = np.radians(transform.rotation.yaw)

        # Create rotation matrix
        rot_matrix = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()

        # Assemble into 4x4 matrix
        matrix = np.eye(4)
        matrix[:3, :3] = rot_matrix
        matrix[:3, 3] = [x, y, z]

        return matrix


    def _spawn_vehicle_and_sensors(self):
        """Spawn a vehicle and LiDAR sensor"""
        # Spawn vehicle
        blueprint_lib = self.world.get_blueprint_library()
        vehicle_bp = blueprint_lib.find('vehicle.nissan.patrol')
        vehicle_bp.set_attribute('role_name', 'ego_vehicle')

        spawn_point = random.choice(self.world.get_map().get_spawn_points())
        self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
        self.vehicle.set_autopilot(True)

        # set inital transform
        initial_carla_tf = self.vehicle.get_transform()
        initial_matrix = self.carla_transform_to_matrix(initial_carla_tf)
        self.icp_tester.current_transform = initial_matrix
        
        # Spawn LiDAR
        lidar_bp = blueprint_lib.find('sensor.lidar.ray_cast')
        for key, value in LIDAR_CONFIG.items():
            lidar_bp.set_attribute(key, value)
        
        lidar_transform = carla.Transform(carla.Location(x=0, y=0, z=2.5))
        self.lidar = self.world.spawn_actor(
            lidar_bp, lidar_transform, attach_to=self.vehicle
        )
        self.lidar.listen(self._lidar_callback)
        # Spawn RGB camera
        camera_bp = blueprint_lib.find('sensor.camera.rgb')
        for key, value in CAMERA_CONFIG.items():
            camera_bp.set_attribute(key, value)
        camera_transform = carla.Transform(carla.Location(x=3.0, y=0.0, z=2.0), carla.Rotation(pitch=-15.0))
        self.camera = self.world.spawn_actor(
            camera_bp, camera_transform, attach_to=self.vehicle
        )        
        self.camera.listen(self._camera_callback)

        
    def _camera_callback(self, image: carla.Image):
        """Render RGB camera feed to pygame window."""
        array = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
        rgb = cv2.cvtColor(array[:, :, :3], cv2.COLOR_BGR2RGB)
        surface = pygame.surfarray.make_surface(rgb.swapaxes(0, 1))
        self.display.blit(surface, (0, 0))
        pygame.display.flip()



    def _lidar_callback(self, lidar_data):
        """Process LiDAR data and run ICP"""
        # Convert to numpy array
        points = np.frombuffer(lidar_data.raw_data, dtype=np.float32)
        points = np.reshape(points, (int(points.shape[0] / 4), 4))[:, :3]

        # Get ground truth transform matrix from CARLA
        gt_transform = self.vehicle.get_transform()
        gt_matrix = self.carla_transform_to_matrix(gt_transform)

        # Run ICP and print results (including ground truth)
        self.icp_tester.update_and_visualize(points, gt_matrix=gt_matrix)



    def run(self):
        """Main simulation loop"""
        try:
            while True:
                self.clock.tick(60)
                self.world.tick()
                
                # Check for quit event
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        return
                    
        finally:
            self._cleanup()

    def _cleanup(self):
        """Cleanup CARLA actors"""
        if hasattr(self, 'lidar') and self.lidar.is_alive:
            self.lidar.destroy()
        if hasattr(self, 'vehicle') and self.vehicle.is_alive:
            self.vehicle.destroy()
        pygame.quit()

if __name__ == "__main__":
    demo = CarlaICPDemo()
    demo.run()