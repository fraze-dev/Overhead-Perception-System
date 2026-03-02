"""
World-Frame Calibration System for Overhead RealSense Camera
Author: Aaron Fraze
Date: February 3, 2026
Purpose: Transform camera coordinates to world coordinates for overhead tracking
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import json
from datetime import datetime
from pathlib import Path
from typing import Tuple, List, Dict, Optional


class WorldFrameCalibrator:
    """
    Handles extrinsic calibration and coordinate transformations
    from camera frame to world frame.
    """
    
    def __init__(self, output_dir="results/calibration"):
        """
        Initialize the calibrator.
        
        Args:
            output_dir: Directory to save calibration data
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Camera parameters (will be populated from camera)
        self.camera_intrinsics = None
        self.depth_scale = None
        
        # Extrinsic calibration parameters
        self.camera_height_cm = None  # Height above ground plane
        self.camera_position_world = None  # [x, y, z] in world frame
        self.rotation_matrix = None  # 3x3 rotation from camera to world
        self.translation_vector = None  # 3x1 translation from camera to world
        
        # Transformation matrix (4x4 homogeneous)
        self.T_world_camera = None
        
        # Calibration quality metrics
        self.calibration_error_cm = None
        self.calibration_points = []
        
        print("World Frame Calibrator initialized")
    
    def setup_camera(self, pipeline: rs.pipeline):
        """
        Extract camera intrinsic parameters from RealSense pipeline.
        
        Args:
            pipeline: Active RealSense pipeline
        """
        profile = pipeline.get_active_profile()
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        
        # Get depth stream intrinsics
        #depth_stream = profile.get_stream(rs.stream.depth)
        color_stream = profile.get_stream(rs.stream.color)
        #self.camera_intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
        self.camera_intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        #self.camera_intrinsics = None
        #print(F"Depth Scale: {self.depth_scale}")


        print(f"Camera intrinsics loaded:")
        print(f"  Resolution: {self.camera_intrinsics.width} x {self.camera_intrinsics.height}")
        print(f"  Focal length: fx={self.camera_intrinsics.fx:.2f}, fy={self.camera_intrinsics.fy:.2f}")
        print(f"  Principal point: ({self.camera_intrinsics.ppx:.2f}, {self.camera_intrinsics.ppy:.2f})")
        print(f"  Depth scale: {self.depth_scale} m/unit")
    
    def define_simple_overhead_calibration(
        self,
        camera_height_cm: float,
        camera_x_world: float = 0.0,
        camera_y_world: float = 0.0,
        camera_tilt_deg: float = 0.0,
        camera_pan_deg: float = 0.0,
        camera_roll_deg: float = 0.0
    ):
        """
        Define calibration for a camera looking straight down (or nearly so).
        
        This is the simplest case: camera is overhead, pointing down at the workspace.
        
        Args:
            camera_height_cm: Height of camera above ground plane (Z coordinate)
            camera_x_world: X position of camera in world frame
            camera_y_world: Y position of camera in world frame
            camera_tilt_deg: Tilt angle (rotation about X-axis, pitch)
            camera_pan_deg: Pan angle (rotation about Y-axis, yaw)
            camera_roll_deg: Roll angle (rotation about Z-axis)
        """
        self.camera_height_cm = camera_height_cm
        self.camera_position_world = np.array([camera_x_world / 100, camera_y_world / 100, camera_height_cm / 100])
        
        # Build rotation matrix from Euler angles (ZYX convention)
        # Convert degrees to radians
        tilt = np.radians(camera_tilt_deg)
        pan = np.radians(camera_pan_deg)
        roll = np.radians(camera_roll_deg)
        
        # Rotation matrices
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(tilt), -np.sin(tilt)],
            [0, np.sin(tilt), np.cos(tilt)]
        ])
        
        Ry = np.array([
            [np.cos(pan), 0, np.sin(pan)],
            [0, 1, 0],
            [-np.sin(pan), 0, np.cos(pan)]
        ])
        
        Rz = np.array([
            [np.cos(roll), -np.sin(roll), 0],
            [np.sin(roll), np.cos(roll), 0],
            [0, 0, 1]
        ])
        
        # Combined rotation: Rz * Ry * Rx
        self.rotation_matrix = Rz @ Ry @ Rx
        
        # For overhead camera looking down, we need to rotate camera frame to world frame
        # Camera Z points forward, we want it to point down (-Z in world)
        # This requires a 90-degree rotation
        R_camera_to_world = np.array([
            [1, 0, 0],
            [0, -1, 0],
            [0, 0, -1]
        ])
        
        # Combine with user-specified rotations
        self.rotation_matrix = self.rotation_matrix @ R_camera_to_world
        
        # Translation is just the camera position
        self.translation_vector = self.camera_position_world.reshape(3, 1)
        
        # Build 4x4 homogeneous transformation matrix
        self.T_world_camera = np.eye(4)
        self.T_world_camera[:3, :3] = self.rotation_matrix
        self.T_world_camera[:3, 3] = self.camera_position_world
        
        print(f"\nSimple overhead calibration set:")
        print(f"  Camera position (world): [{camera_x_world:.2f}, {camera_y_world:.2f}, {camera_height_cm:.2f}] cm")
        print(f"  Camera orientation: tilt={camera_tilt_deg:.1f}°, pan={camera_pan_deg:.1f}°, roll={camera_roll_deg:.1f}°")
        print(f"  Rotation matrix:")
        print(self.rotation_matrix)
    '''
    def pixel_to_camera_coordinates(
            self,
            u: float,
            v: float,
            depth_meters: float,
            depth_frame: rs.depth_frame = None  # Add this parameter
    ) -> np.ndarray:
        # Get intrinsics from frame if available
        if depth_frame:
            intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        elif self.camera_intrinsics is not None:
            intrinsics = self.camera_intrinsics
        else:
            raise ValueError("No intrinsics available")

        # Unproject using intrinsics
        point_camera = rs.rs2_deproject_pixel_to_point(
            intrinsics,
            [u, v],
            depth_meters
        )

        return np.array(point_camera)

    '''
    def pixel_to_camera_coordinates(
        self,
        u: float,
        v: float,
        depth_meters: float
    ) -> np.ndarray:
        """
        Convert pixel coordinates + depth to 3D point in camera frame.
        
        Uses pinhole camera model with intrinsic parameters.
        
        Args:
            u: Pixel x-coordinate
            v: Pixel y-coordinate
            depth_meters: Depth value in meters
            
        Returns:
            3D point [x, y, z] in camera frame (meters)
        """
        if self.camera_intrinsics is None:
            raise ValueError("Camera intrinsics not set. Call setup_camera() first.")
        
        # Unproject using intrinsics
        point_camera = rs.rs2_deproject_pixel_to_point(
            self.camera_intrinsics,
            [u, v],
            depth_meters
        )
        
        return np.array(point_camera)

    def camera_to_world_coordinates(
        self,
        point_camera: np.ndarray,
        return_cm: bool = True
    ) -> np.ndarray:
        """
        Transform point from camera frame to world frame.
        
        Args:
            point_camera: 3D point [x, y, z] in camera frame (meters)
            return_cm: If True, return in centimeters; else meters
            
        Returns:
            3D point [x, y, z] in world frame
        """
        if self.T_world_camera is None:
            raise ValueError("Calibration not set. Call define_simple_overhead_calibration() first.")
        
        # Convert to homogeneous coordinates
        point_camera_h = np.append(point_camera, 1)
        
        # Apply transformation
        point_world_h = self.T_world_camera @ point_camera_h
        
        # Convert back to 3D
        point_world = point_world_h[:3]
        
        # Convert to centimeters if requested
        if return_cm:
            point_world = point_world * 100
        
        return point_world
    '''

    def camera_to_world_coordinates(
            self,
            point_camera: np.ndarray,
            return_cm: bool = True
    ) -> np.ndarray:
        """
        Transform point from camera frame to world frame.

        Args:
            point_camera: 3D point [x, y, z] in camera frame (METERS)
            return_cm: If True, return in centimeters; else meters

        Returns:
            3D point [x, y, z] in world frame
        """
        if self.T_world_camera is None:
            raise ValueError("Calibration not set.")

        # Convert to centimeters FIRST (before transformation)
        point_camera_cm = point_camera * 100

        # Convert to homogeneous coordinates
        point_camera_h = np.append(point_camera_cm, 1)

        # Apply transformation (now everything is in cm)
        point_world_h = self.T_world_camera @ point_camera_h

        # Convert back to 3D
        point_world = point_world_h[:3]

        # Optionally convert back to meters
        if not return_cm:
            point_world = point_world / 100

        return point_world
    '''
    def depth_image_to_world_points(
        self,
        depth_frame: rs.depth_frame,
        subsample: int = 1,
        max_distance_cm: float = 300.0
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Convert entire depth image to world coordinates.
        
        Args:
            depth_frame: RealSense depth frame
            subsample: Sample every Nth pixel (1 = all pixels)
            max_distance_cm: Ignore points beyond this distance
            
        Returns:
            (points_world, colors) - Nx3 array of world points (cm) and Nx3 RGB colors
        """
        depth_image = np.asanyarray(depth_frame.get_data())
        height, width = depth_image.shape
        
        points_world = []
        
        for v in range(0, height, subsample):
            for u in range(0, width, subsample):
                depth_value = depth_image[v, u]
                
                if depth_value == 0:
                    continue
                
                depth_meters = depth_value * self.depth_scale
                depth_cm = depth_meters * 100
                
                if depth_cm > max_distance_cm:
                    continue
                
                # Convert to camera frame
                point_camera = self.pixel_to_camera_coordinates(u, v, depth_meters)
                
                # Convert to world frame
                point_world = self.camera_to_world_coordinates(point_camera, return_cm=True)
                
                points_world.append(point_world)
        
        points_world = np.array(points_world)
        
        return points_world
    
    def add_calibration_point(
        self,
        pixel_coords: Tuple[int, int],
        depth_meters: float,
        world_coords: Tuple[float, float, float],
        label: str = ""
    ):
        """
        Add a known calibration point for validation.
        
        Args:
            pixel_coords: (u, v) pixel coordinates in image
            depth_meters: Measured depth at this pixel
            world_coords: Known (x, y, z) world coordinates (cm)
            label: Optional label for this point
        """
        u, v = pixel_coords
        
        # Convert pixel + depth to camera frame
        point_camera = self.pixel_to_camera_coordinates(u, v, depth_meters)
        
        # Transform to world frame
        point_world_predicted = self.camera_to_world_coordinates(point_camera, return_cm=True)
        
        # Calculate error
        world_coords_array = np.array(world_coords)
        error = np.linalg.norm(point_world_predicted - world_coords_array)
        
        calibration_point = {
            'label': label,
            'pixel_coords': pixel_coords,
            'depth_meters': depth_meters,
            'world_coords_true': world_coords,
            'world_coords_predicted': point_world_predicted.tolist(),
            'error_cm': error
        }
        
        self.calibration_points.append(calibration_point)
        
        print(f"\nCalibration point '{label}':")
        print(f"  True world: {world_coords}")
        print(f"  Predicted:  [{point_world_predicted[0]:.2f}, {point_world_predicted[1]:.2f}, {point_world_predicted[2]:.2f}]")
        print(f"  Error: {error:.2f} cm")
        
        return error
    
    def validate_calibration(self) -> Dict:
        """
        Calculate calibration accuracy metrics from all calibration points.
        
        Returns:
            Dictionary with error statistics
        """
        if len(self.calibration_points) == 0:
            print("No calibration points added yet!")
            return None
        
        errors = [pt['error_cm'] for pt in self.calibration_points]
        
        metrics = {
            'num_points': len(errors),
            'mean_error_cm': np.mean(errors),
            'std_error_cm': np.std(errors),
            'max_error_cm': np.max(errors),
            'min_error_cm': np.min(errors),
            'rmse_cm': np.sqrt(np.mean(np.array(errors) ** 2))
        }
        
        self.calibration_error_cm = metrics['mean_error_cm']
        
        print(f"\n{'='*60}")
        print("CALIBRATION VALIDATION RESULTS")
        print(f"{'='*60}")
        print(f"Number of test points: {metrics['num_points']}")
        print(f"Mean error:            {metrics['mean_error_cm']:.2f} ± {metrics['std_error_cm']:.2f} cm")
        print(f"RMSE:                  {metrics['rmse_cm']:.2f} cm")
        print(f"Min/Max error:         {metrics['min_error_cm']:.2f} / {metrics['max_error_cm']:.2f} cm")
        print(f"{'='*60}")
        
        return metrics
    
    def visualize_world_points_top_down(
        self,
        points_world: np.ndarray,
        calibration_points: bool = True,
        workspace_bounds: Optional[Tuple[float, float, float, float]] = None
    ) -> np.ndarray:
        """
        Create a top-down visualization of world coordinates.
        
        Args:
            points_world: Nx3 array of points in world frame (cm)
            calibration_points: If True, mark calibration points
            workspace_bounds: (x_min, x_max, y_min, y_max) in cm
            
        Returns:
            BGR image for display
        """
        if workspace_bounds is None:
            # Auto-compute bounds from points
            x_min, y_min = np.min(points_world[:, :2], axis=0)
            x_max, y_max = np.max(points_world[:, :2], axis=0)
            margin = 20  # cm
            workspace_bounds = (x_min - margin, x_max + margin, 
                              y_min - margin, y_max + margin)
        
        x_min, x_max, y_min, y_max = workspace_bounds
        
        # Create image
        img_width = 800
        img_height = int(img_width * (y_max - y_min) / (x_max - x_min))
        image = np.ones((img_height, img_width, 3), dtype=np.uint8) * 255
        
        # Helper function to convert world coords to image pixels
        def world_to_pixel(x, y):
            px = int((x - x_min) / (x_max - x_min) * img_width)
            py = int((y - y_min) / (y_max - y_min) * img_height)
            py = img_height - py  # Flip Y axis
            return px, py
        
        # Draw grid
        grid_spacing_cm = 50  # 50 cm grid
        for x in np.arange(x_min, x_max, grid_spacing_cm):
            px, py_top = world_to_pixel(x, y_max)
            px, py_bottom = world_to_pixel(x, y_min)
            cv2.line(image, (px, py_top), (px, py_bottom), (200, 200, 200), 1)
        
        for y in np.arange(y_min, y_max, grid_spacing_cm):
            px_left, py = world_to_pixel(x_min, y)
            px_right, py = world_to_pixel(x_max, y)
            cv2.line(image, (px_left, py), (px_right, py), (200, 200, 200), 1)
        
        # Draw origin
        origin_px, origin_py = world_to_pixel(0, 0)
        cv2.circle(image, (origin_px, origin_py), 5, (0, 0, 255), -1)
        cv2.putText(image, "Origin", (origin_px + 10, origin_py), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        # Draw points (subsample if too many)
        max_points = 5000
        if len(points_world) > max_points:
            indices = np.random.choice(len(points_world), max_points, replace=False)
            points_to_draw = points_world[indices]
        else:
            points_to_draw = points_world
        
        for pt in points_to_draw:
            px, py = world_to_pixel(pt[0], pt[1])
            if 0 <= px < img_width and 0 <= py < img_height:
                cv2.circle(image, (px, py), 1, (100, 100, 100), -1)
        
        # Draw calibration points
        if calibration_points and len(self.calibration_points) > 0:
            for cal_pt in self.calibration_points:
                # True position
                x_true, y_true, z_true = cal_pt['world_coords_true']
                px_true, py_true = world_to_pixel(x_true, y_true)
                cv2.circle(image, (px_true, py_true), 8, (0, 255, 0), 2)
                
                # Predicted position
                x_pred, y_pred, z_pred = cal_pt['world_coords_predicted']
                px_pred, py_pred = world_to_pixel(x_pred, y_pred)
                cv2.circle(image, (px_pred, py_pred), 8, (0, 0, 255), 2)
                
                # Draw line between them
                cv2.line(image, (px_true, py_true), (px_pred, py_pred), (255, 0, 0), 2)
                
                # Label
                label = cal_pt['label'] or f"Point {len(self.calibration_points)}"
                cv2.putText(image, label, (px_true + 10, py_true - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
        
        # Add legend
        legend_y = 30
        cv2.putText(image, "World Frame Top-Down View", (10, legend_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        legend_y += 30
        cv2.circle(image, (20, legend_y), 5, (0, 0, 255), -1)
        cv2.putText(image, "Origin", (40, legend_y + 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        if calibration_points and len(self.calibration_points) > 0:
            legend_y += 25
            cv2.circle(image, (20, legend_y), 8, (0, 255, 0), 2)
            cv2.putText(image, "True cal point", (40, legend_y + 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            legend_y += 25
            cv2.circle(image, (20, legend_y), 8, (0, 0, 255), 2)
            cv2.putText(image, "Predicted cal point", (40, legend_y + 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        return image
    
    def save_calibration(self, filename: str = "calibration.json"):
        """
        Save calibration parameters to file.
        
        Args:
            filename: Name of calibration file
        """
        calibration_data = {
            'timestamp': datetime.now().isoformat(),
            'camera_height_cm': self.camera_height_cm,
            'camera_position_world': self.camera_position_world.tolist(),
            'rotation_matrix': self.rotation_matrix.tolist(),
            'translation_vector': self.translation_vector.tolist(),
            'transformation_matrix': self.T_world_camera.tolist(),
            'calibration_error_cm': self.calibration_error_cm,
            'calibration_points': self.calibration_points,
            'camera_intrinsics': {
                'width': self.camera_intrinsics.width,
                'height': self.camera_intrinsics.height,
                'fx': self.camera_intrinsics.fx,
                'fy': self.camera_intrinsics.fy,
                'ppx': self.camera_intrinsics.ppx,
                'ppy': self.camera_intrinsics.ppy
            } if self.camera_intrinsics else None,
            'depth_scale': self.depth_scale
        }
        
        filepath = self.output_dir / filename
        with open(filepath, 'w') as f:
            json.dump(calibration_data, f, indent=2)
        
        print(f"\n✓ Calibration saved to: {filepath}")
    
    def load_calibration(self, filename: str = "calibration.json"):
        """
        Load calibration parameters from file.
        
        Args:
            filename: Name of calibration file
        """
        filepath = self.output_dir / filename
        
        with open(filepath, 'r') as f:
            calibration_data = json.load(f)
        
        self.camera_height_cm = calibration_data['camera_height_cm']
        self.camera_position_world = np.array(calibration_data['camera_position_world'])
        self.rotation_matrix = np.array(calibration_data['rotation_matrix'])
        self.translation_vector = np.array(calibration_data['translation_vector'])
        self.T_world_camera = np.array(calibration_data['transformation_matrix'])
        self.calibration_error_cm = calibration_data['calibration_error_cm']
        self.calibration_points = calibration_data['calibration_points']
        self.depth_scale = calibration_data['depth_scale']
        
        print(f"\n✓ Calibration loaded from: {filepath}")
        print(f"  Camera height: {self.camera_height_cm} cm")
        print(f"  Calibration error: {self.calibration_error_cm:.2f} cm")


def interactive_calibration_session():
    """
    Run an interactive calibration session with live camera feed.
    """
    print("="*60)
    print("INTERACTIVE WORLD-FRAME CALIBRATION")
    print("="*60)
    print()
    
    # Initialize camera
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    
    print("Starting camera...")
    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)
    # Warm up
    print("Warming up camera...")
    for _ in range(30):
        pipeline.wait_for_frames()

    # Initialize calibrator
    calibrator = WorldFrameCalibrator()
    calibrator.setup_camera(pipeline)
    
    print("\n" + "="*60)
    print("STEP 1: DEFINE WORLD FRAME")
    print("="*60)
    print("Define your world coordinate system:")
    print("  - Where is the origin? (e.g., corner of arena)")
    print("  - X-axis direction? (e.g., toward goal)")
    print("  - Y-axis direction? (e.g., across width)")
    print("  - Z-axis points up from ground")
    print()
    
    camera_height = float(input("Camera height above ground (cm): "))
    camera_x = float(input("Camera X position in world frame (cm, default=0): ") or "0")
    camera_y = float(input("Camera Y position in world frame (cm, default=0): ") or "0")
    
    print("\nCamera orientation:")
    tilt = float(input("  Tilt angle in degrees (0=straight down, default=0): ") or "0")
    pan = float(input("  Pan angle in degrees (default=0): ") or "0")
    roll = float(input("  Roll angle in degrees (default=0): ") or "0")
    
    calibrator.define_simple_overhead_calibration(
        camera_height_cm=camera_height,
        camera_x_world=camera_x,
        camera_y_world=camera_y,
        camera_tilt_deg=tilt,
        camera_pan_deg=pan,
        camera_roll_deg=roll
    )
    
    print("\n" + "="*60)
    print("STEP 2: ADD CALIBRATION POINTS")
    print("="*60)
    print("Place objects at known world coordinates to validate calibration.")
    print("You'll click on them in the live feed and enter their true positions.")
    print()
    
    # Live calibration point collection
    cv2.namedWindow('Calibration - Click on known points', cv2.WINDOW_NORMAL)
    
    # Mouse callback for point selection
    selected_point = [None]
    
    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            selected_point[0] = (x, y)
    
    cv2.setMouseCallback('Calibration - Click on known points', mouse_callback)
    
    num_points = int(input("How many calibration points do you want to add? (recommended: 4-6): "))
    
    for i in range(num_points):
        print(f"\n--- Calibration Point {i+1}/{num_points} ---")
        print("Click on the object in the video feed, then enter its known world coordinates.")
        
        selected_point[0] = None
        
        # Show live feed until point is selected
        while selected_point[0] is None:
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                continue
            
            color_image = np.asanyarray(color_frame.get_data())
            
            # Draw crosshair at mouse position
            cv2.putText(color_image, f"Click on calibration point {i+1}/{num_points}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(color_image, "Press 'q' to skip this point", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            
            cv2.imshow('Calibration - Click on known points', color_image)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("Skipping this point...")
                break
        
        if selected_point[0] is None:
            continue
        
        # Get depth at selected point
        u, v = selected_point[0]
        depth_value = depth_frame.get_distance(u, v)
        
        if depth_value == 0:
            print("No depth data at selected point! Try again.")
            i -= 1
            continue
        
        print(f"Selected pixel: ({u}, {v}), Depth: {depth_value:.3f} m")
        
        # Get true world coordinates from user
        label = input(f"Label for this point (e.g., 'corner', 'center'): ")
        x_true = float(input("  True X coordinate (cm): "))
        y_true = float(input("  True Y coordinate (cm): "))
        z_true = float(input("  True Z coordinate (cm, usually 0 for ground): ") or "0")
        
        # Add calibration point
        calibrator.add_calibration_point(
            pixel_coords=(u, v),
            depth_meters=depth_value,
            world_coords=(x_true, y_true, z_true),
            label=label
        )
    
    cv2.destroyAllWindows()
    
    # Validate calibration
    print("\n" + "="*60)
    print("STEP 3: VALIDATE CALIBRATION")
    print("="*60)
    
    validation_metrics = calibrator.validate_calibration()
    
    # Visualize
    print("\nGenerating visualization...")
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    
    points_world = calibrator.depth_image_to_world_points(depth_frame, subsample=5)
    
    viz_image = calibrator.visualize_world_points_top_down(points_world, calibration_points=True)
    
    cv2.imshow('World Frame Top-Down View', viz_image)
    cv2.imwrite(str(calibrator.output_dir / 'calibration_visualization.png'), viz_image)
    print(f"✓ Visualization saved to: {calibrator.output_dir / 'calibration_visualization.png'}")
    
    print("\nPress any key to close visualization...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    # Save calibration
    print("\n" + "="*60)
    print("STEP 4: SAVE CALIBRATION")
    print("="*60)
    
    save_choice = input("Save this calibration? (y/n): ").lower()
    if save_choice == 'y':
        calibrator.save_calibration("calibration.json")
        print("\n✓ Calibration complete and saved!")
    else:
        print("Calibration not saved.")
    
    # Cleanup
    pipeline.stop()
    print("\nCamera stopped. Calibration session complete!")


if __name__ == "__main__":
    interactive_calibration_session()
