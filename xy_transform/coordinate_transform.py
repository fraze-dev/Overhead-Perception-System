"""
Coordinate Transformation Module
Author: Aaron Fraze
Date: January 31, 2026

This module handles transformations from camera coordinates to world coordinates
for the overhead perception system.

Coordinate Systems:
- Camera Frame: Origin at camera optical center, Z points forward (down), X right, Y down
- World Frame: Origin on floor directly below camera, X forward, Y left, Z up (optional)
"""

import numpy as np
import pyrealsense2 as rs


class CoordinateTransformer:
    """
    Handles transformation from camera pixel coordinates to world coordinates.
    
    The transformation pipeline:
    1. Pixel (u,v) + depth -> 3D camera coordinates using intrinsics
    2. 3D camera coordinates -> 3D world coordinates using extrinsics
    """
    
    def __init__(self, camera_height_m, pitch_deg=0, roll_deg=0, yaw_deg=0):
        """
        Initialize the coordinate transformer.
        
        Args:
            camera_height_m: Height of camera above floor (meters)
            pitch_deg: Camera pitch angle (rotation around X-axis, degrees)
                      Positive = tilted forward (looking more toward bottom of frame)
            roll_deg: Camera roll angle (rotation around Z-axis, degrees)
                     Positive = rotated clockwise when looking down
            yaw_deg: Camera yaw angle (rotation around Y-axis, degrees)
                    Positive = rotated to the right
        """
        self.camera_height = camera_height_m
        self.pitch_deg = pitch_deg
        self.roll_deg = roll_deg
        self.yaw_deg = yaw_deg
        
        # Camera intrinsics (will be set when we get the actual camera)
        self.intrinsics = None
        
        # Build rotation matrix from camera to world frame
        self._build_rotation_matrix()
        
        print(f"CoordinateTransformer initialized:")
        print(f"  Camera height: {camera_height_m:.3f} m")
        print(f"  Pitch: {pitch_deg:.2f}°, Roll: {roll_deg:.2f}°, Yaw: {yaw_deg:.2f}°")
    
    def set_intrinsics(self, intrinsics):
        """
        Set camera intrinsics from RealSense stream profile.
        
        Args:
            intrinsics: rs2.intrinsics object from depth stream
        """
        self.intrinsics = intrinsics
        print(f"\nCamera intrinsics set:")
        print(f"  Resolution: {intrinsics.width} x {intrinsics.height}")
        print(f"  Focal length: fx={intrinsics.fx:.2f}, fy={intrinsics.fy:.2f}")
        print(f"  Principal point: ppx={intrinsics.ppx:.2f}, ppy={intrinsics.ppy:.2f}")
    
    def _build_rotation_matrix(self):
        """
        Build the rotation matrix from camera frame to world frame.
        
        Convention:
        - Camera: Z forward (down), X right, Y down
        - World: X forward, Y left, Z up
        """
        # Convert angles to radians
        pitch = np.radians(self.pitch_deg)
        roll = np.radians(self.roll_deg)
        yaw = np.radians(self.yaw_deg)
        
        # Rotation around X-axis (pitch)
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(pitch), -np.sin(pitch)],
            [0, np.sin(pitch), np.cos(pitch)]
        ])
        
        # Rotation around Y-axis (yaw)
        Ry = np.array([
            [np.cos(yaw), 0, np.sin(yaw)],
            [0, 1, 0],
            [-np.sin(yaw), 0, np.cos(yaw)]
        ])
        
        # Rotation around Z-axis (roll)
        Rz = np.array([
            [np.cos(roll), -np.sin(roll), 0],
            [np.sin(roll), np.cos(roll), 0],
            [0, 0, 1]
        ])
        
        # Combined rotation: Rz * Ry * Rx (order matters!)
        self.R_cam_to_world = Rz @ Ry @ Rx
        
        # For a camera pointing straight down, we also need to account for
        # the camera's native frame orientation vs world frame
        # Camera Z points down (world Z points up), so we need a 180° rotation
        R_flip = np.array([
            [1, 0, 0],
            [0, -1, 0],
            [0, 0, -1]
        ])
        
        self.R_cam_to_world = self.R_cam_to_world @ R_flip
    
    def pixel_to_camera_coords(self, pixel_x, pixel_y, depth_m):
        """
        Convert pixel coordinates to 3D camera coordinates.
        
        Args:
            pixel_x: Pixel x coordinate (column)
            pixel_y: Pixel y coordinate (row)
            depth_m: Depth at that pixel (meters)
            
        Returns:
            np.array([x, y, z]) in camera frame (meters)
        """
        if self.intrinsics is None:
            raise ValueError("Camera intrinsics not set! Call set_intrinsics() first.")
        
        # Use RealSense SDK to deproject pixel to 3D point
        # This handles all the intrinsic camera parameters (focal length, distortion, etc.)
        point_3d = rs.rs2_deproject_pixel_to_point(
            self.intrinsics,
            [pixel_x, pixel_y],
            depth_m
        )
        
        return np.array(point_3d)
    
    def camera_to_world_coords(self, camera_coords):
        """
        Transform 3D camera coordinates to world coordinates.
        
        Args:
            camera_coords: np.array([x, y, z]) in camera frame (meters)
            
        Returns:
            np.array([x, y, z]) in world frame (meters)
        """
        # Apply rotation
        world_coords = self.R_cam_to_world @ camera_coords
        
        # Apply translation (camera height)
        # Camera is at height h, world origin is on floor
        world_coords[2] += self.camera_height
        
        return world_coords
    
    def pixel_to_world_coords(self, pixel_x, pixel_y, depth_m):
        """
        Full pipeline: pixel + depth -> world coordinates.
        
        Args:
            pixel_x: Pixel x coordinate (column)
            pixel_y: Pixel y coordinate (row)  
            depth_m: Depth at that pixel (meters)
            
        Returns:
            dict with:
                - camera_coords: (x, y, z) in camera frame
                - world_coords: (x, y, z) in world frame
                - world_coords_2d: (x, y) on floor plane
        """
        # Step 1: Pixel to 3D camera coordinates
        cam_coords = self.pixel_to_camera_coords(pixel_x, pixel_y, depth_m)
        
        # Step 2: Camera to world coordinates
        world_coords = self.camera_to_world_coords(cam_coords)
        
        return {
            'camera_coords': cam_coords,
            'world_coords': world_coords,
            'world_coords_2d': world_coords[:2],  # Just X, Y on floor
            'pixel': (pixel_x, pixel_y),
            'depth_m': depth_m
        }
    
    def update_tilt(self, pitch_deg=None, roll_deg=None, yaw_deg=None):
        """
        Update camera tilt angles and rebuild rotation matrix.
        
        Args:
            pitch_deg: New pitch angle (None = keep current)
            roll_deg: New roll angle (None = keep current)
            yaw_deg: New yaw angle (None = keep current)
        """
        if pitch_deg is not None:
            self.pitch_deg = pitch_deg
        if roll_deg is not None:
            self.roll_deg = roll_deg
        if yaw_deg is not None:
            self.yaw_deg = yaw_deg
        
        self._build_rotation_matrix()
        print(f"Tilt updated: Pitch={self.pitch_deg:.2f}°, Roll={self.roll_deg:.2f}°, Yaw={self.yaw_deg:.2f}°")


# Utility functions
def format_coordinates(coords_dict):
    """
    Format coordinate transformation results for display.
    
    Args:
        coords_dict: Dictionary from pixel_to_world_coords()
        
    Returns:
        Formatted string
    """
    cam = coords_dict['camera_coords']
    world = coords_dict['world_coords']
    world_2d = coords_dict['world_coords_2d']
    
    output = f"\n{'='*60}\n"
    output += f"Pixel: ({coords_dict['pixel'][0]:.0f}, {coords_dict['pixel'][1]:.0f})\n"
    output += f"Depth: {coords_dict['depth_m']:.3f} m ({coords_dict['depth_m']*100:.2f} cm)\n"
    output += f"{'-'*60}\n"
    output += f"Camera coords: ({cam[0]:+.4f}, {cam[1]:+.4f}, {cam[2]:+.4f}) m\n"
    output += f"World coords:  ({world[0]:+.4f}, {world[1]:+.4f}, {world[2]:+.4f}) m\n"
    output += f"{'-'*60}\n"
    output += f"Floor position (X, Y): ({world_2d[0]*100:+.2f}, {world_2d[1]*100:+.2f}) cm\n"
    output += f"{'='*60}\n"
    
    return output


if __name__ == "__main__":
    # Simple test
    print("Coordinate Transformer Test")
    print("="*60)
    
    # Initialize with camera at 2.21m, 3° forward tilt
    transformer = CoordinateTransformer(
        camera_height_m=2.21,
        pitch_deg=3.0,
        roll_deg=0.0,
        yaw_deg=0.0
    )
    
    print("\nTransformer ready for integration with RealSense camera.")
    print("Use with interactive clicking tool to test transformations.")
