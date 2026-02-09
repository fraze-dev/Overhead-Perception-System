"""
Interactive Calibration Clicking Tool
Author: Aaron Fraze
Date: January 31, 2026

Interactive tool to click points in the camera view and see their world coordinates.
Used for validating the camera-to-world transformation.
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import json
from datetime import datetime
from pathlib import Path
from coordinate_transform import CoordinateTransformer, format_coordinates


class CalibrationClickTool:
    """
    Interactive tool for clicking points and measuring world coordinates.
    """
    
    def __init__(self, camera_height_m=2.21, pitch_deg=0, roll_deg=0, yaw_deg=0,
                 output_dir="results/calibration"):
        """
        Initialize the calibration clicking tool.
        
        Args:
            camera_height_m: Height of camera above floor
            pitch_deg: Camera pitch angle (forward/backward tilt)
            roll_deg: Camera roll angle (left/right rotation)
            yaw_deg: Camera yaw angle (side-to-side rotation)
            output_dir: Directory to save calibration data
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Initialize coordinate transformer
        self.transformer = CoordinateTransformer(
            camera_height_m=camera_height_m,
            pitch_deg=pitch_deg,
            roll_deg=roll_deg,
            yaw_deg=yaw_deg
        )
        
        # Initialize RealSense
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Use 640x480 as decided
        self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
        
        print("Starting RealSense camera...")
        self.profile = self.pipeline.start(self.config)
        
        # Get depth sensor and scale
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        
        # Get intrinsics and pass to transformer
        depth_stream = self.profile.get_stream(rs.stream.depth)
        self.intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
        self.transformer.set_intrinsics(self.intrinsics)
        
        # Align depth to color
        self.align = rs.align(rs.stream.color)
        
        # Warm up camera
        print("Warming up camera...")
        for _ in range(30):
            self.pipeline.wait_for_frames()
        
        # Storage for clicked points
        self.clicked_points = []
        
        # Current frame storage
        self.current_color = None
        self.current_depth = None
        
        print("\n" + "="*60)
        print("Calibration Click Tool Ready!")
        print("="*60)
    
    def _mouse_callback(self, event, x, y, flags, param):
        """Handle mouse clicks on the image."""
        if event == cv2.EVENT_LBUTTONDOWN:
            self._process_click(x, y)
    
    def _process_click(self, pixel_x, pixel_y):
        """
        Process a mouse click and calculate world coordinates.
        
        Args:
            pixel_x: X coordinate of click
            pixel_y: Y coordinate of click
        """
        if self.current_depth is None:
            print("No depth frame available!")
            return
        
        # Get depth at clicked pixel
        depth_value = self.current_depth[pixel_y, pixel_x]
        depth_m = depth_value * self.depth_scale
        
        # Check if depth is valid
        if depth_m == 0 or depth_m > 10:  # No reading or too far
            print(f"\n⚠️  Invalid depth at pixel ({pixel_x}, {pixel_y}): {depth_m:.3f} m")
            return
        
        # Transform to world coordinates
        result = self.transformer.pixel_to_world_coords(pixel_x, pixel_y, depth_m)
        
        # Print results
        print(format_coordinates(result))
        
        # Store the point
        point_data = {
            'pixel_x': int(pixel_x),
            'pixel_y': int(pixel_y),
            'depth_m': float(depth_m),
            'camera_coords_m': result['camera_coords'].tolist(),
            'world_coords_m': result['world_coords'].tolist(),
            'world_xy_cm': [float(result['world_coords_2d'][0] * 100),
                           float(result['world_coords_2d'][1] * 100)],
            'timestamp': datetime.now().isoformat()
        }
        
        self.clicked_points.append(point_data)
        
        # Draw a marker on the image
        cv2.circle(self.current_color, (pixel_x, pixel_y), 5, (0, 255, 0), -1)
        cv2.circle(self.current_color, (pixel_x, pixel_y), 10, (0, 255, 0), 2)
        
        # Add text with world coordinates
        world_x_cm = result['world_coords_2d'][0] * 100
        world_y_cm = result['world_coords_2d'][1] * 100
        text = f"({world_x_cm:+.1f}, {world_y_cm:+.1f}) cm"
        cv2.putText(self.current_color, text, (pixel_x + 15, pixel_y - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    def run(self):
        """
        Main loop - display camera feed and handle clicks.
        """
        window_name = "Calibration Tool - Click on test points (Q to quit, S to save)"
        cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, self._mouse_callback)
        
        print("\nInstructions:")
        print("  - Click on blue tape markers to measure their position")
        print("  - Press 'S' to save all clicked points")
        print("  - Press 'C' to clear all markers from display")
        print("  - Press 'Q' to quit")
        print("\nWaiting for clicks...\n")
        
        try:
            while True:
                # Get frames
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()
                
                if not color_frame or not depth_frame:
                    continue
                
                # Convert to numpy arrays
                self.current_color = np.asanyarray(color_frame.get_data()).copy()
                self.current_depth = np.asanyarray(depth_frame.get_data())
                
                # Add info overlay
                self._add_info_overlay()
                
                # Display
                cv2.imshow(window_name, self.current_color)
                
                # Handle keypresses
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q') or key == ord('Q'):
                    break
                elif key == ord('s') or key == ord('S'):
                    self._save_clicked_points()
                elif key == ord('c') or key == ord('C'):
                    print("Display cleared (data not deleted)")
        
        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
        
        finally:
            cv2.destroyAllWindows()
            self.pipeline.stop()
            print("\nCamera stopped.")
    
    def _add_info_overlay(self):
        """Add informational text overlay to the image."""
        overlay_bg = self.current_color.copy()
        
        # Semi-transparent background for text
        cv2.rectangle(overlay_bg, (5, 5), (635, 80), (0, 0, 0), -1)
        cv2.addWeighted(overlay_bg, 0.3, self.current_color, 0.7, 0, self.current_color)
        
        # Text
        cv2.putText(self.current_color, "Click on test points to measure position",
                   (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(self.current_color, f"Camera: {self.transformer.camera_height:.2f}m | "
                   f"Tilt: {self.transformer.pitch_deg:.1f}deg | "
                   f"Points: {len(self.clicked_points)}",
                   (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(self.current_color, "Press: Q=Quit  S=Save  C=Clear",
                   (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    def _save_clicked_points(self):
        """Save all clicked points to JSON file."""
        if len(self.clicked_points) == 0:
            print("No points to save!")
            return
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{timestamp}_calibration_points.json"
        filepath = self.output_dir / filename
        
        data = {
            'camera_height_m': self.transformer.camera_height,
            'pitch_deg': self.transformer.pitch_deg,
            'roll_deg': self.transformer.roll_deg,
            'yaw_deg': self.transformer.yaw_deg,
            'resolution': f"{self.intrinsics.width}x{self.intrinsics.height}",
            'num_points': len(self.clicked_points),
            'points': self.clicked_points,
            'timestamp': datetime.now().isoformat()
        }
        
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"\n✓ Saved {len(self.clicked_points)} points to: {filepath}")
    
    def add_ground_truth(self, point_index, physical_x_cm, physical_y_cm):
        """
        Add ground truth physical measurement to a clicked point.
        
        Args:
            point_index: Index of the point in self.clicked_points
            physical_x_cm: Physically measured X coordinate (cm)
            physical_y_cm: Physically measured Y coordinate (cm)
        """
        if point_index >= len(self.clicked_points):
            print(f"Point index {point_index} out of range!")
            return
        
        point = self.clicked_points[point_index]
        point['physical_x_cm'] = float(physical_x_cm)
        point['physical_y_cm'] = float(physical_y_cm)
        
        # Calculate error
        measured_x = point['world_xy_cm'][0]
        measured_y = point['world_xy_cm'][1]
        error_x = measured_x - physical_x_cm
        error_y = measured_y - physical_y_cm
        error_total = np.sqrt(error_x**2 + error_y**2)
        
        point['error_x_cm'] = float(error_x)
        point['error_y_cm'] = float(error_y)
        point['error_total_cm'] = float(error_total)
        
        print(f"\nPoint {point_index} ground truth added:")
        print(f"  Physical:  ({physical_x_cm:+.1f}, {physical_y_cm:+.1f}) cm")
        print(f"  Measured:  ({measured_x:+.1f}, {measured_y:+.1f}) cm")
        print(f"  Error:     ({error_x:+.1f}, {error_y:+.1f}) cm | Total: {error_total:.2f} cm")


def main():
    """Main entry point for the calibration tool."""
    print("="*60)
    print("CALIBRATION CLICKING TOOL")
    print("="*60)
    print()
    
    # Get camera parameters from user
    print("Enter camera mounting parameters:")
    print("(Press ENTER to use defaults from spatial test)")
    print()
    
    height_input = input("Camera height (meters) [2.21]: ").strip()
    camera_height = float(height_input) if height_input else 2.21
    
    pitch_input = input("Pitch angle (degrees, forward tilt) [3.0]: ").strip()
    pitch_deg = float(pitch_input) if pitch_input else 3.0
    
    roll_input = input("Roll angle (degrees, left/right) [0.0]: ").strip()
    roll_deg = float(roll_input) if roll_input else 0.0
    
    yaw_input = input("Yaw angle (degrees, rotation) [0.0]: ").strip()
    yaw_deg = float(yaw_input) if yaw_input else 0.0
    
    print("\n" + "="*60)
    
    # Initialize tool
    tool = CalibrationClickTool(
        camera_height_m=camera_height,
        pitch_deg=pitch_deg,
        roll_deg=roll_deg,
        yaw_deg=yaw_deg
    )
    
    # Run interactive session
    tool.run()
    
    # Option to add ground truth measurements
    if len(tool.clicked_points) > 0:
        print("\n" + "="*60)
        print("Add ground truth measurements?")
        print("="*60)
        add_gt = input("Do you want to add physical measurements? (y/n): ").strip().lower()
        
        if add_gt == 'y':
            for i, point in enumerate(tool.clicked_points):
                print(f"\nPoint {i}: Camera measured ({point['world_xy_cm'][0]:+.1f}, {point['world_xy_cm'][1]:+.1f}) cm")
                try:
                    phys_x = float(input(f"  Physical X (cm): ").strip())
                    phys_y = float(input(f"  Physical Y (cm): ").strip())
                    tool.add_ground_truth(i, phys_x, phys_y)
                except ValueError:
                    print("  Skipping point (invalid input)")
            
            # Save with ground truth
            tool._save_clicked_points()
            
            # Print summary statistics
            errors = [p['error_total_cm'] for p in tool.clicked_points if 'error_total_cm' in p]
            if errors:
                print("\n" + "="*60)
                print("CALIBRATION ACCURACY SUMMARY")
                print("="*60)
                print(f"Number of points: {len(errors)}")
                print(f"Mean error: {np.mean(errors):.2f} cm")
                print(f"Std dev: {np.std(errors):.2f} cm")
                print(f"Max error: {np.max(errors):.2f} cm")
                print(f"Min error: {np.min(errors):.2f} cm")
    
    print("\nCalibration session complete!")


if __name__ == "__main__":
    main()
