"""
Week 3: Data Processing and Coordinate Systems
Author: Aaron Fraze
Date: January 27, 2026
Purpose: Implement depth filtering, point cloud generation, and coordinate system tools
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
from pathlib import Path
import json
from datetime import datetime


class RealSenseDataProcessor:
    """
    Comprehensive data processing for RealSense depth camera.
    Handles filtering, point clouds, and coordinate transformations.
    """
    
    def __init__(self, output_dir="results/week3_processing"):
        """Initialize the data processor."""
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Initialize RealSense
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Configure streams
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        
        # Start pipeline
        print("Starting RealSense camera...")
        self.profile = self.pipeline.start(self.config)
        
        # Get device and sensors
        self.device = self.profile.get_device()
        self.depth_sensor = self.device.first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        
        # Get intrinsics (will be populated after first frame)
        self.depth_intrinsics = None
        self.color_intrinsics = None
        
        # Create alignment object (align depth to color)
        self.align = rs.align(rs.stream.color)
        
        # Filtering options
        self.filters = self._setup_filters()
        
        print(f"Depth scale: {self.depth_scale} meters/unit")
        print("Camera initialized!\n")
        
        # Warm up
        print("Warming up camera (3 seconds)...")
        for _ in range(90):
            self.pipeline.wait_for_frames()
        print("Ready!\n")
    
    def _setup_filters(self):
        """
        Set up RealSense post-processing filters.
        
        Available filters:
        - Decimation: Reduces resolution to improve performance
        - Spatial: Edge-preserving smoothing
        - Temporal: Reduces noise over time
        - Hole Filling: Fills holes in depth map
        """
        filters = {
            'decimation': rs.decimation_filter(),
            'spatial': rs.spatial_filter(),
            'temporal': rs.temporal_filter(),
            'hole_filling': rs.hole_filling_filter()
        }
        
        # Configure spatial filter for edge-preserving smoothing
        filters['spatial'].set_option(rs.option.filter_magnitude, 2)
        filters['spatial'].set_option(rs.option.filter_smooth_alpha, 0.5)
        filters['spatial'].set_option(rs.option.filter_smooth_delta, 20)
        
        # Configure temporal filter
        filters['temporal'].set_option(rs.option.filter_smooth_alpha, 0.4)
        filters['temporal'].set_option(rs.option.filter_smooth_delta, 20)
        
        # Hole filling mode (0=fill_from_left, 1=farest_from_around, 2=nearest_from_around)
        filters['hole_filling'].set_option(rs.option.holes_fill, 1)
        
        return filters
    
    def get_frames(self, aligned=True, apply_filters=True):
        """
        Capture and process frames.
        
        Args:
            aligned: If True, align depth to color
            apply_filters: If True, apply post-processing filters
            
        Returns:
            dict with color_image, depth_image, depth_colormap
        """
        frames = self.pipeline.wait_for_frames()
        
        # Align if requested
        if aligned:
            frames = self.align.process(frames)
        
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            return None
        
        # Store intrinsics on first frame
        if self.depth_intrinsics is None:
            self.depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
            self.color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        
        # Apply filters
        if apply_filters:
            depth_frame = self._apply_filters(depth_frame)
        
        # Convert to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        # Create colormap for visualization
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), 
            cv2.COLORMAP_JET
        )
        
        return {
            'color': color_image,
            'depth': depth_image,
            'depth_colormap': depth_colormap,
            'depth_frame': depth_frame,  # Keep for point cloud
            'color_frame': color_frame
        }
    
    def _apply_filters(self, depth_frame):
        """Apply post-processing filters to depth frame."""
        # Apply in recommended order
        filtered = depth_frame
        # Skip decimation - it reduces resolution and causes size mismatch
        #filtered = self.filters['decimation'].process(filtered)
        filtered = self.filters['spatial'].process(filtered)
        filtered = self.filters['temporal'].process(filtered)
        filtered = self.filters['hole_filling'].process(filtered)
        return filtered
    
    def generate_point_cloud(self, depth_frame, color_frame, save_path=None):
        """
        Generate point cloud from depth and color frames.
        
        Args:
            depth_frame: RealSense depth frame
            color_frame: RealSense color frame
            save_path: Optional path to save point cloud (.ply)
            
        Returns:
            Open3D point cloud object
        """
        # Create point cloud using RealSense
        pc = rs.pointcloud()
        pc.map_to(color_frame)
        points = pc.calculate(depth_frame)
        
        # Get vertices and texture coordinates
        vtx = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
        tex = np.asanyarray(points.get_texture_coordinates()).view(np.float32).reshape(-1, 2)
        
        # Get color image
        color_image = np.asanyarray(color_frame.get_data())
        
        # Map texture coordinates to colors
        h, w = color_image.shape[:2]
        u = np.clip(tex[:, 0] * w, 0, w-1).astype(int)
        v = np.clip(tex[:, 1] * h, 0, h-1).astype(int)
        colors = color_image[v, u] / 255.0  # Normalize to [0, 1]
        
        # Convert to Open3D format (BGR -> RGB)
        colors = colors[:, [2, 1, 0]]
        
        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(vtx)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
        # Remove invalid points (zero depth)
        valid_mask = np.any(vtx != 0, axis=1)
        pcd = pcd.select_by_index(np.where(valid_mask)[0])
        
        # Save if path provided
        if save_path:
            o3d.io.write_point_cloud(str(save_path), pcd)
            print(f"Point cloud saved to: {save_path}")
        
        return pcd
    
    def visualize_point_cloud(self, pcd, window_name="Point Cloud"):
        """
        Visualize point cloud using Open3D.
        
        Args:
            pcd: Open3D point cloud
            window_name: Window title
        """
        print("\nPoint Cloud Visualization Controls:")
        print("  - Mouse: Rotate view")
        print("  - Scroll: Zoom")
        print("  - Ctrl + Mouse: Pan")
        print("  - Q or ESC: Close window\n")
        
        o3d.visualization.draw_geometries(
            [pcd],
            window_name=window_name,
            width=1024,
            height=768,
            left=50,
            top=50
        )
    
    def get_camera_intrinsics_info(self):
        """
        Get and display camera intrinsic parameters.
        
        Returns:
            dict with intrinsic parameters
        """
        if self.depth_intrinsics is None:
            print("No frames captured yet. Capture a frame first.")
            return None
        
        depth_info = {
            'width': self.depth_intrinsics.width,
            'height': self.depth_intrinsics.height,
            'fx': self.depth_intrinsics.fx,
            'fy': self.depth_intrinsics.fy,
            'cx': self.depth_intrinsics.ppx,
            'cy': self.depth_intrinsics.ppy,
            'distortion_model': str(self.depth_intrinsics.model),
            'distortion_coeffs': list(self.depth_intrinsics.coeffs)
        }
        
        color_info = {
            'width': self.color_intrinsics.width,
            'height': self.color_intrinsics.height,
            'fx': self.color_intrinsics.fx,
            'fy': self.color_intrinsics.fy,
            'cx': self.color_intrinsics.ppx,
            'cy': self.color_intrinsics.ppy,
            'distortion_model': str(self.color_intrinsics.model),
            'distortion_coeffs': list(self.color_intrinsics.coeffs)
        }
        
        intrinsics_data = {
            'depth': depth_info,
            'color': color_info,
            'depth_scale': self.depth_scale
        }
        
        # Save to file
        filepath = self.output_dir / 'camera_intrinsics.json'
        with open(filepath, 'w') as f:
            json.dump(intrinsics_data, f, indent=2)
        
        print("="*60)
        print("CAMERA INTRINSIC PARAMETERS")
        print("="*60)
        print("\nDEPTH CAMERA:")
        print(f"  Resolution: {depth_info['width']} x {depth_info['height']}")
        print(f"  Focal Length: fx={depth_info['fx']:.2f}, fy={depth_info['fy']:.2f}")
        print(f"  Principal Point: cx={depth_info['cx']:.2f}, cy={depth_info['cy']:.2f}")
        print(f"  Distortion Model: {depth_info['distortion_model']}")
        
        print("\nCOLOR CAMERA:")
        print(f"  Resolution: {color_info['width']} x {color_info['height']}")
        print(f"  Focal Length: fx={color_info['fx']:.2f}, fy={color_info['fy']:.2f}")
        print(f"  Principal Point: cx={color_info['cx']:.2f}, cy={color_info['cy']:.2f}")
        print(f"  Distortion Model: {color_info['distortion_model']}")
        
        print(f"\nDepth Scale: {self.depth_scale} meters/unit")
        print(f"\nSaved to: {filepath}")
        print("="*60)
        
        return intrinsics_data
    
    def pixel_to_3d_point(self, pixel_x, pixel_y, depth_value):
        """
        Convert pixel coordinates to 3D point in camera frame.
        
        Args:
            pixel_x, pixel_y: Pixel coordinates
            depth_value: Depth value at that pixel (in depth units)
            
        Returns:
            (x, y, z) in meters
        """
        if self.depth_intrinsics is None:
            print("No intrinsics available. Capture a frame first.")
            return None
        
        depth_m = depth_value * self.depth_scale
        
        # Use intrinsic parameters to deproject
        point_3d = rs.rs2_deproject_pixel_to_point(
            self.depth_intrinsics,
            [pixel_x, pixel_y],
            depth_m
        )
        
        return point_3d
    
    def demonstrate_coordinate_transform(self):
        """
        Interactive demonstration of coordinate transformations.
        """
        print("\n" + "="*60)
        print("COORDINATE TRANSFORMATION DEMO")
        print("="*60)
        print("\nClick on the image to see 3D coordinates!")
        print("Press 'q' to quit, 's' to save point cloud\n")
        
        # Mouse callback
        clicked_point = {'x': None, 'y': None}
        
        def mouse_callback(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                clicked_point['x'] = x
                clicked_point['y'] = y
        
        cv2.namedWindow('Click for 3D Coordinates')
        cv2.setMouseCallback('Click for 3D Coordinates', mouse_callback)
        
        while True:
            # Get frames
            frames_data = self.get_frames(aligned=True, apply_filters=True)
            if frames_data is None:
                continue
            
            depth_image = frames_data['depth']
            color_image = frames_data['color']
            
            # Create visualization
            vis = color_image.copy()
            
            # If point was clicked, show coordinates
            if clicked_point['x'] is not None:
                px, py = clicked_point['x'], clicked_point['y']
                
                # Get depth at clicked point
                depth_val = depth_image[py, px]
                
                if depth_val > 0:
                    # Convert to 3D
                    point_3d = self.pixel_to_3d_point(px, py, depth_val)
                    
                    # Draw crosshair
                    cv2.drawMarker(vis, (px, py), (0, 255, 0), 
                                 cv2.MARKER_CROSS, 20, 2)
                    
                    # Display coordinates
                    text = f"3D: ({point_3d[0]*100:.1f}, {point_3d[1]*100:.1f}, {point_3d[2]*100:.1f}) cm"
                    cv2.putText(vis, text, (px+10, py-10),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # Print to console
                    print(f"\nPixel ({px}, {py}) -> 3D Point: "
                          f"X={point_3d[0]*100:.2f} cm, "
                          f"Y={point_3d[1]*100:.2f} cm, "
                          f"Z={point_3d[2]*100:.2f} cm")
            
            # Show FPS
            cv2.putText(vis, "Click to measure 3D coordinates | 'q' quit | 's' save point cloud",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            cv2.imshow('Click for 3D Coordinates', vis)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                # Save point cloud
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                save_path = self.output_dir / f"pointcloud_{timestamp}.ply"
                pcd = self.generate_point_cloud(
                    frames_data['depth_frame'],
                    frames_data['color_frame'],
                    save_path=save_path
                )
                print(f"\n✓ Point cloud saved! ({len(pcd.points)} points)")
        
        cv2.destroyAllWindows()
    
    def compare_filtering_methods(self, duration_sec=5):
        """
        Compare raw vs filtered depth maps side-by-side.
        
        Args:
            duration_sec: How long to display comparison
        """
        print("\n" + "="*60)
        print("DEPTH FILTERING COMPARISON")
        print("="*60)
        print(f"\nDisplaying comparison for {duration_sec} seconds...")
        print("Press 'q' to quit early\n")
        
        start_time = cv2.getTickCount()
        
        while True:
            # Get raw frames
            frames_raw = self.get_frames(aligned=True, apply_filters=False)
            frames_filtered = self.get_frames(aligned=True, apply_filters=True)
            
            if frames_raw is None or frames_filtered is None:
                continue
            
            # Create side-by-side comparison
            raw_colormap = frames_raw['depth_colormap']
            filtered_colormap = frames_filtered['depth_colormap']
            
            # Add labels
            cv2.putText(raw_colormap, "RAW DEPTH", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(filtered_colormap, "FILTERED DEPTH", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            comparison = np.hstack([raw_colormap, filtered_colormap])
            
            cv2.imshow('Depth Filtering Comparison', comparison)
            
            # Check time
            elapsed = (cv2.getTickCount() - start_time) / cv2.getTickFrequency()
            if elapsed >= duration_sec:
                break
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cv2.destroyAllWindows()
        print("✓ Comparison complete!")
    
    def measure_workspace_guide(self):
        """
        Interactive guide for measuring workspace dimensions.
        """
        print("\n" + "="*60)
        print("WORKSPACE MEASUREMENT GUIDE")
        print("="*60)
        print("\nThis tool helps you plan your overhead camera setup.")
        print("\nSTEPS:")
        print("1. Position camera at your intended mounting height")
        print("2. Point it at your workspace")
        print("3. We'll calculate the coverage area\n")
        
        input("Press ENTER when camera is positioned...")
        
        # Capture frame
        frames = self.get_frames(aligned=True, apply_filters=True)
        
        # Calculate field of view
        h_fov = 2 * np.arctan(self.depth_intrinsics.width / (2 * self.depth_intrinsics.fx))
        v_fov = 2 * np.arctan(self.depth_intrinsics.height / (2 * self.depth_intrinsics.fy))
        
        h_fov_deg = np.degrees(h_fov)
        v_fov_deg = np.degrees(v_fov)
        
        # Get center depth
        depth_image = frames['depth']
        h, w = depth_image.shape
        center_depth = depth_image[h//2, w//2] * self.depth_scale
        
        # Calculate coverage at this height
        horizontal_coverage = 2 * center_depth * np.tan(h_fov / 2)
        vertical_coverage = 2 * center_depth * np.tan(v_fov / 2)
        
        print("\n" + "="*60)
        print("WORKSPACE COVERAGE ANALYSIS")
        print("="*60)
        print(f"\nCamera Height: {center_depth:.2f} meters ({center_depth*100:.0f} cm)")
        print(f"\nField of View:")
        print(f"  Horizontal: {h_fov_deg:.1f}°")
        print(f"  Vertical: {v_fov_deg:.1f}°")
        print(f"\nWorkspace Coverage at Current Height:")
        print(f"  Horizontal: {horizontal_coverage:.2f} m ({horizontal_coverage*100:.0f} cm)")
        print(f"  Vertical: {vertical_coverage:.2f} m ({vertical_coverage*100:.0f} cm)")
        print(f"  Total Area: {horizontal_coverage * vertical_coverage:.2f} m²")
        
        # Recommendations
        print("\n" + "="*60)
        print("RECOMMENDATIONS")
        print("="*60)
        print("\nBased on your Week 2 accuracy tests:")
        print("  • Optimal height: 1.5-2.5 meters")
        print("  • Expected accuracy: ±1-3 cm")
        print("  • Avoid heights > 3.0 meters (accuracy degrades)")
        
        if center_depth > 3.0:
            print("\n⚠ WARNING: Current height may exceed optimal range!")
        elif center_depth < 1.0:
            print("\n⚠ WARNING: Height may be too low for full workspace view!")
        else:
            print("\n✓ Current height is in optimal range!")
        
        # Save measurements
        measurements = {
            'timestamp': datetime.now().isoformat(),
            'camera_height_m': center_depth,
            'horizontal_fov_deg': h_fov_deg,
            'vertical_fov_deg': v_fov_deg,
            'horizontal_coverage_m': horizontal_coverage,
            'vertical_coverage_m': vertical_coverage,
            'coverage_area_m2': horizontal_coverage * vertical_coverage
        }
        
        filepath = self.output_dir / 'workspace_measurements.json'
        with open(filepath, 'w') as f:
            json.dump(measurements, f, indent=2)
        
        print(f"\n✓ Measurements saved to: {filepath}")
        print("="*60)
    
    def shutdown(self):
        """Stop camera pipeline."""
        print("\nShutting down camera...")
        self.pipeline.stop()
        print("Done!")


# Main demonstration
if __name__ == "__main__":
    print("="*60)
    print("Week 3: Data Processing and Coordinate Systems")
    print("="*60)
    
    processor = RealSenseDataProcessor()
    
    try:
        while True:
            print("\n" + "="*60)
            print("WEEK 3 TASKS MENU")
            print("="*60)
            print("1. Compare Raw vs Filtered Depth")
            print("2. Generate and View Point Cloud")
            print("3. Display Camera Intrinsics")
            print("4. Interactive Coordinate Transform Demo")
            print("5. Workspace Measurement Guide")
            print("6. Exit")
            print("="*60)
            
            choice = input("\nSelect task (1-6): ").strip()
            
            if choice == '1':
                processor.compare_filtering_methods(duration_sec=10)
            
            elif choice == '2':
                print("\nCapturing point cloud...")
                frames = processor.get_frames(aligned=True, apply_filters=True)
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                save_path = processor.output_dir / f"pointcloud_{timestamp}.ply"
                
                pcd = processor.generate_point_cloud(
                    frames['depth_frame'],
                    frames['color_frame'],
                    save_path=save_path
                )
                
                print(f"\n✓ Generated point cloud with {len(pcd.points)} points")
                
                view = input("View point cloud? (y/n): ").strip().lower()
                if view == 'y':
                    processor.visualize_point_cloud(pcd)
            
            elif choice == '3':
                # Capture a frame first to get intrinsics
                frames = processor.get_frames()
                processor.get_camera_intrinsics_info()
            
            elif choice == '4':
                processor.demonstrate_coordinate_transform()
            
            elif choice == '5':
                processor.measure_workspace_guide()
            
            elif choice == '6':
                break
            
            else:
                print("Invalid choice. Please select 1-6.")
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
    
    finally:
        processor.shutdown()
        print("\nWeek 3 tasks complete!")
