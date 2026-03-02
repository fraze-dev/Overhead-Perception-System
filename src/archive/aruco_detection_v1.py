"""
ArUco Marker Detection Script
Author: Aaron Fraze
Date: February 16, 2026
Purpose: Test ArUco marker detection for overhead robot tracking
"""

import sys
import pyrealsense2 as rs
import numpy as np
import cv2
from cv2 import aruco
import time

"""SET DESIRED RESOLUTION"""
resolution_width = 1280
resolution_height = 720


class ArucoDetector:
    """
    ArUco marker detector for overhead tracking system.
    """
    
    def __init__(self, aruco_dict_type=aruco.DICT_6X6_250):
        """
        Initialize ArUco detector.
        
        Args:
            aruco_dict_type: ArUco dictionary to use (default: DICT_6X6_250)
        """
        # Initialize RealSense camera
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Configure streams
        self.config.enable_stream(rs.stream.depth, resolution_width, resolution_height, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, resolution_width, resolution_height, rs.format.bgr8, 30)
        
        try:
            self.profile = self.pipeline.start(self.config)
            print("Camera initialized")
        except RuntimeError as e:
            if "No device connected" in str(e):
                print("Camera not found.")
                print("Check if camera is connected and try again.")
                sys.exit(1)
            else:
                print(f"Camera runtime error: {e}")
                sys.exit(1)
        except Exception as e:
            print(f"Camera not initialized. Error: {e}")
            sys.exit(1)
        
        # Get device and sensors
        self.device = self.profile.get_device()
        self.depth_sensor = self.device.first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        
        # Get intrinsics
        self.depth_intrinsics = None
        self.color_intrinsics = None
        
        # Create alignment object
        self.align = rs.align(rs.stream.color)
        
        # ArUco setup
        self.aruco_dict_type = aruco_dict_type
        self.aruco_dict = aruco.getPredefinedDictionary(aruco_dict_type)
        self.aruco_params = aruco.DetectorParameters()
        
        # Create detector
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Dictionary name for display
        self.dict_names = {
            aruco.DICT_4X4_50: "DICT_4X4_50",
            aruco.DICT_4X4_100: "DICT_4X4_100",
            aruco.DICT_4X4_250: "DICT_4X4_250",
            aruco.DICT_5X5_50: "DICT_5X5_50",
            aruco.DICT_5X5_100: "DICT_5X5_100",
            aruco.DICT_5X5_250: "DICT_5X5_250",
            aruco.DICT_6X6_50: "DICT_6X6_50",
            aruco.DICT_6X6_100: "DICT_6X6_100",
            aruco.DICT_6X6_250: "DICT_6X6_250",
            aruco.DICT_7X7_50: "DICT_7X7_50",
            aruco.DICT_7X7_100: "DICT_7X7_100",
            aruco.DICT_7X7_250: "DICT_7X7_250",
        }
        
        # Camera matrix for pose estimation (will be populated from intrinsics)
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Allow camera to warm up
        print("Warming up camera for 3 seconds...")
        for _ in range(90):
            self.pipeline.wait_for_frames()
        print("Ready!")
    
    def get_frame(self):
        """
        Capture and process frames.
        Returns:
            dict with color_image, depth_image, depth_frame, color_frame
        """
        frames = self.pipeline.wait_for_frames()
        aligned_frame = self.align.process(frames)
        depth_frame = aligned_frame.get_depth_frame()
        color_frame = aligned_frame.get_color_frame()
        
        if not depth_frame or not color_frame:
            return None
        
        # Store intrinsics on first frame
        if self.depth_intrinsics is None:
            self.depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
            self.color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
            
            # Build camera matrix for pose estimation
            self.camera_matrix = np.array([
                [self.color_intrinsics.fx, 0, self.color_intrinsics.ppx],
                [0, self.color_intrinsics.fy, self.color_intrinsics.ppy],
                [0, 0, 1]
            ])
            self.dist_coeffs = np.array(self.color_intrinsics.coeffs)
        
        # Convert to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        return {
            "color_frame": color_frame,
            "depth_frame": depth_frame,
            "color_image": color_image,
            "depth_image": depth_image,
        }
    
    def pixel_to_3d_point(self, pixel_x, pixel_y, depth_value):
        """
        Convert pixel coordinates to 3D point in camera frame.
        Returns:
            (x, y, z) in meters
        """
        if self.depth_intrinsics is None:
            return None
        
        depth_m = depth_value * self.depth_scale
        
        point_3d = rs.rs2_deproject_pixel_to_point(
            self.depth_intrinsics,
            [pixel_x, pixel_y],
            depth_m
        )
        
        return point_3d
    
    def detect_markers(self, color_image, depth_image, marker_size_m=0.10, estimate_pose=True):
        """
        Detect ArUco markers in image.
        
        Args:
            color_image: BGR color image
            depth_image: Depth image (uint16)
            marker_size_m: Physical size of marker in meters (e.g., 0.10 for 10cm)
            estimate_pose: Whether to estimate 3D pose
            
        Returns:
            list of detected markers with ID, corners, center, and pose info
        """
        # Convert to grayscale for detection
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        
        # Detect markers
        corners, ids, rejected = self.detector.detectMarkers(gray)
        
        if ids is None:
            return []
        
        detected_markers = []
        
        for i, marker_id in enumerate(ids.flatten()):
            marker_corners = corners[i][0]  # Shape: (4, 2)
            
            # Calculate center
            center_x = int(np.mean(marker_corners[:, 0]))
            center_y = int(np.mean(marker_corners[:, 1]))
            
            # Get depth at center
            depth_value = depth_image[center_y, center_x]
            
            # Get 3D position
            point_3d = None
            if depth_value > 0:
                point_3d = self.pixel_to_3d_point(center_x, center_y, depth_value)
            
            # Estimate pose if requested
            rvec = None
            tvec = None
            if estimate_pose and self.camera_matrix is not None:
                # Estimate pose - handle both old and new OpenCV versions
                try:
                    # Try new OpenCV 4.7+ method
                    objPoints = np.array([
                        [-marker_size_m/2, marker_size_m/2, 0],
                        [marker_size_m/2, marker_size_m/2, 0],
                        [marker_size_m/2, -marker_size_m/2, 0],
                        [-marker_size_m/2, -marker_size_m/2, 0]
                    ], dtype=np.float32)
                    
                    success, rvec, tvec = cv2.solvePnP(
                        objPoints,
                        marker_corners,
                        self.camera_matrix,
                        self.dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )
                    
                    if success:
                        rvec = rvec.flatten()
                        tvec = tvec.flatten()
                    else:
                        rvec = None
                        tvec = None
                        
                except Exception as e:
                    # Fallback to old method if available
                    try:
                        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                            [marker_corners],
                            marker_size_m,
                            self.camera_matrix,
                            self.dist_coeffs
                        )
                        rvec = rvec[0][0]
                        tvec = tvec[0][0]
                    except:
                        # If both fail, skip pose estimation
                        rvec = None
                        tvec = None
            
            detected_markers.append({
                'id': int(marker_id),
                'corners': marker_corners,
                'center_pixel': (center_x, center_y),
                'depth_value': depth_value,
                'point_3d': point_3d,
                'rvec': rvec,  # Rotation vector
                'tvec': tvec,  # Translation vector
            })
        
        return detected_markers
    
    def draw_markers(self, image, detected_markers, draw_pose=True, draw_info=True):
        """
        Draw detected markers on image.
        
        Args:
            image: Image to draw on
            detected_markers: List of detected markers from detect_markers()
            draw_pose: Whether to draw pose axes
            draw_info: Whether to draw ID and position info
            
        Returns:
            Image with markers drawn
        """
        vis = image.copy()
        
        for marker in detected_markers:
            corners = marker['corners']
            marker_id = marker['id']
            center = marker['center_pixel']
            
            # Draw marker outline
            corners_int = corners.astype(int)
            cv2.polylines(vis, [corners_int], True, (0, 255, 0), 2)
            
            # Draw corners
            for corner in corners_int:
                cv2.circle(vis, tuple(corner), 4, (0, 0, 255), -1)
            
            # Draw center
            cv2.circle(vis, center, 5, (255, 0, 0), -1)
            
            # Draw ID
            cv2.putText(vis, f"ID: {marker_id}", 
                       (center[0] - 30, center[1] - 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            # Draw pose axes if available
            if draw_pose and marker['rvec'] is not None and marker['tvec'] is not None:
                cv2.drawFrameAxes(vis, self.camera_matrix, self.dist_coeffs,
                                 marker['rvec'], marker['tvec'], 0.05)
            
            # Draw 3D position info
            if draw_info and marker['point_3d'] is not None:
                p3d = marker['point_3d']
                # Convert to world coordinates (cm)
                world_x = p3d[0] * 100
                world_y = -p3d[1] * 100  # Flip Y
                world_z = p3d[2] * 100
                
                pos_text = f"Pos: ({world_x:.1f}, {world_y:.1f}, {world_z:.1f}) cm"
                cv2.putText(vis, pos_text,
                           (center[0] - 100, center[1] + 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        return vis
    
    def aruco_detection_demo(self, marker_size_cm=10.0):
        """
        Live ArUco marker detection demo with visualization.
        
        Args:
            marker_size_cm: Physical size of marker in centimeters
        """
        print("\n" + "=" * 60)
        print("ARUCO MARKER DETECTION DEMO")
        print("=" * 60)
        print(f"Dictionary: {self.dict_names.get(self.aruco_dict_type, 'Unknown')}")
        print(f"Marker size: {marker_size_cm} cm")
        print("Press 'q' to quit")
        print("Press 's' to save detection info")
        print("=" * 60)
        
        cv2.namedWindow('ArUco Detection')
        
        detection_count = 0
        frame_count = 0
        start_time = time.time()
        
        while True:
            frames_data = self.get_frame()
            if frames_data is None:
                continue
            
            color_image = frames_data['color_image']
            depth_image = frames_data['depth_image']
            
            frame_count += 1
            
            # Detect markers
            marker_size_m = marker_size_cm / 100.0
            detected_markers = self.detect_markers(color_image, depth_image, 
                                                   marker_size_m=marker_size_m,
                                                   estimate_pose=True)
            
            if len(detected_markers) > 0:
                detection_count += 1
            
            # Draw markers
            vis = self.draw_markers(color_image, detected_markers, 
                                   draw_pose=True, draw_info=True)


            # Calculate FPS
            elapsed = time.time() - start_time
            fps = frame_count / elapsed if elapsed > 0 else 0
            
            # Draw status info
            status_text = f"Markers: {len(detected_markers)} | FPS: {fps:.1f}"
            cv2.putText(vis, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                       0.8, (255, 255, 255), 2)
            
            dict_text = f"Dict: {self.dict_names.get(self.aruco_dict_type, 'Unknown')}"
            cv2.putText(vis, dict_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                       0.6, (200, 200, 200), 1)
            
            # Draw marker IDs if detected
            if len(detected_markers) > 0:
                ids_text = "IDs: " + ", ".join([str(m['id']) for m in detected_markers])
                cv2.putText(vis, ids_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX,
                           0.6, (0, 255, 0), 2)
            
            cv2.imshow('ArUco Detection', vis)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                # Save detection info
                if len(detected_markers) > 0:
                    print("\n" + "=" * 60)
                    print("DETECTED MARKERS:")
                    for marker in detected_markers:
                        print(f"\n  Marker ID: {marker['id']}")
                        print(f"  Center pixel: {marker['center_pixel']}")
                        if marker['point_3d']:
                            p3d = marker['point_3d']
                            print(f"  World position: ({p3d[0]*100:.1f}, {-p3d[1]*100:.1f}, {p3d[2]*100:.1f}) cm")
                        if marker['rvec'] is not None:
                            print(f"  Rotation vector: {marker['rvec']}")
                            print(f"  Translation vector: {marker['tvec']}")
                    print("=" * 60)
                else:
                    print("\nNo markers detected to save.")
        
        cv2.destroyAllWindows()
        
        # Print statistics
        detection_rate = (detection_count / frame_count * 100) if frame_count > 0 else 0
        print(f"\nDetection Statistics:")
        print(f"  Total frames: {frame_count}")
        print(f"  Frames with detections: {detection_count}")
        print(f"  Detection rate: {detection_rate:.1f}%")
        print(f"  Average FPS: {fps:.1f}")
    
    def marker_distance_test(self, marker_size_cm=10.0):
        """
        Test marker detection at various distances.
        Useful for determining optimal marker size.
        """
        print("\n" + "=" * 60)
        print("MARKER DISTANCE TEST")
        print("=" * 60)
        print("This test measures detection success at different distances.")
        print("Place marker at various heights/distances and record results.")
        print("Press SPACE to record measurement, 'q' to quit")
        print("=" * 60)
        
        cv2.namedWindow('Distance Test')
        measurements = []
        
        while True:
            frames_data = self.get_frame()
            if frames_data is None:
                continue
            
            color_image = frames_data['color_image']
            depth_image = frames_data['depth_image']
            
            # Detect markers
            marker_size_m = marker_size_cm / 100.0
            detected_markers = self.detect_markers(color_image, depth_image,
                                                   marker_size_m=marker_size_m,
                                                   estimate_pose=True)
            
            # Draw markers
            vis = self.draw_markers(color_image, detected_markers,
                                   draw_pose=True, draw_info=True)
            
            # Instructions
            status = "DETECTED" if len(detected_markers) > 0 else "NO DETECTION"
            color = (0, 255, 0) if len(detected_markers) > 0 else (0, 0, 255)
            
            cv2.putText(vis, f"Status: {status}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
            cv2.putText(vis, "SPACE: Record | Q: Quit", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            if len(detected_markers) > 0 and detected_markers[0]['point_3d']:
                distance = detected_markers[0]['point_3d'][2] * 100  # cm
                cv2.putText(vis, f"Distance: {distance:.1f} cm", (10, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            cv2.imshow('Distance Test', vis)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord(' '):
                if len(detected_markers) > 0:
                    marker = detected_markers[0]
                    if marker['point_3d']:
                        distance = marker['point_3d'][2] * 100
                        measurements.append({
                            'distance_cm': distance,
                            'marker_id': marker['id'],
                            'detected': True
                        })
                        print(f"  Recorded: Distance = {distance:.1f} cm, ID = {marker['id']}")
                else:
                    print("  No marker detected at this position")
        
        cv2.destroyAllWindows()
        
        # Print summary
        if len(measurements) > 0:
            print("\n" + "=" * 60)
            print("DISTANCE TEST RESULTS:")
            print("=" * 60)
            distances = [m['distance_cm'] for m in measurements]
            print(f"  Measurements: {len(measurements)}")
            print(f"  Min distance: {min(distances):.1f} cm")
            print(f"  Max distance: {max(distances):.1f} cm")
            print(f"  Mean distance: {np.mean(distances):.1f} cm")
            print("=" * 60)
    
    def shutdown(self):
        """Stop camera pipeline."""
        print("\nShutting down camera...")
        self.pipeline.stop()
        print("Done!")


def print_aruco_info():
    """Print information about available ArUco dictionaries."""
    print("\n" + "=" * 60)
    print("AVAILABLE ARUCO DICTIONARIES")
    print("=" * 60)
    print("\nDictionary Format: DICT_[bits]X[bits]_[count]")
    print("  - bits: Size of marker grid (4x4, 5x5, 6x6, 7x7)")
    print("  - count: Number of unique IDs available")
    print("\nRecommendations:")
    print("  - For overhead at 220cm: DICT_6X6_250 or DICT_5X5_100")
    print("  - Larger grids (6x6, 7x7) = better detection at distance")
    print("  - Smaller grids (4x4) = faster detection but less robust")
    print("\nAvailable Dictionaries:")
    print("  1. DICT_4X4_50    - 4x4 grid, 50 IDs")
    print("  2. DICT_4X4_100   - 4x4 grid, 100 IDs")
    print("  3. DICT_4X4_250   - 4x4 grid, 250 IDs")
    print("  4. DICT_5X5_50    - 5x5 grid, 50 IDs")
    print("  5. DICT_5X5_100   - 5x5 grid, 100 IDs")
    print("  6. DICT_5X5_250   - 5x5 grid, 250 IDs")
    print("  7. DICT_6X6_50    - 6x6 grid, 50 IDs (Recommended)")
    print("  8. DICT_6X6_100   - 6x6 grid, 100 IDs (Recommended)")
    print("  9. DICT_6X6_250   - 6x6 grid, 250 IDs (Recommended)")
    print(" 10. DICT_7X7_50    - 7x7 grid, 50 IDs")
    print(" 11. DICT_7X7_100   - 7x7 grid, 100 IDs")
    print(" 12. DICT_7X7_250   - 7x7 grid, 250 IDs")
    print("=" * 60)


def main():
    print("=" * 60)
    print("ArUco Marker Detection for Overhead Tracking")
    print("=" * 60)
    
    # Show dictionary info
    print_aruco_info()
    
    # Select dictionary
    print("\nSelect ArUco dictionary to use:")
    dict_choice = input("Enter number (1-12) or press ENTER for default (DICT_6X6_250): ").strip()
    
    dict_map = {
        '1': aruco.DICT_4X4_50,
        '2': aruco.DICT_4X4_100,
        '3': aruco.DICT_4X4_250,
        '4': aruco.DICT_5X5_50,
        '5': aruco.DICT_5X5_100,
        '6': aruco.DICT_5X5_250,
        '7': aruco.DICT_6X6_50,
        '8': aruco.DICT_6X6_100,
        '9': aruco.DICT_6X6_250,
        '10': aruco.DICT_7X7_50,
        '11': aruco.DICT_7X7_100,
        '12': aruco.DICT_7X7_250,
    }
    
    selected_dict = dict_map.get(dict_choice, aruco.DICT_6X6_250)
    
    # Get marker size
    marker_size = input("\nEnter physical marker size in cm (default 10): ").strip()
    try:
        marker_size_cm = float(marker_size) if marker_size else 10.0
    except ValueError:
        marker_size_cm = 10.0
    
    # Initialize detector
    print("\nInitializing ArUco detector...")
    detector = ArucoDetector(aruco_dict_type=selected_dict)
    
    try:
        print("\n" + "=" * 60)
        print("DEMO MENU")
        print("=" * 60)
        print("1. Live Detection Demo (real-time visualization)")
        print("2. Distance Test (measure detection at various distances)")
        print("=" * 60)
        
        choice = input("\nEnter choice (1-2): ").strip()
        
        if choice == '1':
            detector.aruco_detection_demo(marker_size_cm=marker_size_cm)
        elif choice == '2':
            detector.marker_distance_test(marker_size_cm=marker_size_cm)
        else:
            print("Invalid choice. Running live detection demo...")
            detector.aruco_detection_demo(marker_size_cm=marker_size_cm)
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
    
    finally:
        detector.shutdown()
        print("\nProgram ended.")


if __name__ == "__main__":
    main()
