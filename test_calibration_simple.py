"""
Simple Calibration Test Script
Author: Aaron Fraze
Date: February 3, 2026

This script demonstrates the calibration workflow in a simplified way.
Good for testing and understanding the system before running full interactive calibration.
"""

import pyrealsense2 as rs
import numpy as np
import cv2
from world_frame_calibration import WorldFrameCalibrator


def quick_calibration_test():
    """
    Run a quick calibration test with simulated or manual measurements.
    """
    print("="*60)
    print("QUICK CALIBRATION TEST")
    print("="*60)
    print()
    
    # Initialize camera
    print("1. Initializing camera...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    
    profile = pipeline.start(config)
    
    # Warm up
    for _ in range(30):
        pipeline.wait_for_frames()
    
    print("✓ Camera ready")
    
    # Initialize calibrator
    print("\n2. Setting up calibrator...")
    calibrator = WorldFrameCalibrator()
    calibrator.setup_camera(pipeline)
    print("✓ Calibrator initialized")
    
    # Define simple calibration
    print("\n3. Defining world frame calibration...")
    print("\nAssuming:")
    print("  - Camera mounted at 220 cm height")
    print("  - Camera centered at (0, 0) in X-Y plane")
    print("  - Camera looking straight down (no tilt/pan/roll)")
    print()
    
    calibrator.define_simple_overhead_calibration(
        camera_height_cm=220.0,
        camera_x_world=0.0,
        camera_y_world=0.0,
        camera_tilt_deg=0.0,
        camera_pan_deg=0.0,
        camera_roll_deg=0.0
    )
    print("✓ World frame calibration defined")
    
    # Test transformation on center pixel
    print("\n4. Testing coordinate transformation...")
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    
    if depth_frame:
        # Test center pixel
        cx = 640 // 2
        cy = 480 // 2
        
        depth_meters = depth_frame.get_distance(cx, cy)
        depth_cm = depth_meters * 100
        
        print(f"\nCenter pixel ({cx}, {cy}):")
        print(f"  Depth: {depth_cm:.2f} cm")
        
        # Convert to camera coordinates
        point_camera = calibrator.pixel_to_camera_coordinates(cx, cy, depth_meters)
        print(f"  Camera frame: [{point_camera[0]*100:.2f}, {point_camera[1]*100:.2f}, {point_camera[2]*100:.2f}] cm")
        
        # Convert to world coordinates
        point_world = calibrator.camera_to_world_coordinates(point_camera, return_cm=True)
        print(f"  World frame:  [{point_world[0]:.2f}, {point_world[1]:.2f}, {point_world[2]:.2f}] cm")
        
        # For overhead camera, Z should be close to 0 (ground plane)
        print(f"\n  Expected Z ≈ 0 cm (ground plane)")
        print(f"  Actual Z = {point_world[2]:.2f} cm")
        
        if abs(point_world[2]) < 5:
            print("  ✓ Looks good!")
        else:
            print("  ⚠ Check your calibration parameters!")
    
    # Generate point cloud and visualize
    print("\n5. Generating world point cloud...")
    points_world = calibrator.depth_image_to_world_points(
        depth_frame, 
        subsample=10,  # Sample every 10th pixel
        max_distance_cm=300.0
    )
    
    print(f"  Generated {len(points_world)} points")
    print(f"  X range: [{np.min(points_world[:, 0]):.2f}, {np.max(points_world[:, 0]):.2f}] cm")
    print(f"  Y range: [{np.min(points_world[:, 1]):.2f}, {np.max(points_world[:, 1]):.2f}] cm")
    print(f"  Z range: [{np.min(points_world[:, 2]):.2f}, {np.max(points_world[:, 2]):.2f}] cm")
    
    # Create visualization
    print("\n6. Creating top-down visualization...")
    viz_image = calibrator.visualize_world_points_top_down(points_world)
    
    # Show live feed with overlay
    print("\n7. Displaying results...")
    print("   Press 'q' to quit")
    print("   Press 's' to save calibration")
    
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        
        if not color_frame:
            continue
        
        color_image = np.asanyarray(color_frame.get_data())
        
        # Add info overlay
        cv2.putText(color_image, "Quick Calibration Test", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(color_image, f"Camera height: 200 cm", 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(color_image, "Press 'q' to quit, 's' to save", 
                   (10, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # Draw center crosshair
        cv2.line(color_image, (320-20, 240), (320+20, 240), (0, 255, 0), 2)
        cv2.line(color_image, (320, 240-20), (320, 240+20), (0, 255, 0), 2)
        
        cv2.imshow('Camera View', color_image)
        cv2.imshow('Top-Down World View', viz_image)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            calibrator.save_calibration("quick_test_calibration.json")
            print("\n✓ Calibration saved!")
    
    # Cleanup
    cv2.destroyAllWindows()
    pipeline.stop()
    
    print("\n" + "="*60)
    print("Test complete!")
    print("="*60)
    print("\nNext steps:")
    print("1. Measure your actual camera height and position")
    print("2. Run the full interactive calibration:")
    print("   python world_frame_calibration.py")
    print("3. Or modify this script with your actual measurements")


def demonstrate_coordinate_systems():
    """
    Print a helpful explanation of coordinate systems.
    """
    print("\n" + "="*60)
    print("COORDINATE SYSTEM GUIDE")
    print("="*60)
    print()
    
    print("CAMERA FRAME (what the RealSense sees):")
    print("  Origin: Camera lens")
    print("  X-axis: Points RIGHT in image")
    print("  Y-axis: Points DOWN in image")
    print("  Z-axis: Points INTO the scene (depth direction)")
    print()
    
    print("WORLD FRAME (your workspace):")
    print("  Origin: Corner or center of arena (you choose)")
    print("  X-axis: Length of arena (e.g., toward goal)")
    print("  Y-axis: Width of arena (perpendicular to X)")
    print("  Z-axis: Height above ground (usually 0 at ground)")
    print()
    
    print("TRANSFORMATION:")
    print("  Pixel (u,v) + Depth → Camera frame (X_cam, Y_cam, Z_cam)")
    print("  Camera frame → World frame (X_world, Y_world, Z_world)")
    print()
    
    print("KEY CALIBRATION PARAMETERS:")
    print("  1. Camera height: Distance from lens to ground plane")
    print("  2. Camera position: (X, Y) location in world frame")
    print("  3. Camera orientation: Tilt, pan, roll angles")
    print()
    
    print("VALIDATION:")
    print("  Place objects at known world positions")
    print("  Click on them in camera view")
    print("  Compare predicted vs. true positions")
    print("  Good calibration: <2-3 cm error")
    print()


if __name__ == "__main__":
    print("\n" + "="*60)
    print("CALIBRATION TEST SCRIPT")
    print("="*60)
    print()
    print("This script will:")
    print("1. Initialize your RealSense camera")
    print("2. Set up a simple overhead calibration")
    print("3. Transform depth data to world coordinates")
    print("4. Visualize the results")
    print()
    
    demonstrate_coordinate_systems()
    
    input("\nPress ENTER to start the test...")
    
    try:
        quick_calibration_test()
    except KeyboardInterrupt:
        print("\n\nTest interrupted.")
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()
