"""
RGB to Depth Frame Alignment for Intel RealSense D435
Author: Aaron Fraze
Date: January 24, 2026
Purpose: Implement and demonstrate frame alignment between RGB and depth streams

Frame alignment is necessary because:
1. RGB and depth sensors are physically separated on the D435
2. They have different fields of view and resolutions
3. Without alignment, RGB pixels don't correspond to depth pixels
4. Alignment projects RGB data onto the depth sensor's coordinate frame
"""

import pyrealsense2 as rs
import numpy as np
import cv2
from pathlib import Path
from datetime import datetime
import time


class FrameAligner:
    """
    Handles RGB-to-Depth frame alignment for RealSense camera.
    """
    
    def __init__(self, width=640, height=480, fps=30, output_dir="results/frame_alignment"):
        """
        Initialize the frame aligner.
        
        Args:
            width: Frame width (640 recommended for D435)
            height: Frame height (480 recommended for D435)
            fps: Frames per second
            output_dir: Directory to save results
        """
        self.width = width
        self.height = height
        self.fps = fps
        
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Configure streams - BOTH depth and color
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        
        print(f"Configuring streams: {width}x{height} @ {fps}fps")
        
        # Start pipeline
        print("Starting RealSense camera...")
        self.profile = self.pipeline.start(self.config)
        
        # Create align object - this is the KEY component
        # rs.align aligns frames to a target stream (depth in this case)
        align_to = rs.stream.color  # We can align TO color or TO depth
        self.align = rs.align(align_to)
        
        # Get depth scale
        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        print(f"Depth scale: {self.depth_scale} meters/unit")
        
        # Allow camera to stabilize
        print("Warming up camera (2 seconds)...")
        for _ in range(fps * 2):
            self.pipeline.wait_for_frames()
        
        print("✓ Camera ready!\n")
    
    def get_aligned_frames(self):
        """
        Capture and align RGB and depth frames.
        
        Returns:
            tuple: (aligned_depth_frame, aligned_color_frame, color_image, depth_image, depth_colormap)
        """
        # Wait for frames
        frames = self.pipeline.wait_for_frames()
        
        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)
        
        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        if not aligned_depth_frame or not color_frame:
            return None
        
        # Convert to numpy arrays
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        # Create colorized depth image for visualization
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03),
            cv2.COLORMAP_JET
        )
        
        return aligned_depth_frame, color_frame, color_image, depth_image, depth_colormap
    
    def get_unaligned_frames(self):
        """
        Capture RGB and depth frames WITHOUT alignment (for comparison).
        
        Returns:
            tuple: (depth_frame, color_frame, color_image, depth_image, depth_colormap)
        """
        frames = self.pipeline.wait_for_frames()
        
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            return None
        
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03),
            cv2.COLORMAP_JET
        )
        
        return depth_frame, color_frame, color_image, depth_image, depth_colormap
    
    def demonstrate_alignment_difference(self):
        """
        Show the difference between aligned and unaligned frames.
        This is useful for understanding WHY alignment is important.
        """
        print("="*60)
        print("DEMONSTRATING ALIGNMENT DIFFERENCE")
        print("="*60)
        print("\nCapturing frames...")
        
        # Get unaligned frames
        unaligned_result = self.get_unaligned_frames()
        if unaligned_result is None:
            print("Failed to capture unaligned frames")
            return
        
        _, _, color_unaligned, depth_unaligned, depth_colormap_unaligned = unaligned_result
        
        # Get aligned frames
        aligned_result = self.get_aligned_frames()
        if aligned_result is None:
            print("Failed to capture aligned frames")
            return
        
        _, _, color_aligned, depth_aligned, depth_colormap_aligned = aligned_result
        
        # Create comparison visualization
        print("\nCreating comparison visualization...")
        
        # Stack images side by side
        comparison_color = np.hstack((color_unaligned, color_aligned))
        comparison_depth = np.hstack((depth_colormap_unaligned, depth_colormap_aligned))
        
        # Add labels
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(comparison_color, "Unaligned RGB", (10, 30), font, 0.7, (0, 255, 0), 2)
        cv2.putText(comparison_color, "Aligned RGB", (self.width + 10, 30), font, 0.7, (0, 255, 0), 2)
        cv2.putText(comparison_depth, "Unaligned Depth", (10, 30), font, 0.7, (0, 255, 0), 2)
        cv2.putText(comparison_depth, "Aligned Depth", (self.width + 10, 30), font, 0.7, (0, 255, 0), 2)
        
        # Stack vertically
        final_comparison = np.vstack((comparison_color, comparison_depth))
        
        # Save comparison
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        comparison_path = self.output_dir / f"{timestamp}_alignment_comparison.png"
        cv2.imwrite(str(comparison_path), final_comparison)
        print(f"✓ Saved comparison to: {comparison_path}")
        
        # Display
        cv2.imshow("Alignment Comparison (Press any key to close)", final_comparison)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    def live_aligned_view(self, duration_seconds=10):
        """
        Show live aligned RGB and depth streams.
        
        Args:
            duration_seconds: How long to show the view (0 = until key press)
        """
        print("="*60)
        print("LIVE ALIGNED VIEW")
        print("="*60)
        print(f"Showing aligned frames for {duration_seconds} seconds...")
        print("Press 'q' to quit early, 's' to save frame\n")
        
        start_time = time.time()
        frame_count = 0
        
        try:
            while True:
                # Get aligned frames
                result = self.get_aligned_frames()
                if result is None:
                    continue
                
                aligned_depth_frame, color_frame, color_image, depth_image, depth_colormap = result
                
                # Create side-by-side view
                combined = np.hstack((color_image, depth_colormap))
                
                # Add FPS counter
                frame_count += 1
                elapsed = time.time() - start_time
                fps = frame_count / elapsed if elapsed > 0 else 0
                
                cv2.putText(combined, f"FPS: {fps:.1f}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(combined, "RGB (Aligned)", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(combined, "Depth (Aligned)", (self.width + 10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Show
                cv2.imshow("Live Aligned RGB-D View (Press 'q' to quit, 's' to save)", combined)
                
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    print("\nQuitting...")
                    break
                elif key == ord('s'):
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    save_path = self.output_dir / f"{timestamp}_aligned_frame.png"
                    cv2.imwrite(str(save_path), combined)
                    print(f"✓ Saved frame to: {save_path}")
                
                # Check duration
                if duration_seconds > 0 and elapsed >= duration_seconds:
                    print(f"\n✓ Completed {duration_seconds} second capture")
                    break
        
        finally:
            cv2.destroyAllWindows()
            print(f"\nCaptured {frame_count} frames in {elapsed:.1f} seconds")
            print(f"Average FPS: {fps:.1f}")
    
    def get_depth_at_pixel(self, aligned_depth_frame, x, y):
        """
        Get depth value at specific pixel coordinate.
        Only works correctly with ALIGNED frames!
        
        Args:
            aligned_depth_frame: Aligned depth frame
            x, y: Pixel coordinates
            
        Returns:
            float: Depth in meters
        """
        depth = aligned_depth_frame.get_distance(x, y)
        return depth
    
    def demonstrate_pixel_query(self):
        """
        Demonstrate querying depth at RGB pixel coordinates.
        This only works correctly with aligned frames!
        """
        print("="*60)
        print("PIXEL-DEPTH QUERY DEMONSTRATION")
        print("="*60)
        print("\nClick on the RGB image to query depth at that pixel.")
        print("This demonstrates why alignment is crucial!\n")
        
        # Mouse callback to handle clicks
        click_coords = []
        
        def mouse_callback(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                click_coords.append((x, y))
        
        # Capture frame
        result = self.get_aligned_frames()
        if result is None:
            print("Failed to capture frames")
            return
        
        aligned_depth_frame, color_frame, color_image, depth_image, depth_colormap = result
        
        # Create display image
        display = color_image.copy()
        
        cv2.namedWindow("Click on RGB image to query depth (Press 'q' when done)")
        cv2.setMouseCallback("Click on RGB image to query depth (Press 'q' when done)", mouse_callback)
        
        print("Waiting for clicks...")
        
        while True:
            display_copy = display.copy()
            
            # Draw all clicked points
            for i, (cx, cy) in enumerate(click_coords):
                depth = aligned_depth_frame.get_distance(cx, cy)
                
                # Draw circle
                cv2.circle(display_copy, (cx, cy), 5, (0, 255, 0), -1)
                
                # Draw text
                text = f"#{i+1}: {depth*100:.1f} cm"
                cv2.putText(display_copy, text, (cx + 10, cy - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                print(f"Point #{i+1}: ({cx}, {cy}) -> Depth: {depth*100:.1f} cm")
            
            cv2.imshow("Click on RGB image to query depth (Press 'q' when done)", display_copy)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
        
        cv2.destroyAllWindows()
        
        # Save annotated image
        if click_coords:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            save_path = self.output_dir / f"{timestamp}_depth_query.png"
            cv2.imwrite(str(save_path), display_copy)
            print(f"\n✓ Saved annotated image to: {save_path}")
    
    def create_rgb_depth_overlay(self):
        """
        Create an overlay showing RGB with depth information.
        Useful for object detection where you need both color and depth.
        """
        print("="*60)
        print("RGB-DEPTH OVERLAY")
        print("="*60)
        print("\nCreating overlay with depth edges on RGB image...")
        
        result = self.get_aligned_frames()
        if result is None:
            print("Failed to capture frames")
            return
        
        aligned_depth_frame, color_frame, color_image, depth_image, depth_colormap = result
        
        # Find depth edges (useful for detecting objects)
        depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        edges = cv2.Canny(depth_normalized, 50, 150)
        
        # Create colored edges
        edges_colored = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        edges_colored[edges > 0] = [0, 255, 0]  # Green edges
        
        # Overlay edges on RGB
        overlay = cv2.addWeighted(color_image, 0.7, edges_colored, 0.3, 0)
        
        # Create comparison
        comparison = np.hstack((color_image, overlay, depth_colormap))
        
        # Add labels
        cv2.putText(comparison, "RGB", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(comparison, "RGB + Depth Edges", (self.width + 10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(comparison, "Depth", (self.width * 2 + 10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Save
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        save_path = self.output_dir / f"{timestamp}_rgb_depth_overlay.png"
        cv2.imwrite(str(save_path), comparison)
        print(f"✓ Saved overlay to: {save_path}")
        
        # Display
        cv2.imshow("RGB-Depth Overlay (Press any key to close)", comparison)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    def get_intrinsics(self):
        """
        Get camera intrinsic parameters (useful for 3D reconstruction).
        These parameters are the same after alignment.
        """
        result = self.get_aligned_frames()
        if result is None:
            return None
        
        aligned_depth_frame, color_frame, _, _, _ = result
        
        # Get intrinsics
        color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        depth_intrinsics = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        
        print("\n" + "="*60)
        print("CAMERA INTRINSICS")
        print("="*60)
        print("\nColor Camera Intrinsics:")
        print(f"  Resolution: {color_intrinsics.width} x {color_intrinsics.height}")
        print(f"  Principal Point: ({color_intrinsics.ppx:.2f}, {color_intrinsics.ppy:.2f})")
        print(f"  Focal Length: ({color_intrinsics.fx:.2f}, {color_intrinsics.fy:.2f})")
        print(f"  Distortion Model: {color_intrinsics.model}")
        
        print("\nDepth Camera Intrinsics (After Alignment):")
        print(f"  Resolution: {depth_intrinsics.width} x {depth_intrinsics.height}")
        print(f"  Principal Point: ({depth_intrinsics.ppx:.2f}, {depth_intrinsics.ppy:.2f})")
        print(f"  Focal Length: ({depth_intrinsics.fx:.2f}, {depth_intrinsics.fy:.2f})")
        print(f"  Distortion Model: {depth_intrinsics.model}")
        
        return color_intrinsics, depth_intrinsics
    
    def shutdown(self):
        """Stop the camera pipeline."""
        print("\nShutting down camera...")
        self.pipeline.stop()
        cv2.destroyAllWindows()
        print("✓ Done!")


def main():
    """Main demonstration program."""
    print("="*60)
    print("REALSENSE FRAME ALIGNMENT DEMONSTRATION")
    print("="*60)
    print("\nThis script demonstrates RGB-to-Depth frame alignment.")
    print("Frame alignment is essential for your overhead tracking project")
    print("because it allows you to:")
    print("  1. Combine color information with depth for better detection")
    print("  2. Query depth at any RGB pixel coordinate")
    print("  3. Create accurate 3D point clouds with color")
    print("="*60)
    
    # Initialize aligner
    aligner = FrameAligner(width=640, height=480, fps=30)
    
    try:
        while True:
            print("\n" + "="*60)
            print("DEMONSTRATION MENU")
            print("="*60)
            print("1. Show alignment comparison (aligned vs unaligned)")
            print("2. Live aligned view (10 seconds)")
            print("3. Interactive depth query (click on RGB to get depth)")
            print("4. RGB-Depth overlay (show depth edges on RGB)")
            print("5. Display camera intrinsics")
            print("6. Live view (continuous - press 'q' to stop)")
            print("7. Exit")
            print("="*60)
            
            choice = input("\nEnter choice (1-7): ").strip()
            
            if choice == '1':
                aligner.demonstrate_alignment_difference()
            
            elif choice == '2':
                aligner.live_aligned_view(duration_seconds=10)
            
            elif choice == '3':
                aligner.demonstrate_pixel_query()
            
            elif choice == '4':
                aligner.create_rgb_depth_overlay()
            
            elif choice == '5':
                aligner.get_intrinsics()
            
            elif choice == '6':
                aligner.live_aligned_view(duration_seconds=0)
            
            elif choice == '7':
                break
            
            else:
                print("Invalid choice. Please enter 1-7.")
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
    
    finally:
        aligner.shutdown()
        print("\nThank you for exploring frame alignment!")


if __name__ == "__main__":
    main()
