"""
Minimal Frame Alignment Example
Author: Aaron Fraze
Date: January 24, 2026

This is the SIMPLEST possible example of frame alignment.
Use this to quickly test that alignment is working on your system.
"""

import pyrealsense2 as rs
import numpy as np
import cv2

def main():
    print("Starting minimal frame alignment test...")
    
    # Configure and start pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Enable both streams
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    print("Starting camera...")
    pipeline.start(config)
    
    # Create alignment object
    align = rs.align(rs.stream.color)  # Align depth TO color
    
    print("Camera started. Showing aligned frames...")
    print("Press 'q' to quit, 's' to save a frame")
    
    try:
        frame_count = 0
        while True:
            # Wait for frames
            frames = pipeline.wait_for_frames()
            
            # ALIGN THE FRAMES - This is the important step!
            aligned_frames = align.process(frames)
            
            # Get aligned depth and color frames
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not aligned_depth_frame or not color_frame:
                continue
            
            # Convert to numpy arrays
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            # Create colorized depth for visualization
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03),
                cv2.COLORMAP_JET
            )
            
            # Stack side by side
            images = np.hstack((color_image, depth_colormap))
            
            # Add text
            cv2.putText(images, "RGB (Aligned)", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(images, "Depth (Aligned)", (650, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(images, f"Frame: {frame_count}", (10, 460),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Show
            cv2.imshow('Aligned RGB-D (q=quit, s=save)', images)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                filename = f'aligned_frame_{frame_count:04d}.png'
                cv2.imwrite(filename, images)
                print(f"Saved: {filename}")
            
            frame_count += 1
    
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print(f"\nProcessed {frame_count} frames")
        print("Done!")

if __name__ == "__main__":
    main()
