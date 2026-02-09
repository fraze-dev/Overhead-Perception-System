"""
Overhead Perception System version 1
Author: Aaron Fraze
Date: February 9, 2026
Purpose: Build Overhead Camera class. Demonstrate world coordinate system transformation
"""

import pyrealsense2 as rs
import numpy as np
import cv2


resolution_width = 1280
resolution_height = 720


class OverheadPerceptor:


    def __init__(self):

        # Initialize d435i camera
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Configure color and depth streams - resolution_width x resolution_height
        self.config.enable_stream(rs.stream.depth, resolution_width, resolution_height, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, resolution_width, resolution_height, rs.format.bgr8, 30)

        # Start pipeline
        self.profile = self.pipeline.start(self.config)

        # Get device and sensors
        self.device = self.profile.get_device()
        self.depth_sensor = self.device.first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()

        # Get intrinsics (will populate after first frame)
        self.depth_intrinsics = None
        self.color_intrinsics = None

        # Create alignment object (align depth to color)
        self.align = rs.align(rs.stream.color)

        # Allow camera to warm up
        print("Warming up camera for 3 seconds")
        for _ in range (90):
            self.pipeline.wait_for_frames()
        print("Ready!")

    def frame_getter(self):
        """
        Capture and process frames.
        Returns:
            dict with color_image, depth_image, depth_frame, color_frame
        """

        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            return None

        # Store intrinsics on first frame
        if self.depth_intrinsics is None:
            self.depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
            self.color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

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


    def coordinate_transformation(self):

        print("\n")
        print("="*60)
        print("Click in image to view world coordinates")
        print("Red border defines area of best accuracy")
        print("Red crosshairs are center of image")
        print(f"Current resolution: {resolution_width}x{resolution_height}")
        print("="*60)

        # Mouse callback
        clicked_point = {'x': None, 'y': None, 'new': False, 'counter': 0}

        def mouse_callback(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                clicked_point['x'] = x
                clicked_point['y'] = y
                clicked_point['new'] = True
                clicked_point['counter'] += 1

        cv2.namedWindow('World Coordinates')
        cv2.setMouseCallback('World Coordinates', mouse_callback)

        while True:

            frames_data = self.frame_getter()
            if frames_data is None:
                continue

            depth_image = frames_data['depth_image']
            color_image = frames_data['color_image']

            # Copy image for visualization
            vis = color_image.copy()

            # Show coordinates when image clicked
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

                    # Set text of 3d point
                    text = f"3D: ({point_3d[0] * 100:.1f}, {point_3d[1] * -100:.1f}, {point_3d[2] * 100:.1f}) cm"

                    # Calculate text size
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    font_scale = 0.5
                    thickness = 2
                    (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)

                    # Get image dimensions
                    img_height, img_width = vis.shape[:2]

                    # Smart text to avoid going off-screen
                    margin = 10  # Pixels margin from click point

                    # Horizontal position (left or right of click)
                    if px + margin + text_width > img_width:
                        # Too close to right edge - place text on LEFT
                        text_x = max(0, px - text_width - margin)
                    else:
                        # Normal - place text on RIGHT
                        text_x = px + margin

                    # Vertical position (above or below click)
                    if py - margin < text_height:
                        # Too close to top edge - place text BELOW
                        text_y = py + text_height + margin
                    else:
                        # Normal - place text ABOVE
                        text_y = py - margin

                    # Draw text at calculated position
                    cv2.putText(vis, text, (text_x, text_y), font, font_scale, (0, 255, 0), thickness)

                    # Print to console once per user click
                    if clicked_point['new']:
                        print(f"\nClicked Point# {clicked_point['counter']}: Pixel ({px}, {py}) -> 3D Point: "
                            f"X={point_3d[0] * 100:.2f} cm, "
                            f"Y={point_3d[1] * 100:.2f} cm, "
                            f"Z={point_3d[2] * 100:.2f} cm")
                        clicked_point['new'] = False

            # Show FPS
            cv2.putText(vis, "Click to measure 3D coordinates | 'q' quit",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Show center of image
            center_x = int(resolution_width/2)
            center_y = int(resolution_height/2)
            cv2.drawMarker(vis, (center_x, center_y), (0, 0, 255),
                           cv2.MARKER_CROSS, 20, 2)

            # Show rectangle of best accuracy
            start_pt1 = (int(resolution_width * .1), int(resolution_height * .1))
            end_pt1 = (int(resolution_width * .9), int(resolution_height * .9))
            cv2.rectangle(vis, start_pt1, end_pt1, (0, 0, 255), 2)


            cv2.imshow('World Coordinates', vis)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

        cv2.destroyAllWindows()

    def shutdown(self):
        """Stop camera pipeline."""
        print("\nShutting down camera...")
        self.pipeline.stop()
        print("Done!")


if __name__=="__main__":
    print("="*60)
    print("Overhead Perception System")
    print("="*60)

    perceptor = OverheadPerceptor()

    try:
        perceptor.coordinate_transformation()

    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")

    finally:
        perceptor.shutdown()
        print("Program ended.\n")

