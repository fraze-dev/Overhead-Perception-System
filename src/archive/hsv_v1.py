"""
Overhead Perception System version 2
Author: Aaron Fraze
Date: February 11, 2026
"""
"""
Purpose: Build Overhead Camera class with HSV color tracking for ball detection
"""
import sys
import pyrealsense2 as rs
import numpy as np
import cv2

"""SET DESIRED RESOLUTION"""
"""Suggested: 640x480, 848x480, 1280x720"""
resolution_width = 1280
resolution_height = 720


class OverheadPerceptor:

    def __init__(self):
        # Initialize d435i camera
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Configure color and depth streams
        self.config.enable_stream(rs.stream.depth, resolution_width, resolution_height, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, resolution_width, resolution_height, rs.format.bgr8, 30)

        try:
            self.profile = self.pipeline.start(self.config)
            print("Camera initialized")
        except RuntimeError as e:
            if "No device connected" in str(e):
                print("Camera not found.")
                print("Check if camera is connected and try again.")
                print("Closing program...")
                sys.exit(1)
            else:
                print(f"Camera runtime error: {e}")
                print("Closing program...")
                sys.exit(1)
        except Exception as e:
            print(f"Camera not initialized. Error: {e}")
            print("Closing program...")
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

        # HSV color ranges (these are initial defaults - you'll tune these)
        self.hsv_ranges = {
            'orange_ball': {
                'lower': np.array([2,1,82]),  # Lower HSV bound
                'upper': np.array([7,255,239]),  # Upper HSV bound
                'min_area': 340,  # Minimum contour area (pixels²)
                'max_area': 700,  # Maximum contour area (pixels²)
            },
            'green_ball': {
                'lower': np.array([30, 50, 50]),
                'upper': np.array([50, 255, 255]),
                'min_area': 100,
                'max_area': 50000,
            }
        }

        # Current tracking target
        self.current_target = 'orange_ball'

        # Allow camera to warm up
        print("Warming up camera for 3 seconds")
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

        point_3d = rs.rs2_deproject_pixel_to_point(
            self.depth_intrinsics,
            [pixel_x, pixel_y],
            depth_m
        )

        return point_3d

    def detect_ball_hsv(self, color_image, depth_image, target_name='orange_ball'):
        """
        Detect ball using HSV color masking.

        Args:
            color_image: BGR color image
            depth_image: Depth image (uint16)
            target_name: Name of target from hsv_ranges dict

        Returns:
            dict with detection info or None if not detected
        """
        # Get HSV range for target
        if target_name not in self.hsv_ranges:
            print(f"Unknown target: {target_name}")
            return None

        target_config = self.hsv_ranges[target_name]

        # Convert BGR to HSV
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # Create mask
        mask = cv2.inRange(hsv, target_config['lower'], target_config['upper'])

        # Morphological operations to reduce noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # Remove small noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # Fill small holes

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        # Filter by area and find largest valid contour
        valid_contours = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if target_config['min_area'] <= area <= target_config['max_area']:
                valid_contours.append((cnt, area))

        if not valid_contours:
            return None

        # Get largest valid contour
        best_contour, best_area = max(valid_contours, key=lambda x: x[1])

        # Get centroid
        M = cv2.moments(best_contour)
        if M['m00'] == 0:
            return None

        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        # Get enclosing circle
        (circle_x, circle_y), radius = cv2.minEnclosingCircle(best_contour)

        # Get depth at centroid
        depth_value = depth_image[cy, cx]

        # Get 3D position if depth is valid
        point_3d = None
        if depth_value > 0:
            point_3d = self.pixel_to_3d_point(cx, cy, depth_value)

        return {
            'detected': True,
            'centroid_pixel': (cx, cy),
            'circle_center': (int(circle_x), int(circle_y)),
            'radius': int(radius),
            'area': best_area,
            'contour': best_contour,
            'depth_value': depth_value,
            'point_3d': point_3d,
            'mask': mask
        }

    def hsv_tuning_mode(self):
        """
        Interactive HSV tuning interface with trackbars.
        Allows real-time adjustment of HSV thresholds.
        """
        print("\n" + "=" * 60)
        print("HSV TUNING MODE")
        print("=" * 60)
        print("Use trackbars to adjust HSV ranges")
        print("Press 's' to save current values")
        print("Press '1' for orange ball, '2' for green ball")
        print("Press 'q' to quit")
        print("=" * 60)

        # Create windows
        cv2.namedWindow('HSV Tuning')
        cv2.namedWindow('Mask')
        cv2.namedWindow('Result')

        # Get current target config
        config = self.hsv_ranges[self.current_target]

        # Create trackbars
        cv2.createTrackbar('H Lower', 'HSV Tuning', config['lower'][0], 179, lambda x: None)
        cv2.createTrackbar('S Lower', 'HSV Tuning', config['lower'][1], 255, lambda x: None)
        cv2.createTrackbar('V Lower', 'HSV Tuning', config['lower'][2], 255, lambda x: None)
        cv2.createTrackbar('H Upper', 'HSV Tuning', config['upper'][0], 179, lambda x: None)
        cv2.createTrackbar('S Upper', 'HSV Tuning', config['upper'][1], 255, lambda x: None)
        cv2.createTrackbar('V Upper', 'HSV Tuning', config['upper'][2], 255, lambda x: None)
        cv2.createTrackbar('Min Area', 'HSV Tuning', config['min_area'], 10000, lambda x: None)
        cv2.createTrackbar('Max Area', 'HSV Tuning', config['max_area'], 100000, lambda x: None)

        while True:
            frames_data = self.get_frame()
            if frames_data is None:
                continue

            color_image = frames_data['color_image']
            depth_image = frames_data['depth_image']

            # Get trackbar values
            h_lower = cv2.getTrackbarPos('H Lower', 'HSV Tuning')
            s_lower = cv2.getTrackbarPos('S Lower', 'HSV Tuning')
            v_lower = cv2.getTrackbarPos('V Lower', 'HSV Tuning')
            h_upper = cv2.getTrackbarPos('H Upper', 'HSV Tuning')
            s_upper = cv2.getTrackbarPos('S Upper', 'HSV Tuning')
            v_upper = cv2.getTrackbarPos('V Upper', 'HSV Tuning')
            min_area = cv2.getTrackbarPos('Min Area', 'HSV Tuning')
            max_area = cv2.getTrackbarPos('Max Area', 'HSV Tuning')

            # Update config temporarily
            self.hsv_ranges[self.current_target]['lower'] = np.array([h_lower, s_lower, v_lower])
            self.hsv_ranges[self.current_target]['upper'] = np.array([h_upper, s_upper, v_upper])
            self.hsv_ranges[self.current_target]['min_area'] = min_area
            self.hsv_ranges[self.current_target]['max_area'] = max_area

            # Detect with current settings
            detection = self.detect_ball_hsv(color_image, depth_image, self.current_target)

            # Visualize
            result = color_image.copy()

            if detection:
                # Draw detection
                cx, cy = detection['centroid_pixel']
                circle_center = detection['circle_center']
                radius = detection['radius']

                # Draw circle
                cv2.circle(result, circle_center, radius, (0, 255, 0), 2)
                cv2.circle(result, (cx, cy), 5, (0, 0, 255), -1)

                # Draw info
                if detection['point_3d']:
                    p3d = detection['point_3d']
                    text = f"3D: ({p3d[0] * 100:.1f}, {p3d[1] * -100:.1f}, {p3d[2] * 100:.1f}) cm"
                    cv2.putText(result, text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (0, 255, 0), 2)

                area_text = f"Area: {detection['area']:.0f} px^2"
                cv2.putText(result, area_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 0), 2)

            # Status text
            status_text = f"Tuning: {self.current_target} | 's' save | '1'/'2' switch | 'q' quit"
            cv2.putText(result, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 255, 255), 2)

            # Show windows
            cv2.imshow('HSV Tuning', color_image)
            cv2.imshow('Mask', detection['mask'] if detection else np.zeros_like(depth_image))
            cv2.imshow('Result', result)

            # Handle keys
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            elif key == ord('s'):
                print(f"\nSaved HSV values for {self.current_target}:")
                print(f"  Lower: {self.hsv_ranges[self.current_target]['lower']}")
                print(f"  Upper: {self.hsv_ranges[self.current_target]['upper']}")
                print(f"  Min Area: {min_area}")
                print(f"  Max Area: {max_area}")
            elif key == ord('1'):
                self.current_target = 'orange_ball'
                print(f"\nSwitched to: {self.current_target}")
                config = self.hsv_ranges[self.current_target]
                cv2.setTrackbarPos('H Lower', 'HSV Tuning', config['lower'][0])
                cv2.setTrackbarPos('S Lower', 'HSV Tuning', config['lower'][1])
                cv2.setTrackbarPos('V Lower', 'HSV Tuning', config['lower'][2])
                cv2.setTrackbarPos('H Upper', 'HSV Tuning', config['upper'][0])
                cv2.setTrackbarPos('S Upper', 'HSV Tuning', config['upper'][1])
                cv2.setTrackbarPos('V Upper', 'HSV Tuning', config['upper'][2])
            elif key == ord('2'):
                self.current_target = 'green_ball'
                print(f"\nSwitched to: {self.current_target}")
                config = self.hsv_ranges[self.current_target]
                cv2.setTrackbarPos('H Lower', 'HSV Tuning', config['lower'][0])
                cv2.setTrackbarPos('S Lower', 'HSV Tuning', config['lower'][1])
                cv2.setTrackbarPos('V Lower', 'HSV Tuning', config['lower'][2])
                cv2.setTrackbarPos('H Upper', 'HSV Tuning', config['upper'][0])
                cv2.setTrackbarPos('S Upper', 'HSV Tuning', config['upper'][1])
                cv2.setTrackbarPos('V Upper', 'HSV Tuning', config['upper'][2])

        cv2.destroyAllWindows()

    def ball_tracking_demo(self):
        """
        Demonstrate ball tracking with live visualization.
        """
        print("\n" + "=" * 60)
        print("BALL TRACKING DEMO")
        print("=" * 60)
        print("Tracking orange ball in real-time")
        print("Press 'q' to quit")
        print("=" * 60)

        cv2.namedWindow('Ball Tracking')

        while True:
            frames_data = self.get_frame()
            if frames_data is None:
                continue

            color_image = frames_data['color_image']
            depth_image = frames_data['depth_image']

            # Detect ball
            detection = self.detect_ball_hsv(color_image, depth_image, 'orange_ball')

            # Visualize
            vis = color_image.copy()

            if detection:
                # Draw circle and centroid
                circle_center = detection['circle_center']
                radius = detection['radius']
                cx, cy = detection['centroid_pixel']

                cv2.circle(vis, circle_center, radius, (0, 255, 0), 3)
                cv2.circle(vis, (cx, cy), 5, (255, 0, 0), -1)

                # Draw crosshair
                cv2.line(vis, (cx - 20, cy), (cx + 20, cy), (255, 0, 0), 2)
                cv2.line(vis, (cx, cy - 20), (cx, cy + 20), (255, 0, 0), 2)

                # Display 3D coordinates
                if detection['point_3d']:
                    p3d = detection['point_3d']
                    coord_text = f"World Position: ({p3d[0] * 100:.1f}, {p3d[1] * -100:.1f}, {p3d[2] * 100:.1f}) cm"
                    cv2.putText(vis, coord_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, (0, 255, 0), 2)

                    depth_text = f"Depth: {p3d[2] * 100:.1f} cm"
                    cv2.putText(vis, depth_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, (0, 255, 0), 2)

                status_text = "BALL DETECTED"
                color = (0, 255, 0)
            else:
                status_text = "SEARCHING..."
                color = (0, 0, 255)

            cv2.putText(vis, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, color, 2)

            cv2.imshow('Ball Tracking', vis)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

        cv2.destroyAllWindows()

    def coordinate_transformation(self):
        """Original coordinate transformation demo"""
        print("\n" + "=" * 60)
        print("Click in image to view world coordinates")
        print("Red border defines area of best accuracy")
        print("Red crosshairs are center of image")
        print(f"Current resolution: {resolution_width}x{resolution_height}")
        print("=" * 60)

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
            frames_data = self.get_frame()
            if frames_data is None:
                continue

            depth_image = frames_data['depth_image']
            color_image = frames_data['color_image']
            vis = color_image.copy()

            if clicked_point['x'] is not None:
                px, py = clicked_point['x'], clicked_point['y']
                depth_val = depth_image[py, px]

                if depth_val > 0:
                    point_3d = self.pixel_to_3d_point(px, py, depth_val)
                    cv2.drawMarker(vis, (px, py), (0, 255, 0),
                                   cv2.MARKER_CROSS, 20, 2)

                    text = f"3D: ({point_3d[0] * 100:.1f}, {point_3d[1] * -100:.1f}, {point_3d[2] * 100:.1f}) cm"
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    font_scale = 0.5
                    thickness = 2
                    (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)

                    img_height, img_width = vis.shape[:2]
                    margin = 10

                    if px + margin + text_width > img_width:
                        text_x = max(0, px - text_width - margin)
                    else:
                        text_x = px + margin

                    if py - margin < text_height:
                        text_y = py + text_height + margin
                    else:
                        text_y = py - margin

                    cv2.putText(vis, text, (text_x, text_y), font, font_scale, (0, 255, 0), thickness)

                    if clicked_point['new']:
                        print(f"\nClicked Point# {clicked_point['counter']}: Pixel ({px}, {py}) -> 3D Point: "
                              f"X={point_3d[0] * 100:.2f} cm, "
                              f"Y={point_3d[1] * 100:.2f} cm, "
                              f"Z={point_3d[2] * 100:.2f} cm")
                        clicked_point['new'] = False

            cv2.putText(vis, "Click to measure 3D coordinates | 'q' quit",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            center_x = int(resolution_width / 2)
            center_y = int(resolution_height / 2)
            cv2.drawMarker(vis, (center_x, center_y), (0, 0, 255),
                           cv2.MARKER_CROSS, 20, 2)

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


if __name__ == "__main__":
    print("=" * 60)
    print("Overhead Perception System - HSV Ball Tracking")
    print("=" * 60)

    print("Initializing camera...")
    perceptor = OverheadPerceptor()

    try:
        print("\n" + "=" * 60)
        print("DEMO MENU")
        print("=" * 60)
        print("1. Coordinate Transformation (original demo)")
        print("2. HSV Tuning Mode (adjust color thresholds)")
        print("3. Ball Tracking Demo (real-time tracking)")
        print("=" * 60)

        choice = input("\nEnter choice (1-3): ").strip()

        if choice == '1':
            perceptor.coordinate_transformation()
        elif choice == '2':
            perceptor.hsv_tuning_mode()
        elif choice == '3':
            perceptor.ball_tracking_demo()
        else:
            print("Invalid choice. Running coordinate transformation...")
            perceptor.coordinate_transformation()

    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")

    finally:
        perceptor.shutdown()
        print("Program ended.\n")