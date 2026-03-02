"""
Version 1

camera_v1.py - RealSense Camera Base Class
Overhead Perception System
Author: Aaron Fraze
Date: February 23, 2026

Purpose:
    Shared camera foundation for all detector modules.
    Handles pipeline init, stream config, frame capture,
    intrinsics, alignment, and pixel-to-3D deprojection.
    All detectors import this instead of duplicating camera code.

Usage:
    from camera import RealSenseCamera
    cam = RealSenseCamera()
    frame = cam.get_frame()
    point = cam.pixel_to_world(cx, cy, depth_val)
    cam.shutdown()
"""

import sys
import pyrealsense2 as rs
import numpy as np


# ── Resolution presets ────────────────────────────────────────────────────────
# 1280×720  → best accuracy, ~0.19 cm/pixel at 220 cm height, max 30 FPS
# 848×480   → good balance, supports 60 FPS (better for motion blur reduction)
# 640×480   → fastest, lowest resolution
RESOLUTION_PRESETS = {
    '1280x720':  (1280, 720,  30),
    '848x480':   (848,  480,  60),   # Recommended if upgrading for speed
    '640x480':   (640,  480,  30),
}

DEFAULT_RESOLUTION = '1280x720'


class RealSenseCamera:
    """
    Base class for the RealSense D435 overhead camera.

    Provides:
        - Pipeline init with error handling
        - Configurable resolution and framerate
        - Manual exposure control (critical for motion blur reduction)
        - Aligned depth+color frame capture
        - Intrinsic parameter access
        - pixel_to_3d_point()  → camera-frame (x, y, z) in meters
        - pixel_to_world()     → world-frame (x, y, z) in cm using calibration
        - Camera height config for world transform
    """

    def __init__(
        self,
        resolution: str = DEFAULT_RESOLUTION,
        exposure_us: int = None,
        camera_height_cm: float = 220.0,
        warmup_frames: int = 90,
    ):
        """
        Initialize the RealSense camera.

        Args:
            resolution:        One of '1280x720', '848x480', '640x480'.
                               Use '848x480' to unlock 60 FPS for faster motion.
            exposure_us:       Color sensor exposure in microseconds.
                               None → auto exposure (default).
                               Recommended range for motion: 3000–8000.
                               Lower = less blur, darker image.
                               Start at 6000 and tune down if still blurry.
            camera_height_cm:  Physical height of camera lens above ground (cm).
                               Used in world coordinate transform. Must match
                               your physical mount. Default: 220 cm.
            warmup_frames:     Frames to discard on startup for sensor stabilization.
                               90 frames = 3 seconds at 30 FPS.
        """

        # ── Store config ──────────────────────────────────────────────────────
        if resolution not in RESOLUTION_PRESETS:
            print(f"[Camera] Unknown resolution '{resolution}', using {DEFAULT_RESOLUTION}")
            resolution = DEFAULT_RESOLUTION

        self.resolution_name = resolution
        self.width, self.height, self.fps = RESOLUTION_PRESETS[resolution]
        self.exposure_us = exposure_us
        self.camera_height_m = camera_height_cm / 100.0

        # World transform: camera at (0, 0, height), pointing straight down.
        # Camera frame:  +X right, +Y down, +Z into scene (toward floor)
        # World frame:   +X right, +Y forward, +Z up
        # Transform:     x_world = x_cam
        #                y_world = -y_cam
        #                z_world = -z_cam + camera_height
        self.camera_height_cm = camera_height_cm

        # ── Intrinsics (populated after first frame) ──────────────────────────
        self.depth_intrinsics = None
        self.color_intrinsics = None
        self.camera_matrix = None   # 3×3 for OpenCV pose estimation
        self.dist_coeffs = None

        # ── Pipeline init ─────────────────────────────────────────────────────
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(
            rs.stream.depth, self.width, self.height, rs.format.z16, self.fps
        )
        self.config.enable_stream(
            rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps
        )

        print(f"[Camera] Starting pipeline  {self.width}×{self.height} @ {self.fps} FPS...")

        try:
            self.profile = self.pipeline.start(self.config)
        except RuntimeError as e:
            if "No device connected" in str(e):
                print("[Camera] ERROR: No RealSense device found.")
                print("         Check USB connection and try again.")
            else:
                print(f"[Camera] ERROR: {e}")
            sys.exit(1)
        except Exception as e:
            print(f"[Camera] ERROR: {e}")
            sys.exit(1)

        # ── Sensors ───────────────────────────────────────────────────────────
        self.device = self.profile.get_device()
        self.depth_sensor = self.device.first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()

        # Color sensor — needed for exposure control
        self.color_sensor = None
        for sensor in self.device.sensors:
            if sensor.is_color_sensor():
                self.color_sensor = sensor
                break

        # ── Alignment (depth → color frame) ──────────────────────────────────
        self.align = rs.align(rs.stream.color)

        # ── Exposure ──────────────────────────────────────────────────────────
        if self.exposure_us is not None:
            self._apply_exposure(self.exposure_us)
        else:
            print("[Camera] Exposure: AUTO")

        # ── Warmup ────────────────────────────────────────────────────────────
        print(f"[Camera] Warming up ({warmup_frames} frames)...")
        for _ in range(warmup_frames):
            self.pipeline.wait_for_frames()

        print(f"[Camera] Ready.  Depth scale: {self.depth_scale:.4f} m/unit")

    # ── Exposure control ──────────────────────────────────────────────────────

    def _apply_exposure(self, exposure_us: int):
        """
        Set manual exposure on the color sensor.

        The color sensor must have auto-exposure disabled before setting
        a fixed value. Calling this switches to manual mode.

        Args:
            exposure_us: Exposure time in microseconds. 
                         Typical range: 1 – 166000.
                         For motion blur reduction: 3000–8000.
        """
        if self.color_sensor is None:
            print("[Camera] WARNING: Could not find color sensor for exposure control.")
            return

        try:
            # Disable auto-exposure first
            self.color_sensor.set_option(rs.option.enable_auto_exposure, 0)
            # Set manual value
            self.color_sensor.set_option(rs.option.exposure, float(exposure_us))
            print(f"[Camera] Exposure: MANUAL  {exposure_us} µs")
        except Exception as e:
            print(f"[Camera] WARNING: Could not set exposure: {e}")
            print("[Camera] Falling back to auto exposure.")

    def set_exposure(self, exposure_us: int):
        """
        Change exposure at runtime (e.g., during a live demo).

        Args:
            exposure_us: New exposure time in microseconds.
                         Pass 0 to re-enable auto exposure.
        """
        if exposure_us == 0:
            if self.color_sensor:
                self.color_sensor.set_option(rs.option.enable_auto_exposure, 1)
                print("[Camera] Exposure: AUTO (restored)")
            self.exposure_us = None
        else:
            self._apply_exposure(exposure_us)
            self.exposure_us = exposure_us

    def get_exposure_range(self):
        """
        Return the supported exposure range for the connected camera.

        Returns:
            (min_us, max_us, step_us) or None if unavailable.
        """
        if self.color_sensor is None:
            return None
        try:
            r = self.color_sensor.get_option_range(rs.option.exposure)
            return (r.min, r.max, r.step)
        except Exception:
            return None

    # ── Frame capture ─────────────────────────────────────────────────────────

    def get_frame(self) -> dict | None:
        """
        Capture one aligned depth+color frame pair.

        Intrinsics and camera matrix are populated on the first successful call
        and reused for all subsequent calls (they don't change per frame).

        Returns:
            dict with keys:
                'color_image'  : np.ndarray (H×W×3, BGR, uint8)
                'depth_image'  : np.ndarray (H×W, uint16, raw depth units)
                'color_frame'  : rs.frame  (for advanced SDK use)
                'depth_frame'  : rs.frame  (for advanced SDK use)
            None if frames could not be retrieved.
        """
        try:
            frames = self.pipeline.wait_for_frames()
        except RuntimeError:
            return None

        aligned = self.align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()

        if not depth_frame or not color_frame:
            return None

        # Populate intrinsics once
        if self.depth_intrinsics is None:
            self._init_intrinsics(depth_frame, color_frame)

        return {
            'color_image': np.asanyarray(color_frame.get_data()),
            'depth_image': np.asanyarray(depth_frame.get_data()),
            'color_frame': color_frame,
            'depth_frame': depth_frame,
        }

    def _init_intrinsics(self, depth_frame, color_frame):
        """Populate intrinsic parameters from the first live frame."""
        self.depth_intrinsics = (
            depth_frame.profile.as_video_stream_profile().intrinsics
        )
        self.color_intrinsics = (
            color_frame.profile.as_video_stream_profile().intrinsics
        )
        # OpenCV-format camera matrix (used by solvePnP / drawFrameAxes)
        ci = self.color_intrinsics
        self.camera_matrix = np.array([
            [ci.fx,  0.0,  ci.ppx],
            [0.0,  ci.fy,  ci.ppy],
            [0.0,   0.0,    1.0  ],
        ], dtype=np.float64)
        self.dist_coeffs = np.array(ci.coeffs, dtype=np.float64)

        print(
            f"[Camera] Intrinsics loaded  "
            f"fx={ci.fx:.1f}  fy={ci.fy:.1f}  "
            f"ppx={ci.ppx:.1f}  ppy={ci.ppy:.1f}"
        )

    # ── Coordinate transforms ─────────────────────────────────────────────────

    def pixel_to_3d_point(self, pixel_x: int, pixel_y: int, depth_value: int):
        """
        Deproject a pixel + raw depth value to a 3D point in the camera frame.

        The result is in the RealSense camera coordinate system:
            +X → right, +Y → down, +Z → into the scene (toward floor)
        Units are meters.

        Args:
            pixel_x:     Column index (0 = left edge)
            pixel_y:     Row index    (0 = top edge)
            depth_value: Raw uint16 depth value from depth_image[y, x]

        Returns:
            (x, y, z) tuple in meters (camera frame), or None if depth is zero.
        """
        if self.depth_intrinsics is None or depth_value == 0:
            return None

        depth_m = depth_value * self.depth_scale
        return rs.rs2_deproject_pixel_to_point(
            self.depth_intrinsics, [pixel_x, pixel_y], depth_m
        )

    def pixel_to_world(self, pixel_x: int, pixel_y: int, depth_value: int):
        """
        Convert a pixel + raw depth value to world coordinates in cm.

        World coordinate frame (matches your calibration report):
            Origin : center of workspace at ground level
            +X     : right
            +Y     : forward (away from camera mount side)
            +Z     : up (away from floor)

        The camera is assumed to be directly overhead, pointing straight down,
        centered over the workspace at height = camera_height_cm.

        Args:
            pixel_x:     Column index
            pixel_y:     Row index
            depth_value: Raw uint16 depth value from depth_image[y, x]

        Returns:
            (x_cm, y_cm, z_cm) tuple in cm (world frame), or None if depth is zero.
        """
        cam_point = self.pixel_to_3d_point(pixel_x, pixel_y, depth_value)
        if cam_point is None:
            return None

        x_cam, y_cam, z_cam = cam_point  # meters, camera frame

        # Apply calibration transform (matches Calibration_Report_Final.md):
        #   x_world =  x_cam
        #   y_world = -y_cam   (camera Y points down → world Y points forward)
        #   z_world = -z_cam + camera_height   (camera Z points into scene)
        x_world_cm = x_cam * 100.0
        y_world_cm = -y_cam * 100.0
        z_world_cm = (-z_cam + self.camera_height_m) * 100.0

        return (x_world_cm, y_world_cm, z_world_cm)

    # ── Camera info ───────────────────────────────────────────────────────────

    def print_info(self):
        """Print a summary of camera configuration to the console."""
        print("\n" + "=" * 50)
        print("RealSense Camera Configuration")
        print("=" * 50)
        print(f"  Resolution  : {self.width}×{self.height} @ {self.fps} FPS")
        print(f"  Depth scale : {self.depth_scale:.4f} m/unit")
        print(f"  Camera height: {self.camera_height_cm:.1f} cm")

        exp_range = self.get_exposure_range()
        if exp_range:
            print(f"  Exposure range: {exp_range[0]:.0f} – {exp_range[1]:.0f} µs")
        if self.exposure_us:
            print(f"  Current exposure: {self.exposure_us} µs (manual)")
        else:
            print(f"  Current exposure: auto")

        if self.color_intrinsics:
            ci = self.color_intrinsics
            print(f"  fx={ci.fx:.2f}  fy={ci.fy:.2f}  ppx={ci.ppx:.2f}  ppy={ci.ppy:.2f}")
        print("=" * 50 + "\n")

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def shutdown(self):
        """Stop the RealSense pipeline cleanly."""
        print("[Camera] Shutting down...")
        self.pipeline.stop()
        print("[Camera] Done.")


# ── Standalone test ───────────────────────────────────────────────────────────
# Run this file directly to verify camera init and frame capture:
#   python camera.py

if __name__ == "__main__":
    import cv2

    print("=" * 50)
    print("camera.py — Standalone Test")
    print("=" * 50)
    print("Testing camera init and frame capture.")
    print("Press 'e' to cycle through exposure presets.")
    print("Press 'i' to print camera info.")
    print("Press 'q' to quit.\n")

    # Exposure presets to cycle through (useful for finding your sweet spot)
    EXPOSURE_PRESETS = [None, 8000, 6000, 4000, 3000]
    exposure_idx = 0
    exposure_labels = ["AUTO", "8000µs", "6000µs", "4000µs", "3000µs"]

    cam = RealSenseCamera(resolution='1280x720', exposure_us=None)
    cam.print_info()

    import time
    frame_count = 0
    fps_timer = time.time()
    fps_display = 0.0

    while True:
        frame_data = cam.get_frame()
        if frame_data is None:
            continue

        frame_count += 1
        elapsed = time.time() - fps_timer
        if elapsed >= 1.0:
            fps_display = frame_count / elapsed
            frame_count = 0
            fps_timer = time.time()

        vis = frame_data['color_image'].copy()

        # Overlay info
        exp_label = exposure_labels[exposure_idx]
        cv2.putText(vis, f"FPS: {fps_display:.1f}  Exposure: {exp_label}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(vis, "e=cycle exposure  i=info  q=quit",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        cv2.imshow("camera.py test", vis)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('i'):
            cam.print_info()
        elif key == ord('e'):
            exposure_idx = (exposure_idx + 1) % len(EXPOSURE_PRESETS)
            new_exp = EXPOSURE_PRESETS[exposure_idx]
            cam.set_exposure(new_exp if new_exp is not None else 0)

    cv2.destroyAllWindows()
    cam.shutdown()
