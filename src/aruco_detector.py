"""
aruco_detector.py - ArUco Marker Detector for Overhead Robot Tracking
Overhead Perception System
Author: Aaron Fraze
Date: February 23, 2026

Purpose:
    Detects ArUco markers from overhead and returns robot position + heading.
    Key improvement over aruco_detection.py:
        - Manual exposure control to eliminate motion blur at speed
        - Optimized DetectorParameters for blur robustness
        - DICT_4X4_50 default (fewer cells = more robust under motion)
        - Per-detection confidence score
        - Heading angle extracted from marker corners (no solvePnP needed
          for overhead-only use case — saves ~2ms per frame)
        - Clean separation from camera code via RealSenseCamera base class

Usage:
    # Standalone live demo:
    python aruco_detector.py

    # Import in other modules:
    from aruco_detector import ArucoDetector
    detector = ArucoDetector(cam, marker_size_cm=8.255)
    markers = detector.detect(frame_data)
"""

import time
import numpy as np
import cv2
from cv2 import aruco

from camera import RealSenseCamera


# ── Marker size constants (for reference) ────────────────────────────────────
MARKER_2_125_INCH_CM = 5.40   # 2 1/8" marker
MARKER_3_25_INCH_CM  = 8.255  # 3 1/4" marker  ← use this one on HamBot


class ArucoDetector:
    """
    Overhead ArUco marker detector optimized for moving robots.

    Key design decisions vs. the original aruco_detection.py:
    
    1. Uses DICT_4X4_50 by default.
       4×4 markers have larger cells per pixel at a given physical size,
       making them significantly more robust to motion blur than 6×6 or 7×7.
       You only need one unique ID for HamBot, so 50 IDs is plenty.

    2. Tuned DetectorParameters for motion robustness:
       - Wider adaptive threshold window range catches blurred edges
       - Disabled corner subpixel refinement (adds ~1ms, hurts blurry corners)
       - Relaxed error correction for partial decodes under motion

    3. Heading from corners (not solvePnP).
       From directly overhead, heading = angle of the marker's top edge
       in world X/Y. This is faster than solvePnP and equally accurate
       for a camera pointing straight down.

    4. Confidence score per detection.
       Based on: corner regularity, depth validity, and perimeter size.
       Lets the hybrid detector (world_state.py) decide when to trust ArUco
       vs. fall back to HSV.
    """

    def __init__(
        self,
        camera: RealSenseCamera,
        marker_size_cm: float = MARKER_3_25_INCH_CM,
        aruco_dict_type=aruco.DICT_4X4_50,
    ):
        """
        Args:
            camera:          Initialized RealSenseCamera instance.
                             Exposure should be set before passing in.
            marker_size_cm:  Physical side length of your printed marker in cm.
                             Use MARKER_3_25_INCH_CM (8.255) for the large marker.
            aruco_dict_type: ArUco dictionary. DICT_4X4_50 recommended for speed.
                             Switch to DICT_4X4_100 if you ever need more IDs.
        """
        self.cam = camera
        self.marker_size_cm = marker_size_cm
        self.marker_size_m = marker_size_cm / 100.0

        # ── ArUco dictionary ──────────────────────────────────────────────────
        self.aruco_dict_type = aruco_dict_type
        self.aruco_dict = aruco.getPredefinedDictionary(aruco_dict_type)

        # ── Detector parameters tuned for motion robustness ───────────────────
        #
        # The default DetectorParameters work well for static markers.
        # For a moving robot the key parameters are the adaptive threshold
        # window sizes. The adaptive threshold binarizes the grayscale image
        # by comparing each pixel to its local neighborhood average.
        #
        # When the marker is blurry:
        #   - A small window (e.g. 3px) can't "see" the blurred edges at all.
        #   - Larger windows (up to ~25px) still pick up the overall contrast
        #     gradient even when individual edges are soft.
        #
        # The detector tries multiple window sizes between Min and Max.
        # Wider range = more attempts = slightly slower but much more robust.
        # At 30 FPS you have 33ms; even with wider range this stays <5ms.

        params = aruco.DetectorParameters()

        # Adaptive threshold window range — wider catches blurred edges
        params.adaptiveThreshWinSizeMin  = 3    # default: 3
        params.adaptiveThreshWinSizeMax  = 25   # default: 23  (bumped up)
        params.adaptiveThreshWinSizeStep = 4    # default: 10  (finer sweep)
        params.adaptiveThreshConstant    = 7    # default: 7   (keep as-is)

        # Corner refinement — DISABLE for motion.
        # Subpixel refinement tries to find the exact corner location to
        # sub-pixel accuracy. This works great on sharp images but
        # converges to the wrong location on blurry ones. Costs ~1ms too.
        params.cornerRefinementMethod = aruco.CORNER_REFINE_NONE

        # Error correction rate — how many bit errors to tolerate.
        # Default is 0.6. Bumping to 0.8 helps partial decodes under motion
        # but can increase false positives. 0.7 is a good middle ground.
        params.errorCorrectionRate = 0.7

        # Minimum marker perimeter in pixels.
        # At 220cm height with 1280×720 your 3.25" marker is ~43px wide,
        # so perimeter ≈ 172px. Setting min to 80 rejects tiny noise blobs.
        params.minMarkerPerimeterRate = 0.02   # fraction of image perimeter
        params.maxMarkerPerimeterRate = 0.5

        # Perspective remove size — leave at default
        params.perspectiveRemovePixelPerCell = 4
        params.perspectiveRemoveIgnoredMarginPerCell = 0.13

        self.detector = aruco.ArucoDetector(self.aruco_dict, params)

        # ── Internal state ────────────────────────────────────────────────────
        # Track detection history per marker ID for confidence smoothing
        # history[id] = deque of recent confidence values
        from collections import deque
        self._history: dict[int, deque] = {}
        self._history_len = 5   # smooth over last 5 frames

        print(
            f"[ArUco] Initialized  "
            f"dict={self._dict_name(aruco_dict_type)}  "
            f"marker={marker_size_cm:.2f}cm"
        )

    # ── Main detection method ─────────────────────────────────────────────────

    def detect(self, frame_data: dict) -> list[dict]:
        """
        Detect all ArUco markers in a frame.

        Args:
            frame_data: dict from RealSenseCamera.get_frame()

        Returns:
            List of detection dicts (empty if none found). Each dict:
            {
                'id':            int   — marker ID
                'center_pixel':  (int, int)  — (col, row) of marker center
                'corners':       np.ndarray shape (4,2) float32
                'heading_deg':   float — robot heading in degrees, world frame
                                         0° = +X direction (right)
                                         90° = +Y direction (forward)
                                         Increases counter-clockwise.
                'position_world': (float, float, float) | None
                                         (x_cm, y_cm, z_cm) in world frame
                'depth_value':   int   — raw depth at marker center
                'confidence':    float — 0.0–1.0, how reliable this detection is
                'raw_corners':   np.ndarray — unmodified corners from detector
            }
        """
        color_image = frame_data['color_image']
        depth_image = frame_data['depth_image']

        # Convert to grayscale
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        gray_small = cv2.resize(gray, (640, 360))

        # Detect
        corners, ids, _ = self.detector.detectMarkers(gray_small)
        corners = [c * 2.0 for c in corners]


        if ids is None:
            return []

        results = []

        for i, marker_id in enumerate(ids.flatten()):
            marker_corners = corners[i][0]   # shape (4, 2), float32

            # ── Center pixel ──────────────────────────────────────────────────
            cx = int(np.mean(marker_corners[:, 0]))
            cy = int(np.mean(marker_corners[:, 1]))

            # Clamp to image bounds for safe depth lookup
            cx_safe = np.clip(cx, 0, depth_image.shape[1] - 1)
            cy_safe = np.clip(cy, 0, depth_image.shape[0] - 1)
            depth_value = int(depth_image[cy_safe, cx_safe])

            # ── World position ────────────────────────────────────────────────
            position_world = self.cam.pixel_to_world(cx_safe, cy_safe, depth_value)

            # ── Heading ───────────────────────────────────────────────────────
            heading_deg = self._compute_heading(marker_corners)

            # ── Confidence ───────────────────────────────────────────────────
            raw_conf = self._compute_confidence(marker_corners, depth_value, depth_image.shape)
            smoothed_conf = self._smooth_confidence(int(marker_id), raw_conf)

            results.append({
                'id':             int(marker_id),
                'center_pixel':   (cx, cy),
                'corners':        marker_corners,
                'heading_deg':    heading_deg,
                'position_world': position_world,
                'depth_value':    depth_value,
                'confidence':     smoothed_conf,
                'raw_corners':    corners[i],
            })

        return results

    # ── Heading calculation ───────────────────────────────────────────────────

    def _compute_heading(self, corners: np.ndarray) -> float:
        """
        Compute robot heading from marker corner positions.

        ArUco corner order (standard): top-left, top-right, bottom-right, bottom-left.
        The "forward" direction of the marker is defined as the vector from
        the midpoint of the bottom edge to the midpoint of the top edge.
        This lets you orient your printed marker so its "top" faces the robot's
        front, giving you heading without any additional sensors.

        The angle is measured in the world X/Y plane:
            0°   = pointing in +X direction (right)
            90°  = pointing in +Y direction (forward)
            180° = pointing in -X direction (left)
            270° = pointing in -Y direction (backward)
        Angle increases counter-clockwise (standard math convention).

        To align with your robot: mount the ArUco marker so its printed
        "top" (corner 0→1 edge) faces the robot's forward direction.
        Then heading_deg == 0 when robot faces right, 90 when facing forward, etc.

        Args:
            corners: np.ndarray shape (4, 2) — marker corner pixels
                     Order: [top-left, top-right, bottom-right, bottom-left]

        Returns:
            Heading angle in degrees [0, 360).
        """
        # Top edge midpoint (between corner 0 and corner 1)
        top_mid = (corners[0] + corners[1]) / 2.0
        # Bottom edge midpoint (between corner 3 and corner 2)
        bot_mid = (corners[3] + corners[2]) / 2.0

        # Vector from bottom-mid to top-mid (pixel space)
        # In pixel space Y increases downward, so we flip Y to get world direction
        dx =  (top_mid[0] - bot_mid[0])   # pixel X → world X (no flip)
        dy = -(top_mid[1] - bot_mid[1])   # pixel Y → world Y (flip sign)

        # atan2 gives angle from +X axis, counter-clockwise
        angle_rad = np.arctan2(dy, dx)
        angle_deg = np.degrees(angle_rad) % 360.0

        return angle_deg

    # ── Confidence scoring ────────────────────────────────────────────────────

    def _compute_confidence(
        self,
        corners: np.ndarray,
        depth_value: int,
        image_shape: tuple,
    ) -> float:
        """
        Compute a raw confidence score (0.0–1.0) for one detection.

        Three components, weighted equally:

        1. Depth validity (0 or 1)
           A zero depth means the sensor returned no reading at the marker
           center — happens at edges or with IR interference. Hard zero.

        2. Size plausibility (0.0–1.0)
           The marker's pixel perimeter should match what we'd expect given
           the camera height. Too small → partial detection. Too large → noise.
           Score peaks at the expected size and falls off gracefully.

        3. Corner regularity (0.0–1.0)
           A real square viewed from directly overhead should have four corners
           forming a near-perfect square. We measure how close the four side
           lengths are to equal. High variance → blurry/distorted detection.

        Args:
            corners:     shape (4, 2) marker corners
            depth_value: raw uint16 depth at marker center
            image_shape: (H, W) of the image (for size normalization)

        Returns:
            Confidence score in [0.0, 1.0].
        """

        # ── Component 1: depth validity ───────────────────────────────────────
        depth_score = 1.0 if depth_value > 0 else 0.0

        # ── Component 2: size plausibility ───────────────────────────────────
        # Expected pixel width at our calibrated camera height:
        #   pixel_width ≈ (marker_cm / coverage_cm) * image_width
        # Coverage from calibration report: 243 cm wide at 1280px
        # For 3.25" (8.255cm) marker: 8.255/243 * 1280 ≈ 43px
        img_h, img_w = image_shape[:2]
        coverage_cm_per_pixel = 243.0 / 1280.0   # from calibration
        expected_px = self.marker_size_cm / coverage_cm_per_pixel

        # Measure actual pixel width (average of two horizontal-ish sides)
        side_top = np.linalg.norm(corners[1] - corners[0])
        side_bot = np.linalg.norm(corners[2] - corners[3])
        actual_px = (side_top + side_bot) / 2.0

        # Gaussian falloff: score = 1 at expected, drops toward 0 at 2× off
        size_ratio = actual_px / max(expected_px, 1.0)
        size_score = np.exp(-((size_ratio - 1.0) ** 2) / (2 * 0.4 ** 2))

        # ── Component 3: corner regularity ───────────────────────────────────
        sides = [
            np.linalg.norm(corners[1] - corners[0]),  # top
            np.linalg.norm(corners[2] - corners[1]),  # right
            np.linalg.norm(corners[3] - corners[2]),  # bottom
            np.linalg.norm(corners[0] - corners[3]),  # left
        ]
        mean_side = np.mean(sides)
        if mean_side > 0:
            cv_sides = np.std(sides) / mean_side   # coefficient of variation
            # cv=0 → perfect square → score=1; cv=0.5 → very distorted → score≈0
            regularity_score = np.exp(-((cv_sides / 0.25) ** 2))
        else:
            regularity_score = 0.0

        # ── Weighted combination ──────────────────────────────────────────────
        # Depth validity is a gate: if depth is bad, cap total at 0.5
        raw = (depth_score * 0.3) + (size_score * 0.35) + (regularity_score * 0.35)
        if depth_score == 0.0:
            raw = min(raw, 0.5)

        return float(np.clip(raw, 0.0, 1.0))

    def _smooth_confidence(self, marker_id: int, raw_conf: float) -> float:
        """
        Smooth confidence over recent frames to reduce single-frame noise.

        Args:
            marker_id: Integer marker ID
            raw_conf:  Raw confidence for this frame

        Returns:
            Smoothed confidence (rolling mean over last N frames).
        """
        from collections import deque
        if marker_id not in self._history:
            self._history[marker_id] = deque(maxlen=self._history_len)
        self._history[marker_id].append(raw_conf)
        return float(np.mean(self._history[marker_id]))

    # ── Visualization ─────────────────────────────────────────────────────────

    def draw_detections(
        self,
        image: np.ndarray,
        detections: list[dict],
        draw_heading: bool = True,
        draw_axes: bool = False,
    ) -> np.ndarray:
        """
        Draw detected markers on an image.

        Args:
            image:         BGR image to draw on (will be copied)
            detections:    Output of detect()
            draw_heading:  Draw an arrow showing robot heading direction
            draw_axes:     Draw pose axes (requires camera matrix, slower)

        Returns:
            Annotated BGR image.
        """
        vis = image.copy()

        for det in detections:
            corners = det['corners'].astype(int)
            cx, cy  = det['center_pixel']
            conf    = det['confidence']
            heading = det['heading_deg']
            marker_id = det['id']

            # Color based on confidence: green (high) → yellow → red (low)
            if conf >= 0.7:
                color = (0, 220, 0)
            elif conf >= 0.4:
                color = (0, 200, 200)
            else:
                color = (0, 60, 220)

            # Marker outline
            cv2.polylines(vis, [corners], True, color, 2)

            # Corner dots
            for corner in corners:
                cv2.circle(vis, tuple(corner), 4, (0, 0, 255), -1)

            # Center dot
            cv2.circle(vis, (cx, cy), 5, (255, 255, 255), -1)

            # Heading arrow
            if draw_heading:
                arrow_len = 40
                angle_rad = np.radians(heading)
                # In pixel space Y is flipped vs world Y
                ax = int(cx + arrow_len * np.cos(angle_rad))
                ay = int(cy - arrow_len * np.sin(angle_rad))
                cv2.arrowedLine(vis, (cx, cy), (ax, ay), (0, 255, 255), 2, tipLength=0.3)

            # Pose axes (3D, optional — needs camera matrix)
            if draw_axes and self.cam.camera_matrix is not None:
                # Quick single-axis via solvePnP for overhead
                # (overhead use case: just shows Z axis pointing up from marker)
                pass   # Implement in Week 7 when pose tracking is added

            # ID label
            cv2.putText(vis, f"ID:{marker_id}",
                        (cx - 25, cy - 45),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            # Confidence label
            cv2.putText(vis, f"{conf:.2f}",
                        (cx - 20, cy - 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)

            # Heading label
            cv2.putText(vis, f"{heading:.0f}°",
                        (cx + 10, cy + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)

            # World position label
            if det['position_world']:
                wx, wy, wz = det['position_world']
                pos_text = f"({wx:.0f}, {wy:.0f}) cm"
                cv2.putText(vis, pos_text,
                            (cx - 60, cy + 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 0), 1)

        return vis

    # ── Diagnostics ───────────────────────────────────────────────────────────

    def get_camera_diagnostics(self) -> dict:
        """
        Read live diagnostic values directly from the RealSense color sensor.

        Returns a dict with the actual exposure the auto system chose,
        plus gain and white balance. Call this once per second (not every
        frame) since sensor option reads have a small overhead.

        Returns:
            dict with keys:
                'exposure_us'    : float  — actual exposure in microseconds
                'gain'           : float  — analog gain (higher = noisier)
                'auto_exposure'  : bool   — True if auto exposure is active
                'white_balance'  : float  — color temperature in Kelvin
                'auto_wb'        : bool   — True if auto white balance active
                'brightness'     : float  — brightness setting
        """
        sensor = self.cam.color_sensor
        if sensor is None:
            return {}

        def safe_get(option):
            try:
                return sensor.get_option(option)
            except Exception:
                return None

        return {
            'exposure_us':   safe_get(rs.option.exposure),
            'gain':          safe_get(rs.option.gain),
            'auto_exposure': bool(safe_get(rs.option.enable_auto_exposure)),
            'white_balance': safe_get(rs.option.white_balance),
            'auto_wb':       bool(safe_get(rs.option.enable_auto_white_balance)),
            'brightness':    safe_get(rs.option.brightness),
        }

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _dict_name(aruco_dict_type: int) -> str:
        names = {
            aruco.DICT_4X4_50:  "DICT_4X4_50",
            aruco.DICT_4X4_100: "DICT_4X4_100",
            aruco.DICT_5X5_50:  "DICT_5X5_50",
            aruco.DICT_5X5_100: "DICT_5X5_100",
            aruco.DICT_6X6_50:  "DICT_6X6_50",
            aruco.DICT_6X6_250: "DICT_6X6_250",
        }
        return names.get(aruco_dict_type, f"type_{aruco_dict_type}")


# ── Standalone diagnostic demo ────────────────────────────────────────────────
# Run directly to diagnose detection issues and characterize your camera:
#   python aruco_detector.py
#
# This diagnostic mode answers three questions:
#   1. What exposure value is auto-exposure actually choosing?
#   2. Is the pipeline dropping frames during fast motion?
#   3. What does confidence look like when the marker is still vs. moving?
#
# All per-frame data is logged to a CSV so you can analyze it afterward
# in Excel or plot it to find patterns (e.g. "detection drops when FPS dips").

if __name__ == "__main__":

    import csv
    import os
    import pyrealsense2 as rs
    from collections import deque
    from datetime import datetime

    print("=" * 60)
    print("aruco_detector.py — Diagnostic Mode")
    print("=" * 60)
    print()
    print("This mode logs per-frame data to a CSV file so you can")
    print("analyze exactly what the camera and detector are doing.")
    print()
    print("Controls:")
    print("  d  →  Switch to DICT_4X4_50  (fewer cells, faster)")
    print("  D  →  Switch to DICT_6X6_250 (more cells, original)")
    print("  r  →  Reset frame counters")
    print("  s  →  Print snapshot to console")
    print("  q  →  Quit and save CSV")
    print()
    print("Workflow:")
    print("  1. Let it run still for ~5 seconds (baseline)")
    print("  2. Move marker slowly for ~5 seconds")
    print("  3. Move marker at full robot speed for ~5 seconds")
    print("  4. Press q — open the CSV to see what changed")
    print()

    # ── Init camera ───────────────────────────────────────────────────────────
    cam = RealSenseCamera(
        resolution='1280x720',
        exposure_us=None,           # Auto — we'll read the actual value live
        camera_height_cm=220.0,
    )

    detector = ArucoDetector(
        cam,
        marker_size_cm=MARKER_3_25_INCH_CM,
        aruco_dict_type=aruco.DICT_4X4_50,
    )

    cv2.namedWindow("ArUco Diagnostic", cv2.WINDOW_NORMAL)

    # ── CSV log setup ─────────────────────────────────────────────────────────
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        f"aruco_diagnostic_{timestamp_str}.csv"
    )
    csv_file   = open(csv_path, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        # Timing
        'wall_time_s',        # seconds since session start
        'frame_interval_ms',  # time since last frame (ideal = 33ms at 30FPS)
        'detect_time_ms',     # how long detect() took this frame
        # Camera sensor state
        'exposure_us',        # what auto-exposure actually chose
        'gain',               # analog gain (higher = noisier image)
        # Detection result
        'num_detected',       # markers found this frame (0 or 1+)
        'confidence',         # best marker confidence (blank if none)
        'heading_deg',        # heading of first marker (blank if none)
        'world_x_cm',         # world X of first marker (blank if none)
        'world_y_cm',         # world Y of first marker (blank if none)
        # Rolling stats
        'rolling_fps',        # FPS over last 30 frames
        'rolling_det_rate',   # detection rate over last 30 frames (%)
    ])

    # ── State ─────────────────────────────────────────────────────────────────
    session_start     = time.perf_counter()
    last_frame_time   = time.perf_counter()
    total_frames      = 0
    detected_frames   = 0
    current_dict      = "DICT_4X4_50"

    # Rolling windows (last 30 frames) for live FPS and detection rate display
    interval_window   = deque(maxlen=30)   # frame intervals in ms
    detected_window   = deque(maxlen=30)   # 1 if detected, 0 if not

    # Diagnostic sensor read — only read sensor options once per second
    # (sensor option reads have overhead; don't do it every frame)
    last_diag_time    = 0.0
    cached_diag       = {'exposure_us': None, 'gain': None}

    print(f"Logging to: {csv_path}")
    print("Running... move the marker in different ways to collect data.\n")

    while True:

        # ── Frame capture + timing ────────────────────────────────────────────
        frame_start   = time.perf_counter()
        frame_data    = cam.get_frame()
        if frame_data is None:
            continue

        now            = time.perf_counter()
        wall_time_s    = now - session_start
        frame_interval = (now - last_frame_time) * 1000.0   # ms
        last_frame_time = now

        # ── Detection + timing ────────────────────────────────────────────────
        detect_start = time.perf_counter()
        detections   = detector.detect(frame_data)
        detect_ms    = (time.perf_counter() - detect_start) * 1000.0

        total_frames  += 1
        did_detect     = len(detections) > 0
        if did_detect:
            detected_frames += 1

        interval_window.append(frame_interval)
        detected_window.append(1 if did_detect else 0)

        # ── Rolling stats ─────────────────────────────────────────────────────
        rolling_fps      = 1000.0 / np.mean(interval_window) if interval_window else 0.0
        rolling_det_rate = np.mean(detected_window) * 100.0 if detected_window else 0.0

        # Flag frames that are suspiciously slow — potential pipeline drops
        # At 30 FPS a normal frame is 33ms. >50ms means a frame was likely skipped.
        frame_drop_flag = frame_interval > 50.0

        # ── Sensor diagnostics (once per second) ──────────────────────────────
        if wall_time_s - last_diag_time >= 1.0:
            cached_diag   = detector.get_camera_diagnostics()
            last_diag_time = wall_time_s

        exposure_us = cached_diag.get('exposure_us')
        gain        = cached_diag.get('gain')

        # ── Extract best detection data for CSV ───────────────────────────────
        best = detections[0] if detections else None
        csv_writer.writerow([
            f"{wall_time_s:.3f}",
            f"{frame_interval:.2f}",
            f"{detect_ms:.2f}",
            f"{exposure_us:.0f}" if exposure_us is not None else "",
            f"{gain:.1f}"        if gain        is not None else "",
            len(detections),
            f"{best['confidence']:.3f}"       if best else "",
            f"{best['heading_deg']:.1f}"      if best else "",
            f"{best['position_world'][0]:.1f}" if best and best['position_world'] else "",
            f"{best['position_world'][1]:.1f}" if best and best['position_world'] else "",
            f"{rolling_fps:.1f}",
            f"{rolling_det_rate:.1f}",
        ])

        # ── Draw ──────────────────────────────────────────────────────────────
        vis = detector.draw_detections(frame_data['color_image'], detections)
        h, w = vis.shape[:2]

        # ── HUD — left column ─────────────────────────────────────────────────
        def hud(text, row, color=(220, 220, 220)):
            cv2.putText(vis, text, (10, 30 + row * 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)

        hud(f"FPS: {rolling_fps:.1f}", 0)
        hud(f"Dict: {current_dict}", 1)

        if exposure_us is not None:
            hud(f"Exposure: {exposure_us:.0f} us", 2, (0, 255, 255))
        else:
            hud("Exposure: reading...", 2, (180, 180, 0))

        if gain is not None:
            hud(f"Gain: {gain:.1f}", 3, (0, 255, 255))

        det_color = (0, 220, 0) if did_detect else (0, 60, 220)
        hud(f"Detected: {'YES' if did_detect else 'NO '}  "
            f"Rate: {rolling_det_rate:.0f}%", 4, det_color)

        if best:
            hud(f"Conf: {best['confidence']:.3f}  "
                f"Hdg: {best['heading_deg']:.0f}°", 5, (0, 220, 0))

        # Frame drop warning — red banner at top
        if frame_drop_flag:
            cv2.rectangle(vis, (0, 0), (w, 28), (0, 0, 180), -1)
            cv2.putText(vis, f"FRAME DROP DETECTED  ({frame_interval:.0f} ms)",
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

        # ── HUD — bottom bar ──────────────────────────────────────────────────
        cv2.rectangle(vis, (0, h - 30), (w, h), (40, 40, 40), -1)
        cv2.putText(vis, "d=DICT_4X4  D=DICT_6X6  r=reset  s=snapshot  q=quit+save",
                    (10, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # Frame interval graph — right edge, last 30 frames as bar chart
        # Green bar = normal (<50ms). Red = drop (>50ms).
        bar_w    = 6
        bar_x0   = w - (bar_w + 2) * 30 - 10
        bar_base = h - 35
        bar_scale = 2.0   # pixels per ms (50ms → 100px tall)
        for i, iv in enumerate(interval_window):
            bx     = bar_x0 + i * (bar_w + 2)
            bar_h  = int(min(iv * bar_scale, 120))
            color  = (0, 180, 0) if iv < 50 else (0, 0, 220)
            cv2.rectangle(vis, (bx, bar_base - bar_h), (bx + bar_w, bar_base), color, -1)
        # Label
        cv2.putText(vis, "frame intervals (30fr)",
                    (bar_x0, bar_base + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (160, 160, 160), 1)

        cv2.imshow("ArUco Diagnostic", vis)

        # ── Keys ──────────────────────────────────────────────────────────────
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

        elif key == ord('r'):
            total_frames = detected_frames = 0
            interval_window.clear()
            detected_window.clear()
            print("[Diagnostic] Counters reset.")

        elif key == ord('d'):
            detector = ArucoDetector(cam, marker_size_cm=MARKER_3_25_INCH_CM,
                                     aruco_dict_type=aruco.DICT_4X4_50)
            current_dict = "DICT_4X4_50"
            total_frames = detected_frames = 0
            interval_window.clear(); detected_window.clear()
            print("[Diagnostic] Switched to DICT_4X4_50")

        elif key == ord('D'):
            detector = ArucoDetector(cam, marker_size_cm=MARKER_3_25_INCH_CM,
                                     aruco_dict_type=aruco.DICT_6X6_250)
            current_dict = "DICT_6X6_250"
            total_frames = detected_frames = 0
            interval_window.clear(); detected_window.clear()
            print("[Diagnostic] Switched to DICT_6X6_250")

        elif key == ord('s'):
            # ── Console snapshot ──────────────────────────────────────────────
            print("\n" + "=" * 55)
            print(f"Diagnostic Snapshot  t={wall_time_s:.1f}s")
            print("=" * 55)
            print(f"  Rolling FPS          : {rolling_fps:.1f}")
            print(f"  Detection rate (30f) : {rolling_det_rate:.1f}%")
            print(f"  Total frames         : {total_frames}")
            print(f"  Total detected       : {detected_frames}  "
                  f"({detected_frames/max(total_frames,1)*100:.1f}%)")
            print(f"  Last frame interval  : {frame_interval:.2f} ms")
            print(f"  Last detect() time   : {detect_ms:.2f} ms")
            if exposure_us is not None:
                print(f"  Auto-exposure chose  : {exposure_us:.0f} µs")
                print(f"  Gain                 : {gain:.1f}")
                print()
                # Interpret the exposure value for the user
                if exposure_us < 2000:
                    print("  ► Very low exposure — excellent for fast motion.")
                    print("    Blur should not be the issue. Check marker mounting.")
                elif exposure_us < 5000:
                    print("  ► Low-moderate exposure — good for moderate speed.")
                    print("    May get slight blur at max robot speed.")
                elif exposure_us < 10000:
                    print("  ► Moderate exposure — fine for slow motion.")
                    print("    Motion blur likely at full robot speed.")
                else:
                    print("  ► High exposure — motion blur very likely.")
                    print("    Consider adding more light or forcing lower exposure.")
            if detections:
                for d in detections:
                    print(f"\n  Marker ID {d['id']}:")
                    print(f"    Confidence : {d['confidence']:.3f}")
                    print(f"    Heading    : {d['heading_deg']:.1f}°")
                    if d['position_world']:
                        wx, wy, wz = d['position_world']
                        print(f"    World pos  : ({wx:.1f}, {wy:.1f}, {wz:.1f}) cm")
            print("=" * 55 + "\n")

    # ── Cleanup ───────────────────────────────────────────────────────────────
    cv2.destroyAllWindows()
    csv_file.close()
    cam.shutdown()

    # ── End-of-session summary ────────────────────────────────────────────────
    print("\n" + "=" * 60)
    print("Diagnostic Session Complete")
    print("=" * 60)
    print(f"  Total frames     : {total_frames}")
    if total_frames > 0:
        print(f"  Overall det rate : {detected_frames/total_frames*100:.1f}%")
    print(f"  CSV saved to     : {csv_path}")
    print()
    print("What to look for in the CSV:")
    print("  1. 'exposure_us' column — what did auto-exposure actually use?")
    print("     Under 3000 µs → blur is NOT your problem.")
    print("     Over 8000 µs  → blur IS likely contributing.")
    print()
    print("  2. 'frame_interval_ms' column — look for values > 50 ms.")
    print("     These are frame drops. If they correlate with num_detected=0,")
    print("     the pipeline is the bottleneck, not the detector.")
    print()
    print("  3. 'confidence' column — does it drop before num_detected goes to 0?")
    print("     If yes: the detector sees a degraded marker before losing it.")
    print("     If no (sudden drop): likely a geometric/mounting issue.")
    print()
    print("  4. 'detect_time_ms' column — should be 2-8 ms per frame.")
    print("     Consistently > 15 ms means the detector is your FPS bottleneck.")
    print("=" * 60)
