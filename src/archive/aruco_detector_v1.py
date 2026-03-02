"""
aruco_detector_v1.py - ArUco Marker Detector for Overhead Robot Tracking
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
    python aruco_detector_v1.py

    # Import in other modules:
    from aruco_detector import ArucoDetector
    detector = ArucoDetector(cam, marker_size_cm=8.255)
    markers = detector.detect(frame_data)
"""

import time
import numpy as np
import cv2
from cv2 import aruco

from camera_v1 import RealSenseCamera


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

        # Detect
        corners, ids, _ = self.detector.detectMarkers(gray)

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


# ── Standalone live demo ──────────────────────────────────────────────────────
# Run directly to test detection and tune exposure:
#   python aruco_detector_v1.py

if __name__ == "__main__":

    print("=" * 60)
    print("aruco_detector_v1.py — Live Detection Demo")
    print("=" * 60)
    print()
    print("Controls:")
    print("  1  →  Auto exposure")
    print("  2  →  8000 µs  (try this first)")
    print("  3  →  6000 µs  (good starting point)")
    print("  4  →  4000 µs")
    print("  5  →  3000 µs  (for fast motion)")
    print("  d  →  Switch to DICT_4X4_50  (recommended)")
    print("  D  →  Switch to DICT_6X6_250 (original)")
    print("  s  →  Print detection stats to console")
    print("  q  →  Quit")
    print()
    print("TIP: Start with key '3' (6000µs). Move the marker quickly.")
    print("     If still losing detection, try '4' or '5'.")
    print("     If image is too dark, go back up to '2'.")
    print()

    # ── Exposure options ──────────────────────────────────────────────────────
    EXPOSURE_OPTIONS = {
        ord('1'): (None,  "AUTO"),
        ord('2'): (8000,  "8000 µs"),
        ord('3'): (6000,  "6000 µs"),
        ord('4'): (4000,  "4000 µs"),
        ord('5'): (3000,  "3000 µs"),
    }

    # ── Init ──────────────────────────────────────────────────────────────────
    cam = RealSenseCamera(
        resolution='1280x720',
        exposure_us=None,          # Start with auto; press 3 to switch
        camera_height_cm=220.0,
    )

    detector = ArucoDetector(
        cam,
        marker_size_cm=MARKER_3_25_INCH_CM,
        aruco_dict_type=aruco.DICT_4X4_50,
    )

    cv2.namedWindow("ArUco Detector", cv2.WINDOW_NORMAL)

    # ── Stats tracking ────────────────────────────────────────────────────────
    total_frames   = 0
    detected_frames = 0
    fps_timer      = time.time()
    fps_display    = 0.0
    frame_counter  = 0
    current_dict   = "DICT_4X4_50"
    current_exp    = "AUTO"

    print("Running... (camera window should open)")

    while True:
        frame_data = cam.get_frame()
        if frame_data is None:
            continue

        # ── Detection ─────────────────────────────────────────────────────────
        detections = detector.detect(frame_data)

        total_frames   += 1
        frame_counter  += 1
        if len(detections) > 0:
            detected_frames += 1

        # ── FPS ───────────────────────────────────────────────────────────────
        elapsed = time.time() - fps_timer
        if elapsed >= 1.0:
            fps_display   = frame_counter / elapsed
            frame_counter = 0
            fps_timer     = time.time()

        # ── Draw ──────────────────────────────────────────────────────────────
        vis = detector.draw_detections(frame_data['color_image'], detections)

        det_rate = (detected_frames / total_frames * 100) if total_frames > 0 else 0.0

        # HUD
        cv2.putText(vis, f"FPS: {fps_display:.1f}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        cv2.putText(vis, f"Exp: {current_exp}  Dict: {current_dict}",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 1)
        cv2.putText(vis, f"Detected: {len(detections)}  Rate: {det_rate:.1f}%",
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0,220,0) if len(detections) > 0 else (0,60,220), 2)
        cv2.putText(vis, "1-5=exposure  d/D=dict  s=stats  q=quit",
                    (10, vis.shape[0] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180,180,180), 1)

        cv2.imshow("ArUco Detector", vis)

        # ── Key handling ──────────────────────────────────────────────────────
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

        elif key in EXPOSURE_OPTIONS:
            exp_val, exp_label = EXPOSURE_OPTIONS[key]
            current_exp = exp_label
            cam.set_exposure(exp_val if exp_val is not None else 0)

        elif key == ord('d'):
            detector = ArucoDetector(
                cam,
                marker_size_cm=MARKER_3_25_INCH_CM,
                aruco_dict_type=aruco.DICT_4X4_50,
            )
            current_dict = "DICT_4X4_50"
            total_frames = detected_frames = 0
            print("[Demo] Switched to DICT_4X4_50")

        elif key == ord('D'):
            detector = ArucoDetector(
                cam,
                marker_size_cm=MARKER_3_25_INCH_CM,
                aruco_dict_type=aruco.DICT_6X6_250,
            )
            current_dict = "DICT_6X6_250"
            total_frames = detected_frames = 0
            print("[Demo] Switched to DICT_6X6_250")

        elif key == ord('s'):
            print("\n" + "=" * 50)
            print("Detection Stats")
            print("=" * 50)
            print(f"  Total frames:    {total_frames}")
            print(f"  Detected frames: {detected_frames}")
            print(f"  Detection rate:  {det_rate:.1f}%")
            print(f"  FPS:             {fps_display:.1f}")
            print(f"  Exposure:        {current_exp}")
            print(f"  Dictionary:      {current_dict}")
            if detections:
                for d in detections:
                    print(f"\n  Marker ID {d['id']}:")
                    print(f"    Confidence : {d['confidence']:.3f}")
                    print(f"    Heading    : {d['heading_deg']:.1f}°")
                    if d['position_world']:
                        wx, wy, wz = d['position_world']
                        print(f"    World pos  : ({wx:.1f}, {wy:.1f}, {wz:.1f}) cm")
            print("=" * 50 + "\n")

    cv2.destroyAllWindows()
    cam.shutdown()

    # ── Final summary ─────────────────────────────────────────────────────────
    print("\n" + "=" * 60)
    print("Session Summary")
    print("=" * 60)
    if total_frames > 0:
        print(f"  Total frames   : {total_frames}")
        print(f"  Detection rate : {detected_frames/total_frames*100:.1f}%")
        print(f"  Final exposure : {current_exp}")
        print(f"  Final dict     : {current_dict}")
    print("\nRecommendation:")
    print("  Record the exposure value that gives best detection")
    print("  while the robot is moving at full speed.")
    print("  Use that value in RealSenseCamera(exposure_us=...) going forward.")
    print("=" * 60)
