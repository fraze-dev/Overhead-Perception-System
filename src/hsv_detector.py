"""
hsv_detector.py - HSV Color Detector for Ball and Robot
Overhead Perception System
Author: Aaron Fraze
Date: February 23, 2026

Refactored from hsv_v2_with_depth_FIXED.py. Key changes:
    - Camera code removed; takes RealSenseCamera from camera.py
    - Added 'hambot_green' target for the dark green robot body
    - Added circularity filter on the ball (ball is round, noise blobs aren't)
    - Added depth gate: rejects detections at implausible heights above floor
    - Added confidence score (0.0-1.0) matching ArucoDetector output format
    - HSV profiles save/load to JSON so tuned values survive between sessions
    - Tuning mode: '1' = ball, '2' = robot green, 's' = save to disk
    - Output dict uses 'position_world' (x_cm, y_cm, z_cm) consistently

Usage:
    # Standalone tuning or live demo:
    python hsv_detector.py

    # As a module:
    from camera import RealSenseCamera
    from hsv_detector import HsvDetector

    cam      = RealSenseCamera(camera_height_cm=220.0)
    detector = HsvDetector(cam)
    results  = detector.detect(frame_data)   # frame_data from cam.get_frame()

    ball  = results['ball']   # dict or None
    robot = results['robot']  # dict or None
"""

import json
import os
import time
import numpy as np
import cv2

from camera import RealSenseCamera


# ── Default HSV profiles ──────────────────────────────────────────────────────
# Orange ball values come directly from your tuned hsv_v2 values.
# Robot green is a starting point. Run tuning mode (key '2') to dial it in.
#
# HSV reminder (OpenCV ranges):
#   H  0-179   hue / colour family
#   S  0-255   saturation: 0 = grey, 255 = vivid
#   V  0-255   brightness: 0 = black, 255 = bright
#
# Dark green from overhead lighting tips:
#   H typically 35-85 for greens
#   S moderate to high depending on paint finish
#   V LOW for dark colours — start around 20-100 and tune upward

DEFAULT_PROFILES = {

    'orange_ball': {
        # Your tuned values from hsv_v2 — kept exactly
        'lower':                     [2,   1,   82],
        'upper':                     [7,   255, 239],
        'min_area':                  340,
        'max_area':                  700,
        # Circularity = 4pi*area / perimeter^2  (1.0 = perfect circle)
        # Ball should score >= 0.6 to reject elongated noise blobs
        'min_circularity':           0.6,
        # Depth gate: ball sits on the floor, should NOT be elevated much
        'max_height_above_floor_cm': 10.0,
    },

    'hambot_green': {
        # Starting point for dark green robot body viewed from 220 cm overhead.
        # Run tuning mode and adjust until only the robot body is white in mask.
        'lower':                     [35,  40,  20],
        'upper':                     [85,  255, 120],
        'min_area':                  1000,
        'max_area':                  30000,
        # Robot blob is NOT circular -- no circularity penalty
        'min_circularity':           0.0,
        # Robot chassis ~8-15 cm tall; 30 cm gives comfortable headroom
        'max_height_above_floor_cm': 30.0,
    },
}

# Saved profiles live next to this file so values persist between sessions
PROFILE_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'hsv_profiles.json'
)


class HsvDetector:
    """
    HSV colour detector for the orange ball and dark-green HamBot robot.

    Runs two independent detectors per frame:
        ball  -- orange ball: HSV mask + circularity filter + depth gate
        robot -- robot green body: HSV mask + depth gate
                 (ArUco is the primary robot detector; this is the fallback)

    Both return a standardised dict or None, compatible with WorldState.
    """

    def __init__(self, camera: RealSenseCamera):
        self.cam      = camera
        self.profiles = self._load_profiles()
        print(f"[HSV] Initialized  targets: {list(self.profiles.keys())}")

    # ── Profile persistence ───────────────────────────────────────────────────

    def _load_profiles(self) -> dict:
        """Load from JSON if it exists, otherwise return built-in defaults."""
        if os.path.exists(PROFILE_PATH):
            try:
                with open(PROFILE_PATH) as f:
                    saved = json.load(f)
                # Merge: start from defaults so any new keys always exist,
                # then overlay saved values on top
                profiles = {k: v.copy() for k, v in DEFAULT_PROFILES.items()}
                for name, vals in saved.items():
                    if name in profiles:
                        profiles[name].update(vals)
                print(f"[HSV] Profiles loaded from {PROFILE_PATH}")
                return profiles
            except Exception as e:
                print(f"[HSV] Could not load profiles ({e}), using defaults.")
        return {k: v.copy() for k, v in DEFAULT_PROFILES.items()}

    def save_profiles(self):
        """Write current profiles to JSON. Called on 's' in tuning mode."""
        out = {}
        for name, prof in self.profiles.items():
            out[name] = {
                k: v.tolist() if isinstance(v, np.ndarray) else v
                for k, v in prof.items()
            }
        with open(PROFILE_PATH, 'w') as f:
            json.dump(out, f, indent=2)
        print(f"[HSV] Profiles saved -> {PROFILE_PATH}")

    def _np_bounds(self, name: str):
        """Return lower/upper for a profile as numpy uint8 arrays for cv2.inRange."""
        p = self.profiles[name]
        return (
            np.array(p['lower'], dtype=np.uint8),
            np.array(p['upper'], dtype=np.uint8),
        )

    # ── Public API ────────────────────────────────────────────────────────────

    def detect(self, frame_data: dict) -> dict:
        """
        Run ball and robot detectors on one frame.

        Args:
            frame_data: dict returned by RealSenseCamera.get_frame()

        Returns:
            {'ball': <dict or None>, 'robot': <dict or None>}

        Detection dict keys:
            detected              True
            target                'orange_ball' or 'hambot_green'
            centroid_pixel        (int, int)  pixel coords of blob centre
            circle_center         (int, int)  centre of min enclosing circle
            radius                int         radius in pixels
            area                  float       contour area in pixels^2
            circularity           float       0.0-1.0
            depth_value           int         raw uint16 depth at centroid
            position_world        (x,y,z) cm or None if depth invalid
            height_above_floor_cm float or None
            confidence            float 0.0-1.0
            mask                  np.ndarray uint8 (for debug/visualisation)
        """
        color = frame_data['color_image']
        depth = frame_data['depth_image']
        floor = self._estimate_floor_depth(depth)

        return {
            'ball':  self._detect_target(color, depth, 'orange_ball',  floor),
            'robot': self._detect_target(color, depth, 'hambot_green', floor),
        }

    # ── Core detection pipeline ───────────────────────────────────────────────

    def _detect_target(
        self,
        color_image: np.ndarray,
        depth_image: np.ndarray,
        target_name: str,
        floor_depth_m: float,
    ):
        """Detect one HSV target. Returns detection dict or None."""

        prof   = self.profiles[target_name]
        lo, hi = self._np_bounds(target_name)

        # 1. HSV mask (identical to original hsv_v2 approach)
        hsv  = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lo, hi)

        # 2. Morphological cleanup (same kernel/ops as original)
        kernel = np.ones((5, 5), np.uint8)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 3. Contours
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return None

        # 4. Area + optional circularity filter
        min_circ = prof.get('min_circularity', 0.0)
        valid = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if not (prof['min_area'] <= area <= prof['max_area']):
                continue
            perim = cv2.arcLength(cnt, True)
            circ  = (4 * np.pi * area / perim ** 2) if perim > 0 else 0.0
            if circ < min_circ:
                continue
            valid.append((cnt, area, circ))

        if not valid:
            return None

        # 5. Best = largest valid contour (same rule as original hsv_v2)
        best_cnt, best_area, best_circ = max(valid, key=lambda x: x[1])

        # 6. Centroid + enclosing circle
        M = cv2.moments(best_cnt)
        if M['m00'] == 0:
            return None
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        (ex, ey), radius = cv2.minEnclosingCircle(best_cnt)

        # 7. Depth gate -- reject physically impossible heights
        cx_s        = int(np.clip(cx, 0, depth_image.shape[1] - 1))
        cy_s        = int(np.clip(cy, 0, depth_image.shape[0] - 1))
        depth_value = int(depth_image[cy_s, cx_s])

        height_cm = None
        if depth_value > 0 and floor_depth_m > 0:
            obj_m     = depth_value * self.cam.depth_scale
            height_cm = (floor_depth_m - obj_m) * 100.0
            max_h     = prof.get('max_height_above_floor_cm', 50.0)
            if height_cm < 0 or height_cm > max_h:
                return None  # below floor (noise) or impossibly high

        # 8. World position via camera.py
        position_world = self.cam.pixel_to_world(cx_s, cy_s, depth_value)

        # 9. Confidence score
        confidence = self._confidence(best_area, best_circ, depth_value, prof)

        return {
            'detected':              True,
            'target':                target_name,
            'centroid_pixel':        (cx, cy),
            'circle_center':         (int(ex), int(ey)),
            'radius':                int(radius),
            'area':                  best_area,
            'circularity':           best_circ,
            'depth_value':           depth_value,
            'position_world':        position_world,
            'height_above_floor_cm': height_cm,
            'confidence':            confidence,
            'mask':                  mask,
            'contour':               best_cnt,
        }

    # ── Confidence scoring ────────────────────────────────────────────────────

    def _confidence(self, area, circularity, depth_value, prof) -> float:
        """
        Confidence score 0.0-1.0 from three components:
            area score       Gaussian centred on midpoint of min/max area
            circularity      linear ramp from min_circularity to 1.0
                             (robot target always gets 1.0 here)
            depth score      1.0 if valid depth reading, 0.3 if not
        """
        mid_a   = (prof['min_area'] + prof['max_area']) / 2.0
        sigma_a = (prof['max_area'] - prof['min_area']) / 3.0
        area_s  = float(np.exp(-((area - mid_a) ** 2) / (2 * sigma_a ** 2)))

        min_c  = prof.get('min_circularity', 0.0)
        circ_s = float(np.clip(
            (circularity - min_c) / (1.0 - min_c) if 0 < min_c < 1.0 else 1.0,
            0.0, 1.0
        )) if min_c > 0 else 1.0

        depth_s = 1.0 if depth_value > 0 else 0.3

        return float(np.clip(area_s * 0.35 + circ_s * 0.35 + depth_s * 0.30, 0.0, 1.0))

    # ── Floor estimation ──────────────────────────────────────────────────────

    def _estimate_floor_depth(self, depth_image: np.ndarray) -> float:
        """
        Median of all valid depth pixels as a floor depth estimate.
        From directly overhead the floor is the most common depth value,
        so the median is robust even when the robot and ball are present.
        """
        depth_m = depth_image.astype(np.float32) * self.cam.depth_scale
        valid   = depth_m[depth_m > 0]
        return float(np.median(valid)) if len(valid) >= 100 else 0.0

    # ── Visualisation ─────────────────────────────────────────────────────────

    def draw_detections(self, image: np.ndarray, results: dict) -> np.ndarray:
        """Annotate a copy of image with ball and robot detection results."""
        vis = image.copy()
        targets = [
            ('ball',  results.get('ball'),  (0, 165, 255)),
            ('robot', results.get('robot'), (0, 200, 0)),
        ]
        for label, det, color in targets:
            if det is None:
                cv2.putText(vis, f"{label.upper()}: searching",
                            (10, 30 if label == 'ball' else 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (100, 100, 100), 1)
                continue

            cx, cy = det['centroid_pixel']
            ex, ey = det['circle_center']
            cv2.circle(vis, (ex, ey), det['radius'], color, 2)
            cv2.circle(vis, (cx, cy), 5, (255, 255, 255), -1)
            cv2.line(vis, (cx - 20, cy), (cx + 20, cy), color, 2)
            cv2.line(vis, (cx, cy - 20), (cx, cy + 20), color, 2)

            if det['position_world']:
                wx, wy, _ = det['position_world']
                cv2.putText(vis, f"{label} ({wx:.0f}, {wy:.0f}) cm",
                            (ex - 60, ey - det['radius'] - 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            cv2.putText(vis,
                        f"conf={det['confidence']:.2f}  area={det['area']:.0f}  circ={det['circularity']:.2f}",
                        (ex - 60, ey - det['radius'] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)
            if det['height_above_floor_cm'] is not None:
                cv2.putText(vis, f"h={det['height_above_floor_cm']:.1f}cm",
                            (ex - 20, ey + det['radius'] + 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)
        return vis

    # ── Interactive tuning mode ───────────────────────────────────────────────

    def tuning_mode(self):
        """
        Interactive HSV tuning with live trackbars.
        Refactored from hsv_v2 hsv_tuning_mode(). Differences:
            - Third window always shows the raw colour mask BEFORE area/
              circularity filters so you can see what the colour range captures
              even when area or shape filters then reject the blob
            - 's' writes values to JSON for persistence
            - Supports two targets switchable with '1' and '2'

        Keys:  1=orange ball   2=robot green   s=save   q=quit
        """
        WIN_CTRL   = 'HSV Tuning Controls'
        WIN_MASK   = 'Mask (white = colour match)'
        WIN_RESULT = 'Result'

        print("\n" + "=" * 60)
        print("HSV TUNING MODE")
        print("  1=orange ball   2=robot green   s=save   q=quit")
        print("  Adjust until ONLY your target is white in the Mask window.")
        print("=" * 60)

        current = 'orange_ball'

        cv2.namedWindow(WIN_CTRL)
        cv2.namedWindow(WIN_MASK)
        cv2.namedWindow(WIN_RESULT)

        def sync_trackbars(name):
            p  = self.profiles[name]
            lo = p['lower'] if isinstance(p['lower'], list) else p['lower'].tolist()
            hi = p['upper'] if isinstance(p['upper'], list) else p['upper'].tolist()
            cv2.setTrackbarPos('H Low',    WIN_CTRL, lo[0])
            cv2.setTrackbarPos('S Low',    WIN_CTRL, lo[1])
            cv2.setTrackbarPos('V Low',    WIN_CTRL, lo[2])
            cv2.setTrackbarPos('H High',   WIN_CTRL, hi[0])
            cv2.setTrackbarPos('S High',   WIN_CTRL, hi[1])
            cv2.setTrackbarPos('V High',   WIN_CTRL, hi[2])
            cv2.setTrackbarPos('Min Area', WIN_CTRL, p['min_area'])
            cv2.setTrackbarPos('Max Area', WIN_CTRL, min(p['max_area'], 100000))

        p0 = self.profiles[current]
        lo0 = p0['lower'] if isinstance(p0['lower'], list) else p0['lower'].tolist()
        hi0 = p0['upper'] if isinstance(p0['upper'], list) else p0['upper'].tolist()
        cv2.createTrackbar('H Low',    WIN_CTRL, lo0[0], 179,    lambda x: None)
        cv2.createTrackbar('S Low',    WIN_CTRL, lo0[1], 255,    lambda x: None)
        cv2.createTrackbar('V Low',    WIN_CTRL, lo0[2], 255,    lambda x: None)
        cv2.createTrackbar('H High',   WIN_CTRL, hi0[0], 179,    lambda x: None)
        cv2.createTrackbar('S High',   WIN_CTRL, hi0[1], 255,    lambda x: None)
        cv2.createTrackbar('V High',   WIN_CTRL, hi0[2], 255,    lambda x: None)
        cv2.createTrackbar('Min Area', WIN_CTRL, p0['min_area'],              10000,  lambda x: None)
        cv2.createTrackbar('Max Area', WIN_CTRL, min(p0['max_area'], 100000), 100000, lambda x: None)

        while True:
            frame_data = self.cam.get_frame()
            if frame_data is None:
                continue

            color = frame_data['color_image']
            depth = frame_data['depth_image']

            hl  = cv2.getTrackbarPos('H Low',    WIN_CTRL)
            sl  = cv2.getTrackbarPos('S Low',    WIN_CTRL)
            vl  = cv2.getTrackbarPos('V Low',    WIN_CTRL)
            hh  = cv2.getTrackbarPos('H High',   WIN_CTRL)
            sh  = cv2.getTrackbarPos('S High',   WIN_CTRL)
            vh  = cv2.getTrackbarPos('V High',   WIN_CTRL)
            mna = cv2.getTrackbarPos('Min Area', WIN_CTRL)
            mxa = cv2.getTrackbarPos('Max Area', WIN_CTRL)

            self.profiles[current]['lower']    = [hl, sl, vl]
            self.profiles[current]['upper']    = [hh, sh, vh]
            self.profiles[current]['min_area'] = mna
            self.profiles[current]['max_area'] = mxa

            # Raw colour mask -- always shown so you can see what the HSV
            # range captures BEFORE area/circularity filters reject blobs
            hsv_img  = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
            raw_mask = cv2.inRange(
                hsv_img,
                np.array([hl, sl, vl], dtype=np.uint8),
                np.array([hh, sh, vh], dtype=np.uint8),
            )

            # Full detection pipeline with live profile values
            floor = self._estimate_floor_depth(depth)
            det   = self._detect_target(color, depth, current, floor)

            result = color.copy()
            if det:
                cx, cy = det['centroid_pixel']
                ex, ey = det['circle_center']
                cv2.circle(result, (ex, ey), det['radius'], (0, 255, 0), 2)
                cv2.circle(result, (cx, cy), 5, (0, 0, 255), -1)
                if det['position_world']:
                    wx, wy, _ = det['position_world']
                    cv2.putText(result, f"World: ({wx:.1f}, {wy:.1f}) cm",
                                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(result,
                            (f"area={det['area']:.0f}  circ={det['circularity']:.2f}  "
                             f"h={det['height_above_floor_cm']:.1f}cm  conf={det['confidence']:.2f}"),
                            (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            cv2.putText(result,
                        f"Target: {current}  |  {'DETECTED' if det else 'not found'}  |  1=ball 2=robot s=save q=quit",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

            cv2.imshow(WIN_CTRL,   color)
            cv2.imshow(WIN_MASK,   raw_mask)
            cv2.imshow(WIN_RESULT, result)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                self.save_profiles()
                p = self.profiles[current]
                print(f"\n[Tuning] '{current}' saved:")
                print(f"  lower    = {p['lower']}")
                print(f"  upper    = {p['upper']}")
                print(f"  min_area = {p['min_area']}")
                print(f"  max_area = {p['max_area']}")
            elif key == ord('1'):
                current = 'orange_ball'
                sync_trackbars(current)
                print(f"\n[Tuning] -> {current}")
            elif key == ord('2'):
                current = 'hambot_green'
                sync_trackbars(current)
                print(f"\n[Tuning] -> {current}")

        cv2.destroyAllWindows()


# ── Standalone entry point ────────────────────────────────────────────────────
if __name__ == '__main__':
    print("=" * 60)
    print("hsv_detector.py — Standalone")
    print("=" * 60)
    print("1. Tuning mode  (dial in HSV values for each target)")
    print("2. Live demo    (run both detectors, annotated feed)")
    print()

    cam      = RealSenseCamera(resolution='1280x720', camera_height_cm=220.0)
    detector = HsvDetector(cam)
    choice   = input("Enter choice (1/2): ").strip()

    try:
        if choice == '1':
            detector.tuning_mode()
        else:
            print("\nLive detection running. Press 'q' to quit.")
            cv2.namedWindow('HSV Detection', cv2.WINDOW_NORMAL)
            fc, t0, fps = 0, time.time(), 0.0
            while True:
                fd = cam.get_frame()
                if fd is None:
                    continue
                results = detector.detect(fd)
                vis     = detector.draw_detections(fd['color_image'], results)
                fc += 1
                elapsed = time.time() - t0
                if elapsed >= 1.0:
                    fps = fc / elapsed
                    fc, t0 = 0, time.time()
                cv2.putText(vis, f"FPS: {fps:.1f}",
                            (10, vis.shape[0] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
                cv2.imshow('HSV Detection', vis)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            cv2.destroyAllWindows()
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        cam.shutdown()
