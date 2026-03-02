"""
depth_segmenter.py - Depth-Based Obstacle and Object Segmentation
Overhead Perception System
Author: Aaron Fraze
Date: February 23, 2026

Refactored from hsv_v2_with_depth_FIXED.py
    (detect_elevated_objects_depth + depth_segmentation_demo)

Changes from original:
    - Camera code removed; takes RealSenseCamera from camera.py
    - Floor plane calibration: calibrate() averages N frames of a clear floor
      once at startup, instead of estimating floor from median every frame.
      This handles slight camera tilt naturally and is more stable.
    - Size classification: sorts blobs into 'ball', 'robot', 'obstacle'
      by pixel area so WorldState doesn't have to guess
    - Arena boundary support: define workspace corners once; detections
      outside that polygon are ignored (rejects tripod legs, walls outside arena)
    - Confidence score per detection
    - Output format standardised for WorldState

Usage:
    # Standalone demo:
    python depth_segmenter.py

    # As a module:
    from camera import RealSenseCamera
    from depth_segmenter import DepthSegmenter

    cam = RealSenseCamera(camera_height_cm=220.0)
    seg = DepthSegmenter(cam)
    seg.calibrate()                    # run once with clear floor
    results = seg.detect(frame_data)   # frame_data from cam.get_frame()
    for obj in results['elevated_objects']:
        print(obj['class'], obj['position_world'])
"""

import time
import numpy as np
import cv2

from camera import RealSenseCamera


# ── Size classification thresholds ───────────────────────────────────────────
# At 220 cm height, 1280x720:
#   Ball   ~7 cm diameter  -> ~37 px diameter -> ~1100 px area
#   HamBot ~18 cm wide     -> ~95 px wide     -> ~7000 px area (rough)
#   Wall / large obstacle  -> much larger
#
# Tune these after seeing your first detections.

SIZE_CLASSES = {
    # name      : (min_area_px, max_area_px)
    'ball'      : (300,   3000),
    'robot'     : (3000,  20000),
    'obstacle'  : (20000, 500000),
}


class DepthSegmenter:
    """
    Detects elevated objects by subtracting a calibrated floor depth map.

    Algorithm:
        1. calibrate() -- average N frames of a clear floor -> floor_map
        2. detect()    -- per frame:
              elevation = floor_map - current_depth
              mask      = elevation >= height_threshold
              contours  -> filter by area -> classify by size -> world position

    Primary use: obstacle/wall detection (no colour or ArUco marker needed)
    Secondary use: robot fallback when ArUco fails at high speed
    """

    def __init__(
        self,
        camera: RealSenseCamera,
        height_threshold_cm: float = 5.0,
        min_area_px: int = 300,
    ):
        """
        Args:
            camera:               Initialized RealSenseCamera instance.
            height_threshold_cm:  Minimum cm above floor to count as elevated.
                                  5 cm catches the ball and robot while
                                  rejecting floor-level sensor noise.
                                  Adjust with the live demo trackbar.
            min_area_px:          Minimum contour area. Rejects single-pixel clusters.
        """
        self.cam                 = camera
        self.height_threshold_cm = height_threshold_cm
        self.min_area_px         = min_area_px
        self.floor_map           = None   # set by calibrate()
        self.arena_mask          = None   # set by set_arena_boundary() (optional)
        print(
            f"[Depth] Initialized  "
            f"threshold={height_threshold_cm}cm  min_area={min_area_px}px"
        )

    # ── Floor calibration ─────────────────────────────────────────────────────

    def calibrate(self, num_frames: int = 60, verbose: bool = True) -> bool:
        """
        Build a per-pixel floor depth map by averaging frames of a clear floor.

        Why this beats the per-frame median approach in the original:
            - Not biased by objects currently on the floor
            - Captures true depth at every pixel, handles slight camera tilt
            - Computed once at startup; used every frame with no overhead

        Args:
            num_frames: Frames to average. 60 = 2 seconds at 30 FPS.
            verbose:    Print progress.

        Returns:
            True on success, False if too few valid frames captured.
        """
        if verbose:
            print(f"\n[Depth] Calibrating floor ({num_frames} frames)")
            print("[Depth] Clear the floor of all objects.")
            input("[Depth] Press ENTER when ready...")

        accumulator = None
        count       = 0

        for i in range(num_frames):
            fd = self.cam.get_frame()
            if fd is None:
                continue
            depth_m              = fd['depth_image'].astype(np.float32) * self.cam.depth_scale
            depth_m[depth_m == 0] = np.nan   # invalid pixels -> NaN (excluded from mean)

            if accumulator is None:
                accumulator = np.zeros(
                    (num_frames, depth_m.shape[0], depth_m.shape[1]), np.float32
                )
            accumulator[count] = depth_m
            count += 1
            if verbose and (i + 1) % 15 == 0:
                print(f"[Depth]   {i + 1}/{num_frames}")

        if count < 10:
            print("[Depth] ERROR: too few valid frames for calibration.")
            return False

        self.floor_map = np.nanmean(accumulator[:count], axis=0)

        if verbose:
            valid      = self.floor_map[~np.isnan(self.floor_map)]
            mean_depth = np.mean(valid)
            print(f"[Depth] Calibration done  floor={mean_depth:.3f}m  valid_px={len(valid):,}")

        return True

    def set_arena_boundary(self, corners_pixel: list):
        """
        Ignore detections outside a polygon defined by pixel corners.

        Useful for rejecting the tripod legs, walls outside the arena, or
        anything else that appears elevated but is not in the workspace.

        Args:
            corners_pixel: List of (x, y) tuples in pixel coordinates.
                           Clockwise or counter-clockwise, any convex/concave shape.
                           Example rectangle:
                               [(100, 80), (1180, 80), (1180, 640), (100, 640)]

        Call after calibrate() so the image shape is known.
        """
        if self.floor_map is None:
            print("[Depth] WARNING: call calibrate() before set_arena_boundary().")
            return
        h, w = self.floor_map.shape
        pts  = np.array(corners_pixel, dtype=np.int32)
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask, [pts], 255)
        self.arena_mask = mask
        print(f"[Depth] Arena boundary set  ({len(corners_pixel)} corners)")

    # ── Per-frame detection ───────────────────────────────────────────────────

    def detect(self, frame_data: dict, height_threshold_cm: float = None) -> dict:
        """
        Detect elevated objects in one frame.

        Args:
            frame_data:          dict from RealSenseCamera.get_frame()
            height_threshold_cm: Override instance threshold for this call.
                                 Used by live demo trackbar.

        Returns:
            {
                'elevated_objects':    list of detection dicts (may be empty),
                'elevation_mask':      np.ndarray uint8 -- white = elevated,
                'floor_depth_m':       float,
                'height_threshold_cm': float,
            }

        Each detection dict:
            class                'ball', 'robot', 'obstacle', or 'unknown'
            centroid_pixel       (int, int)
            bounding_box         (x, y, w, h) pixels
            area                 float  pixels^2
            height_above_floor_cm float
            depth_value          int   raw uint16 at centroid
            position_world       (x_cm, y_cm, z_cm) or None
            confidence           float 0.0-1.0
            contour              np.ndarray
        """
        threshold   = height_threshold_cm if height_threshold_cm is not None \
                      else self.height_threshold_cm
        depth_image = frame_data['depth_image']
        depth_m     = depth_image.astype(np.float32) * self.cam.depth_scale

        # Floor reference
        if self.floor_map is not None:
            floor_ref     = self.floor_map
            floor_depth_m = float(np.nanmean(floor_ref))
        else:
            # Fallback: per-frame median (works without calibrate())
            valid         = depth_m[depth_m > 0]
            floor_depth_m = float(np.median(valid)) if len(valid) > 100 else 0.0
            floor_ref     = np.full_like(depth_m, floor_depth_m)

        # Elevation map: positive = object is closer to camera = elevated
        elevation_m   = floor_ref - depth_m
        threshold_m   = threshold / 100.0

        elevated_mask = np.zeros(depth_image.shape, dtype=np.uint8)
        elevated_mask[(depth_m > 0) & (elevation_m >= threshold_m)] = 255

        # Apply arena boundary if defined
        if self.arena_mask is not None:
            elevated_mask = cv2.bitwise_and(elevated_mask, self.arena_mask)

        # Morphological cleanup (same as original detect_elevated_objects_depth)
        kernel        = np.ones((5, 5), np.uint8)
        elevated_mask = cv2.morphologyEx(elevated_mask, cv2.MORPH_OPEN,  kernel)
        elevated_mask = cv2.morphologyEx(elevated_mask, cv2.MORPH_CLOSE, kernel)

        # Contours
        contours, _ = cv2.findContours(
            elevated_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        detected_objects = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area_px:
                continue

            M = cv2.moments(cnt)
            if M['m00'] == 0:
                continue
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            x, y, w, h = cv2.boundingRect(cnt)

            cx_s        = int(np.clip(cx, 0, depth_image.shape[1] - 1))
            cy_s        = int(np.clip(cy, 0, depth_image.shape[0] - 1))
            depth_value = int(depth_image[cy_s, cx_s])

            # Height at this specific pixel using per-pixel floor map
            if depth_value > 0:
                floor_at = float(floor_ref[cy_s, cx_s]) \
                           if not np.isnan(floor_ref[cy_s, cx_s]) \
                           else floor_depth_m
                height_cm = (floor_at - depth_value * self.cam.depth_scale) * 100.0
            else:
                height_cm = 0.0

            position_world = self.cam.pixel_to_world(cx_s, cy_s, depth_value)
            obj_class      = self._classify(area)
            confidence     = self._confidence(area, depth_value, height_cm)

            detected_objects.append({
                'class':                  obj_class,
                'centroid_pixel':         (cx, cy),
                'bounding_box':           (x, y, w, h),
                'area':                   area,
                'height_above_floor_cm':  height_cm,
                'depth_value':            depth_value,
                'position_world':         position_world,
                'confidence':             confidence,
                'contour':                cnt,
            })

        # Largest first (robot is typically the largest elevated object)
        detected_objects.sort(key=lambda o: o['area'], reverse=True)

        return {
            'elevated_objects':    detected_objects,
            'elevation_mask':      elevated_mask,
            'floor_depth_m':       floor_depth_m,
            'height_threshold_cm': threshold,
        }

    # ── Classification ────────────────────────────────────────────────────────

    @staticmethod
    def _classify(area: float) -> str:
        for name, (lo, hi) in SIZE_CLASSES.items():
            if lo <= area < hi:
                return name
        return 'unknown'

    # ── Confidence scoring ────────────────────────────────────────────────────

    def _confidence(self, area: float, depth_value: int, height_cm: float) -> float:
        """
        Confidence 0.0-1.0 from three components:
            depth_valid   valid reading at centroid
            height_range  2-60 cm is physically sensible for arena objects
            area_size     penalises tiny noise clusters and enormous regions
        """
        depth_s  = 1.0 if depth_value > 0 else 0.0

        if 2.0 <= height_cm <= 60.0:
            height_s = 1.0
        elif height_cm < 2.0:
            height_s = height_cm / 2.0
        else:
            height_s = max(0.0, 1.0 - (height_cm - 60.0) / 40.0)

        if 500 <= area <= 50000:
            area_s = 1.0
        elif area < 500:
            area_s = area / 500.0
        else:
            area_s = max(0.0, 1.0 - (area - 50000) / 50000.0)

        return float(np.clip(depth_s * 0.4 + height_s * 0.35 + area_s * 0.25, 0.0, 1.0))

    # ── Visualisation ─────────────────────────────────────────────────────────

    def draw_detections(self, image: np.ndarray, results: dict) -> np.ndarray:
        """Annotate a copy of image with depth segmentation results."""
        vis = image.copy()
        COLOURS = {
            'ball':     (0,   165, 255),
            'robot':    (0,   220, 0),
            'obstacle': (0,   0,   220),
            'unknown':  (150, 150, 150),
        }
        for obj in results['elevated_objects']:
            cx, cy     = obj['centroid_pixel']
            x, y, w, h = obj['bounding_box']
            cls        = obj['class']
            color      = COLOURS.get(cls, (150, 150, 150))

            cv2.rectangle(vis, (x, y), (x + w, y + h), color, 2)
            cv2.circle(vis, (cx, cy), 5, (255, 255, 255), -1)
            cv2.putText(vis, f"{cls}  h={obj['height_above_floor_cm']:.1f}cm",
                        (x, y - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            cv2.putText(vis, f"conf={obj['confidence']:.2f}  area={obj['area']:.0f}",
                        (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)
            if obj['position_world']:
                wx, wy, _ = obj['position_world']
                cv2.putText(vis, f"({wx:.0f}, {wy:.0f}) cm",
                            (x, y + h + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 0), 1)

        cv2.putText(vis,
                    (f"Depth seg: {len(results['elevated_objects'])} objects  "
                     f"floor={results['floor_depth_m']:.2f}m  "
                     f"thresh={results['height_threshold_cm']:.0f}cm"),
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        return vis

    # ── Interactive demo ──────────────────────────────────────────────────────

    def live_demo(self):
        """
        Live demo with trackbars. Refactored from hsv_v2 depth_segmentation_demo().
        Adds calibrated floor map, object classification, and confidence scores.

        Keys:  c=recalibrate floor   q=quit
        """
        print("\n" + "=" * 60)
        print("DEPTH SEGMENTATION LIVE DEMO")
        print("  c=recalibrate floor   q=quit")
        print("=" * 60)

        if self.floor_map is None:
            self.calibrate()

        WIN_VIS   = 'Depth Segmentation'
        WIN_MASK  = 'Elevation Mask'
        WIN_DEPTH = 'Depth Colormap'

        def make_windows(thresh, min_area):
            cv2.namedWindow(WIN_VIS)
            cv2.namedWindow(WIN_MASK)
            cv2.namedWindow(WIN_DEPTH)
            cv2.createTrackbar('Height (cm)', WIN_VIS, thresh,    50,   lambda x: None)
            cv2.createTrackbar('Min Area',    WIN_VIS, min_area,  5000, lambda x: None)

        make_windows(int(self.height_threshold_cm), self.min_area_px)

        while True:
            fd = self.cam.get_frame()
            if fd is None:
                continue

            thresh   = max(1, cv2.getTrackbarPos('Height (cm)', WIN_VIS))
            min_area = max(1, cv2.getTrackbarPos('Min Area',    WIN_VIS))
            self.min_area_px = min_area

            results = self.detect(fd, height_threshold_cm=thresh)
            vis     = self.draw_detections(fd['color_image'], results)

            cv2.imshow(WIN_VIS,  vis)
            cv2.imshow(WIN_MASK, results['elevation_mask'])

            # Colorised depth view (same normalisation as original demo)
            d = fd['depth_image'].copy().astype(np.float32)
            d[d == 0] = np.nan
            valid = d[~np.isnan(d)]
            if len(valid) > 0:
                lo = np.percentile(valid, 1)
                hi = np.percentile(valid, 99)
                d  = np.clip(d, lo, hi)
                d  = ((d - lo) / (hi - lo) * 255)
                d  = np.nan_to_num(d).astype(np.uint8)
            else:
                d  = np.zeros_like(fd['depth_image'], dtype=np.uint8)
            cv2.imshow(WIN_DEPTH, cv2.applyColorMap(d, cv2.COLORMAP_JET))

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('c'):
                cv2.destroyAllWindows()
                self.calibrate()
                make_windows(thresh, min_area)

        cv2.destroyAllWindows()


# ── Standalone entry point ────────────────────────────────────────────────────
if __name__ == '__main__':
    print("=" * 60)
    print("depth_segmenter.py — Standalone Demo")
    print("=" * 60)

    cam = RealSenseCamera(resolution='1280x720', camera_height_cm=220.0)
    seg = DepthSegmenter(cam, height_threshold_cm=5.0, min_area_px=300)

    try:
        seg.live_demo()
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        cam.shutdown()
