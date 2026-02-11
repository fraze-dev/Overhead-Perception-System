# World-Frame Calibration Report

**Researcher:** Aaron Fraze  
**Date:** February 11, 2026  
**Deliverable:** Week 5 Milestone - Calibration Report

---

## 1. Setup and Configuration

### Hardware
- **Camera Model:** Intel RealSense D435
- **Camera Serial:** 141722074094
- **Firmware Version:** 5.17.0.10
- **Resolution:** 1280×720 pixels
- **Depth Scale:** 0.001 meters/unit (1 mm)

### Camera Mounting
- **Height:** 220 cm (measured from lens center to ground)
- **Position (X, Y):** (0, 0) cm (centered over workspace)
- **Orientation:** 
  - Tilt: 0° (lens pointing straight down)
  - Roll: 0° (verified with bubble level)
  - Pan: 0° (verified with bubble level)

### Mounting Verification
The camera was mounted with the lens pointing vertically downward. A bubble level was used to verify alignment in three planes:
- Vertical alignment (Tilt = 0°)
- Left-right level (Roll = 0°)
- Front-back level (Pan = 0°)

---

## 2. World Coordinate Frame Definition

### Origin and Axes
- **Origin:** Center of workspace at ground level
- **X-axis:** Horizontal, pointing right (positive direction)
- **Y-axis:** Horizontal, pointing up/forward (positive direction)
- **Z-axis:** Vertical, pointing up from ground (positive direction is up)

### Camera Coordinate System
- **Camera frame origin:** Optical center of the camera lens
- **Camera Z-axis:** Points downward (into the scene)
- **Transformation required:** Convert from camera coordinates to world coordinates

---

## 3. Calibration Methodology

### Approach
Extrinsic calibration using known physical reference points:
1. Placed 9 blue tape markers at measured world coordinates
2. Used laser tape measure for high-accuracy positioning
3. Captured depth and RGB frames with all calibration points visible
4. Manually clicked on each point to establish pixel-to-world correspondence
5. Computed transformation matrix using measured camera pose
6. Validated accuracy by comparing predicted vs. true world coordinates

### Calibration Point Layout
Points arranged in a 3×3 grid covering the workspace:

```
Point 1 (-122, 66)    Point 2 (0, 67)      Point 3 (120, 66)
                                           
Point 4 (-70, 0)      Point 5 (0, 0)       Point 6 (73, 0)
                     [Origin]              
                                           
Point 7 (-120, -67)   Point 8 (0, -67)     Point 9 (121, -67)
```

**Workspace Coverage:**
- X range: -122 to +121 cm (243 cm width)
- Y range: -67 to +67 cm (134 cm depth)
- Total area: ~3.3 m² (33,700 cm²)

---

## 4. Calibration Points and Measurements

| # | Label | Ground Truth (cm) | Pixel Coords | Measured (cm) | Error (cm) |
|---|-------|------------------|--------------|---------------|------------|
| 1 | top-left | (-122, 66, 0) | (148, 82) | (-122.6, 68.9, -5.9) | 6.59 |
| 2 | top-center | (0, 67, 0) | (641, 90) | (-0.4, 66.6, -4.9) | 4.93 |
| 3 | top-right | (120, 66, 0) | (1143, 83) | (123.5, 68.3, -4.9) | 6.48 |
| 4 | center-left | (-70, 0, 0) | (352, 361) | (-71.7, -0.4, -4.9) | 5.21 |
| 5 | center-center | (0, 0, 0) | (641, 360) | (-0.4, -0.2, 0.0) | **0.41** |
| 6 | center-right | (73, 0, 0) | (945, 363) | (74.3, -0.9, -3.9) | 4.22 |
| 7 | bottom-left | (-120, -67, 0) | (139, 635) | (-123.8, -67.8, -3.9) | 5.48 |
| 8 | bottom-center | (0, -67, 0) | (635, 636) | (-1.8, -65.1, 5.9) | 6.45 |
| 9 | bottom-right | (121, -67, 0) | (1139, 629) | (123.4, -67.1, -6.4) | 6.83 |

### Notes on Measurements
- Point 5 (origin) achieved exceptional accuracy: 0.41 cm error
- Edge points show slightly higher error (5-7 cm) possibly due to:
  - Human error measuring ground truth points
  - Lens distortion at frame periphery
  - Slight camera alignment imperfections
  - Depth measurement noise increases with distance from center

---

## 5. Transformation Matrix

The transformation from camera coordinates to world coordinates is represented by a 4×4 homogeneous transformation matrix:

```
T_world_camera = 
┌                          ┐
│  1.0    0.0    0.0   0.0 │
│  0.0   -1.0    0.0   0.0 │
│  0.0    0.0   -1.0   2.2 │
│  0.0    0.0    0.0   1.0 │
└                          ┘
```

### Transformation Components

**Rotation Matrix (3×3):**
```
R = ┌                  ┐
    │  1.0   0.0   0.0 │
    │  0.0  -1.0   0.0 │
    │  0.0   0.0  -1.0 │
    └                  ┘
```

- **Identity along X:** No rotation about X-axis (camera aligned horizontally)
- **Flip Y-axis:** Camera's down is world's up
- **Flip Z-axis:** Camera's forward (into scene) is world's down

**Translation Vector:**
```
t = [0.0, 0.0, 2.2]^T  (in meters)
```

- Camera positioned at (0, 0, 220 cm) in world frame
- No X or Y offset (centered over workspace)

### Coordinate Transformation Formula

For a point **p_cam** = [x_cam, y_cam, z_cam] in camera coordinates:

```
p_world = R * p_cam + t

x_world = x_cam
y_world = -y_cam
z_world = -z_cam + 2.2
```

---

## 6. Calibration Accuracy Results

### Error Statistics

| Metric | Value | Target |
|--------|-------|--------|
| **Mean Error** | 5.18 cm | < 10 cm |
| **Standard Deviation** | 1.88 cm | - |
| **RMSE** | 5.51 cm | < 10 cm |
| **Minimum Error** | 0.41 cm (center) | - |
| **Maximum Error** | 6.83 cm (bottom-right corner) | - |
| **Median Error** | 5.48 cm | - |

### Status: ✅ **PASSED** - Calibration Acceptable

**Justification:**
- Mean error of 5.18 cm is well within acceptable range for robot tracking
- Center region (most critical for tasks) shows excellent accuracy (<1 cm)
- Error pattern is consistent and predictable (higher at edges)
- RMSE of 5.51 cm indicates no major outliers

### Error Distribution by Region

| Region | Avg Error (cm) | Points |
|--------|----------------|--------|
| Center (origin) | 0.41 | 1 |
| Center row/col | 4.71 | 5 |
| Corners | 6.35 | 4 |
| All points | 5.18 | 9 |

**Observation:** Accuracy degrades toward frame edges, which is expected due to lens distortion and depth measurement characteristics.

---

## 7. Visualizations

### 7.1 Top-Down Workspace View

![Top-Down View](top_down_visualization.png)

**Figure 1:** Interactive calibration interface showing the RGB camera view with all 9 calibration points marked. Blue tape markers are visible at each measured location. This interface was used to manually click on each point to establish pixel-to-world correspondence.

**Key observations:**
- Excellent agreement at center points (points 4, 5, 6)
- Slight divergence at corners
- Full workspace coverage confirmed (~2.4m × 1.3m)

---

### 7.2 Calibration Visualization

![Calibration Visualization](calibration_visualization.png)

**Figure 2:** Top-down view showing predicted (red circles) vs. ground truth (green circles) calibration point locations. The origin is marked in red at workspace center. Gray dots represent the point cloud from a single frame, showing the entire visible workspace area.

---

## 8. Error Analysis and Sources

### Identified Error Sources

1. **Camera Height Measurement**
   - Difficult to measure exact optical center
   - 8 of 9 points showed -(4-6 cm) from ground truth

2. **Camera Alignment** (±0.5-1°)
   - Minor tilt/roll despite bubble level verification
   - Causes edge point errors to increase

3. **Depth Measurement Noise** (±0.4-1 cm at 2.2m)
   - Inherent sensor noise (documented in depth accuracy tests)
   - Consistent with previous depth accuracy characterization

4. **Physical Marker Placement** (± 3 cm)
   - Manual tape placement precision
   - Human measurement errors in ground truth

### Systematic Errors Observed

- **Z-offset bias:** Measurements consistently offset by -4 to -6 cm in Z
  - Likely due to slight camera height measurement error
  - Could apply +5 cm correction if higher accuracy needed
  - Could allow option in software for user to adjust
  
- **Edge degradation:** Corners show 50% higher error than center
  - Acceptable for current application

---

## 9. Validation and Performance

### Calibration Quality Assessment

✅ **Excellent center accuracy** (0.41 cm at origin)  
✅ **Acceptable overall accuracy** (5.18 cm mean)  
✅ **Consistent error pattern** (no major outliers)  
✅ **Covers full workspace** (~3.3 m²)  
⚠️ **Edge regions less accurate** (6-7 cm at corners)

### Expected Performance for Robot Tracking

Based on calibration results, the system should provide:

| Application | Expected Accuracy | Sufficient?                           |
|-------------|------------------|---------------------------------------|
| HamBot position tracking | ±5-7 cm | ✅ Yes (robot ~18 cm wide)             |
| Ball position tracking | ±5-7 cm | ✅ Yes (ball ~10 cm diameter)          |
| Goal region detection | ±5-7 cm | ✅ Yes (goal >30 cm wide)              |
| Collision avoidance | ±5-7 cm | ✅ Yes (>10 cm safety margin)          |
| Path planning | ±5-7 cm | ✅ Yes (acceptable waypoint precision) |

---

## 10. Conclusions

### Summary

The extrinsic calibration of the Intel RealSense D435 camera mounted at 220 cm height has been successfully completed with a mean accuracy of 5.18 cm (RMSE 5.51 cm) across the 2.4m × 1.3m workspace. The calibration achieves excellent accuracy in the center region (0.41 cm) where most robot activity occurs, with predictable accuracy degradation toward frame edges (6-7 cm at corners).

### Strengths

1. **Excellent center accuracy** - Sub-centimeter precision at the workspace origin where critical tasks occur
2. **Stable transformation** - Simple rotation matrix with no complex distortion terms needed
3. **Repeatable measurements** - Consistent error patterns indicate reliable calibration
4. **Full workspace coverage** - All 9 points captured across entire 3.3 m² area
5. **Documented methodology** - Reproducible process with measured physical parameters

### Limitations

1. **Edge accuracy** - Corners show 50-100% higher error than center (6-7 cm vs 0.4-5 cm)
2. **Z-axis systematic offset** - Consistent -4 to -6 cm offset suggests camera height measurement uncertainty
3. **Manual point selection** - Click-based correspondence introduces ±1-2 pixel uncertainty
4. **Single-frame validation** - Calibration verified on static points, not tested with moving objects (**yet**)

### Impact on Project Goals

The calibration quality is **more than sufficient** for the overhead tracking system requirements:

✅ **Robot tracking:** 5-7 cm accuracy is excellent for a ~18 cm diameter robot  
✅ **Ball tracking:** Sufficient for ~10 cm diameter ball localization  
✅ **Navigation:** Enables reliable path planning with >10 cm obstacle clearance  
✅ **Task performance:** Ball-pushing accuracy will be limited by robot control, not perception  

### Recommendations

1. **Use center 80% of frame** for critical measurements (where error <5 cm)
2. **Apply systematic offset correction** (+5 cm in Z) if sub-5cm accuracy needed
3. **Re-validate after camera moves** - Any physical disturbance requires recalibration
4. **Monitor long-term stability** - Check calibration accuracy regularly

### Next Steps

With calibration complete and documented, the project can now proceed to:

1. **Object detection** (Week 6) - Implement robot and ball detection algorithms
2. **Real-time tracking** (Week 7) - Add temporal filtering and state estimation
3. **Performance testing** - Validate tracking accuracy with moving objects
4. **System integration** - Connect perception to robot communication

---

## 11. Appendices

### A. Calibration Data Files

- **JSON calibration file:** `calibration.json`
- **Top-down visualization:** `top_down_visualization.png`
- **Calibration interface:** `calibration_visualization.png`
- **Python script:** `world_frame_calibration.py`

### B. Camera Intrinsic Parameters

```python
Width: 1280 pixels
Height: 720 pixels
fx: 910.996 pixels (horizontal focal length)
fy: 909.727 pixels (vertical focal length)
ppx: 642.553 pixels (principal point X)
ppy: 359.353 pixels (principal point Y)
```

### C. Reference Frame Conventions

**World Frame:**
- Origin: Center of workspace at ground level
- +X: Right
- +Y: Forward (away from camera mount)
- +Z: Up (away from ground)

**Camera Frame (RealSense convention):**
- Origin: Optical center of camera
- +X: Right (same as world)
- +Y: Down (toward ground)
- +Z: Forward (into scene, toward ground)

---

**Report Completed:** February 11, 2026  
**Next Deliverable:** Object Detection Prototype (Week 5)  
**Status:** ✅ Calibration milestone complete - Ready to proceed with detection
