# World-Frame Calibration Guide
## Intel RealSense Overhead Tracking System

**Author:** Aaron Fraze  
**Date:** February 3, 2026  
**Week:** 4 of 16

---

## Overview

This guide explains how to calibrate your overhead RealSense camera to accurately transform depth measurements into real-world coordinates. This calibration is essential for tracking objects in your workspace.

---

## 1. Understanding Coordinate Systems

### Camera Frame (RealSense Native)
- **Origin**: Camera optical center (lens)
- **X-axis**: Points right (→) in the image
- **Y-axis**: Points down (↓) in the image  
- **Z-axis**: Points forward into the scene (depth direction)
- **Units**: Meters

### World Frame (Your Workspace)
- **Origin**: You define it! (e.g., corner of arena, center of workspace)
- **X-axis**: Length of workspace (e.g., toward goal)
- **Y-axis**: Width of workspace (perpendicular to X)
- **Z-axis**: Height above ground (Z=0 is typically ground level)
- **Units**: Centimeters (for convenience)

### Why Two Frames?
The camera sees everything relative to itself, but you want positions relative to your workspace. Calibration bridges the gap.

---

## 2. The Calibration Pipeline

```
Pixel Coordinates (u, v)
         |
         | + Depth value
         ↓
    [Intrinsic Parameters]
         ↓
Camera Frame (X_cam, Y_cam, Z_cam)
         |
         | [Extrinsic Calibration]
         ↓
World Frame (X_world, Y_world, Z_world)
```

### Step 1: Pixel → Camera Frame
Uses camera **intrinsic parameters** (focal length, principal point):
- Already calibrated at factory
- Extracted from RealSense SDK
- Converts 2D pixel + depth → 3D point in camera frame

### Step 2: Camera Frame → World Frame  
Uses **extrinsic parameters** (rotation + translation):
- **This is what you're calibrating!**
- Defines where the camera is in the world
- Defines how the camera is oriented

---

## 3. Extrinsic Calibration Parameters

You need to define 6 parameters (6 degrees of freedom):

### Position (Translation) - 3 DOF
1. **X_cam**: Camera X position in world frame (cm)
2. **Y_cam**: Camera Y position in world frame (cm)  
3. **Z_cam**: Camera height above ground (cm)

### Orientation (Rotation) - 3 DOF
4. **Tilt** (pitch, rotation about X-axis): 0° = straight down
5. **Pan** (yaw, rotation about Y-axis): 0° = aligned with world
6. **Roll** (rotation about Z-axis): 0° = level with world

---

## 4. Simplified Overhead Case

For an overhead camera pointing straight down, calibration is simpler:

### Assumptions
- Camera is roughly centered above workspace
- Camera points straight down (or nearly so)
- Small tilts/rotations are acceptable

### Required Measurements
1. **Camera height**: Measure with tape measure from lens to ground
2. **Camera X, Y position**: Measure offset from your chosen origin
3. **Tilt angle**: Estimate or measure with level/protractor (usually small)

### Transformation Matrix
For overhead camera, the transformation is:

```
[X_world]   [R11 R12 R13] [X_cam]   [X_cam_pos]
[Y_world] = [R21 R22 R23] [Y_cam] + [Y_cam_pos]
[Z_world]   [R31 R32 R33] [Z_cam]   [Z_cam_pos]
```

Where:
- **R** is the 3×3 rotation matrix (computed from tilt, pan, roll)
- **Translation** is camera position in world frame

---

## 5. Calibration Procedure

### Method 1: Simple Measurement (Quick)

**Pros**: Fast, no special equipment  
**Cons**: Less accurate (~2-5 cm error typical)

**Steps**:
1. Mount camera overhead
2. Measure height with tape measure  
3. Estimate camera position (X, Y) relative to workspace origin
4. Estimate tilt/pan/roll if camera isn't perfectly level
5. Input these values into `define_simple_overhead_calibration()`

**When to use**: Initial setup, rough validation, low precision requirements

### Method 2: Calibration Points (Accurate)

**Pros**: High accuracy (~0.5-2 cm error possible)  
**Cons**: Requires placing markers and clicking points

**Steps**:
1. Start with simple measurement as initial guess
2. Place 4-6 objects at **known world coordinates**
   - Use corners of workspace
   - Use tape measure to find exact positions
   - Include points at different heights if possible
3. Run interactive calibration:
   - Click on each object in camera view
   - Enter its true world coordinates
   - System computes transformation error
4. Refine parameters if error is high
5. Save validated calibration

**When to use**: Final calibration, high precision tracking needed

---

## 6. Choosing Calibration Points

### Good Calibration Points
- ✓ Distributed across workspace (all 4 corners + center)
- ✓ Easy to identify precisely in camera view  
- ✓ Flat objects or markers on ground plane
- ✓ High contrast with background
- ✓ Stable, won't move during calibration

### Examples
- Colored markers/tape at measured positions
- Small colored balls placed at grid intersections  
- Corners of the workspace itself
- AprilTags (if you have them)

### How Many Points?
- **Minimum**: 4 points (overdetermined system)
- **Recommended**: 6-8 points
- **More points**: Better error averaging, catches systematic errors

---

## 7. Validation and Error Metrics

After calibration, you'll see these metrics:

### Mean Absolute Error (MAE)
- Average distance between predicted and true positions
- **Target**: <2 cm for robot tracking, <5 cm for ball tracking
- If higher: Check measurements, add more calibration points

### Root Mean Squared Error (RMSE)  
- Emphasizes large errors more than MAE
- Should be close to MAE if errors are consistent
- If RMSE >> MAE: You have outliers, check for bad points

### Max Error
- Largest error among all calibration points
- Should be <2× MAE
- If one point has huge error: Likely measurement mistake

### What Error is Acceptable?
Based on your depth testing results:
- Camera precision: ±0.36 cm at 2m
- Systematic bias: +0.5 to +2 cm
- **Realistic goal**: 1-3 cm calibration error
- **Acceptable**: <5 cm for your application (robot is ~10-20 cm wide)

---

## 8. Troubleshooting Common Issues

### High Calibration Error (>5 cm)

**Possible causes**:
1. Incorrect camera height measurement
   - **Fix**: Re-measure carefully from lens center to ground
2. Significant camera tilt not accounted for
   - **Fix**: Estimate tilt angle, adjust in calibration
3. Calibration point positions measured incorrectly  
   - **Fix**: Re-measure with tape measure, ensure origin is correct
4. Depth measurements have large noise/errors at edges
   - **Fix**: Use center 80% of image, avoid edges

### Systematic Error (all points off by similar amount)

**Likely cause**: Camera position (X, Y, Z) is wrong

**Fix**: Adjust translation parameters, re-validate

### Non-uniform Error (some points good, others bad)

**Likely cause**: Rotation (tilt/pan/roll) is incorrect

**Fix**: Adjust orientation parameters, ensure camera is level

### Z-coordinate is Wrong (X, Y look okay)

**Likely cause**: Camera height measurement is off

**Fix**: Double-check height measurement

---

## 9. Using the Calibration Scripts

### Quick Test (No Manual Input)

```bash
python test_calibration_simple.py
```

- Uses default calibration (200 cm height, centered, straight down)
- Good for seeing if the system works
- Shows live visualization

### Interactive Calibration Session

```bash
python world_frame_calibration.py
```

- Walks you through the entire process
- Prompts for camera measurements
- Allows you to click calibration points
- Validates and saves calibration

### Loading Saved Calibration

```python
from world_frame_calibration import WorldFrameCalibrator

calibrator = WorldFrameCalibrator()
calibrator.load_calibration("calibration.json")

# Now you can use it for transformations
point_world = calibrator.camera_to_world_coordinates(point_camera)
```

---

## 10. Integration with Object Detection

Once calibrated, you can:

1. **Detect objects** in camera frame (Week 5-6)
   - Find pixel coordinates of robot, ball, obstacles
   
2. **Get depth** at detected positions
   - Use depth frame to get Z distance

3. **Transform to world coordinates**
   - `pixel_to_camera_coordinates()` → camera frame
   - `camera_to_world_coordinates()` → world frame

4. **Track in world frame** (Week 7)
   - All objects now in common coordinate system
   - Can compute distances, velocities, trajectories

---

## 11. Best Practices

### Before Calibration
- ✓ Ensure camera is securely mounted (won't shift)
- ✓ Define your world coordinate system clearly
- ✓ Mark your origin with tape or marker
- ✓ Have tape measure and level handy

### During Calibration  
- ✓ Take measurements carefully (±1 cm accuracy)
- ✓ Use bright, contrasting calibration objects
- ✓ Place calibration points across entire workspace
- ✓ Click precisely on center of objects

### After Calibration
- ✓ Save calibration file with descriptive name
- ✓ Validate with fresh points not used in calibration
- ✓ Re-calibrate if camera is moved or bumped
- ✓ Document any changes to setup

---

## 12. Expected Results

### From Your Depth Testing
- Camera accuracy at 2m: 1.1% (±2.2 cm)
- Precision: ±0.36 cm std dev
- No calibration needed for depth itself

### Expected Calibration Accuracy
- **With simple measurement**: 2-5 cm error typical
- **With calibration points**: 1-3 cm error achievable  
- **Best case**: <1 cm with careful calibration

### This is Good Enough Because:
- Robot is ~10-20 cm in size → 2-3 cm error is <20% of size
- Ball is ~5-10 cm → 2-3 cm error is acceptable
- Temporal filtering will smooth tracking noise
- You can refine based on observed performance

---

## 13. Next Steps After Calibration

### Week 4 Deliverable
✓ **Calibration Report** (1-2 pages + figures):
- Document your calibration methodology
- Include transformation matrices
- Report accuracy measurements
- Show top-down visualization

### Week 5: Object Detection
- Use calibrated system to detect and localize objects
- Report positions in world coordinates
- Verify detection accuracy using calibration

### Week 7: Tracking  
- Track objects over time in world frame
- Compute velocities and trajectories
- All in meaningful real-world units

---

## 14. Calibration Checklist

Before your Week 4 meeting, ensure you have:

- [ ] Camera securely mounted overhead
- [ ] World frame origin and axes defined
- [ ] Camera height measured accurately
- [ ] Camera position (X, Y) measured or estimated
- [ ] Initial calibration parameters entered in code
- [ ] 4-6 calibration points placed at known positions
- [ ] Interactive calibration run successfully  
- [ ] Calibration error computed (<5 cm goal)
- [ ] Top-down visualization generated
- [ ] Calibration saved to file
- [ ] Test transformations validated

---

## 15. Code Example: Complete Workflow

```python
import pyrealsense2 as rs
from world_frame_calibration import WorldFrameCalibrator

# Initialize camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
profile = pipeline.start(config)

# Initialize calibrator
calibrator = WorldFrameCalibrator()
calibrator.setup_camera(pipeline)

# Define calibration (using your measurements)
calibrator.define_simple_overhead_calibration(
    camera_height_cm=210.0,    # Measure this!
    camera_x_world=0.0,         # Relative to origin
    camera_y_world=0.0,
    camera_tilt_deg=2.0,        # Estimate if not level
    camera_pan_deg=0.0,
    camera_roll_deg=0.0
)

# Get a frame
frames = pipeline.wait_for_frames()
depth_frame = frames.get_depth_frame()

# Transform a specific point
u, v = 320, 240  # Center pixel
depth_m = depth_frame.get_distance(u, v)

point_camera = calibrator.pixel_to_camera_coordinates(u, v, depth_m)
point_world = calibrator.camera_to_world_coordinates(point_camera)

print(f"World coordinates: {point_world} cm")

# Or transform entire scene
points_world = calibrator.depth_image_to_world_points(depth_frame)

# Save for later use
calibrator.save_calibration("my_calibration.json")
```

---

## 16. Theory: Transformation Mathematics

For those interested in the math behind the transformation:

### Pinhole Camera Model
```
u = fx * (X_cam / Z_cam) + ppx
v = fy * (Y_cam / Z_cam) + ppy
```

Where:
- (u, v) = pixel coordinates
- (X, Y, Z)_cam = 3D point in camera frame
- fx, fy = focal lengths (from intrinsics)
- ppx, ppy = principal point (from intrinsics)

### Reverse (Deprojection)
```
X_cam = (u - ppx) * Z_cam / fx
Y_cam = (v - ppy) * Z_cam / fy
Z_cam = depth_value
```

### World Frame Transformation
```
P_world = R * P_camera + T
```

Where:
- P = point [X, Y, Z]
- R = 3×3 rotation matrix (from Euler angles)
- T = translation vector [X_cam, Y_cam, Z_cam]

### Rotation from Euler Angles (ZYX convention)
```
R = Rz(roll) * Ry(pan) * Rx(tilt)
```

This creates the full camera-to-world transformation.

---

## References

- RealSense SDK Documentation: https://dev.intelrealsense.com/
- Multiple View Geometry (Hartley & Zisserman) - Chapters 2, 6
- Your depth accuracy test results (01-24-2026)

---

**Good luck with your calibration! See you at the Week 4 meeting!** 🎯
