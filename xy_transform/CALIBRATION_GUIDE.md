# Coordinate Transformation System - Usage Guide

## Overview

You now have a complete system for transforming camera pixel coordinates to world (floor) coordinates. This guide explains how to use it for your calibration validation.

## Files Created

1. **`coordinate_transform.py`** - Core transformation math
2. **`calibration_click_tool.py`** - Interactive clicking tool
3. This guide!

## Quick Start

### 1. Prepare Your Test Grid

- Place blue tape markers on floor at known positions
- Measure each marker's position from origin (0,0) 
- Example grid points: (0,0), (50,0), (100,0), (-50,0), (0,50), (0,-50), etc.
- Write down the physical measurements

### 2. Run the Clicking Tool

```bash
cd /c/Users/usf23/Projects/Overhead-Perception-System
python calibration_click_tool.py
```

### 3. Enter Camera Parameters

The tool will ask for:
- **Camera height**: 2.21 meters (your measurement)
- **Pitch angle**: 3.0 degrees (from spatial test)
- **Roll angle**: 0.0 degrees (you leveled left-right)
- **Yaw angle**: 0.0 degrees (pointing straight down)

Press ENTER to accept defaults.

### 4. Click on Test Points

- Window will open showing camera view
- Click on each blue tape marker
- Tool displays world coordinates in cm
- Markers are drawn on image with coordinates

### 5. Save Your Data

Press **'S'** to save all clicked points to JSON file.

### 6. Add Physical Measurements

After clicking, tool asks if you want to add ground truth:
- Enter the physical X,Y coordinates you measured
- Tool calculates error automatically
- Saves complete validation data

## Example Session

```
CALIBRATION CLICKING TOOL
============================================================

Enter camera mounting parameters:
(Press ENTER to use defaults from spatial test)

Camera height (meters) [2.21]: <ENTER>
Pitch angle (degrees, forward tilt) [3.0]: <ENTER>
Roll angle (degrees, left/right) [0.0]: <ENTER>
Yaw angle (degrees, rotation) [0.0]: <ENTER>

CoordinateTransformer initialized:
  Camera height: 2.210 m
  Pitch: 3.00°, Roll: 0.00°, Yaw: 0.00°

Camera intrinsics set:
  Resolution: 640 x 480
  Focal length: fx=383.15, fy=383.15
  Principal point: cx=321.25, cy=237.03

Instructions:
  - Click on blue tape markers to measure their position
  - Press 'S' to save all clicked points
  - Press 'C' to clear all markers from display
  - Press 'Q' to quit

Waiting for clicks...

[You click on marker at origin]

============================================================
Pixel: (320, 240)
Depth: 2.210 m (221.00 cm)
------------------------------------------------------------
Camera coords: (+0.0000, +0.0000, +2.2100) m
World coords:  (+0.0000, +0.0000, +0.0000) m
------------------------------------------------------------
Floor position (X, Y): (+0.00, +0.00) cm
============================================================

[Continue clicking other markers...]
[Press 'S' to save]

✓ Saved 7 points to: results/calibration/20260131_173045_calibration_points.json

Add ground truth measurements?
============================================================
Do you want to add physical measurements? (y/n): y

Point 0: Camera measured (+0.0, +0.0) cm
  Physical X (cm): 0
  Physical Y (cm): 0

Point 0 ground truth added:
  Physical:  (+0.0, +0.0) cm
  Measured:  (+0.0, +0.0) cm
  Error:     (+0.0, +0.0) cm | Total: 0.00 cm

[Continue for all points...]

============================================================
CALIBRATION ACCURACY SUMMARY
============================================================
Number of points: 7
Mean error: 2.34 cm
Std dev: 1.12 cm
Max error: 4.18 cm
Min error: 0.45 cm

Calibration session complete!
```

## Understanding the Output

### Coordinate Systems

**Camera Coordinates**:
- Origin at camera lens
- X = right (in camera view)
- Y = down (in camera view)
- Z = forward (toward floor)

**World Coordinates**:
- Origin on floor directly below camera
- X = forward (you define which direction)
- Y = left (you define which direction)
- Z = up from floor (usually 0 for floor points)

### The Transformation

```
1. Pixel (u, v) = where you clicked
2. Depth (d) = distance from camera to that point
3. Camera 3D = RealSense deprojection using intrinsics
4. World 3D = Rotation + translation accounting for tilt and height
5. Floor 2D (X, Y) = what you compare to physical measurement
```

## Tips for Best Results

### Clicking Accuracy
- Click center of blue tape markers
- Ensure good lighting (but not direct sunlight)
- Verify depth value is reasonable (~2.2m)
- Invalid depth shows warning, don't use that point

### Test Grid Design
- Start with conservative grid (±100cm × ±75cm)
- Use cardinal directions: (0,0), (±50,0), (0,±50), etc.
- Add diagonal points: (50,50), (-50,-50), etc.
- Minimum 7-10 points for good validation
- More points = better error statistics

### Physical Measurements
- Use laser tape measure from origin
- Measure twice, record once
- Mark origin clearly on floor
- Keep measurements aligned to world axes
- Record all measurements in notebook

## Expected Accuracy

Based on your depth tests at 2.21m:
- **Depth accuracy**: ±2.3 cm (from manufacturer spec)
- **Transformation error**: ±1-2 cm (geometric calculation)
- **Total expected**: ±3-4 cm in center region
- **Edge regions**: ±5-7 cm (spatial uniformity degradation)

Your conservative grid should achieve **±3-5 cm accuracy** throughout.

## Adjusting Parameters

### If Results Show Systematic Error

**All points off in one direction**:
- Check origin placement
- Verify world coordinate axes alignment
- Confirm camera height measurement

**Top/bottom rows have different errors**:
- Adjust pitch angle
- Re-measure camera tilt
- Try pitch ± 1-2 degrees

**Left/right sides have different errors**:
- Check if camera is truly level left-right
- Adjust roll angle if needed

### Refining Tilt Angles

You can update angles interactively or edit the defaults in the code:

```python
tool = CalibrationClickTool(
    camera_height_m=2.21,
    pitch_deg=3.5,  # Adjust this
    roll_deg=0.5,   # Or this
    yaw_deg=0.0     # Or this
)
```

## Next Steps

### After Initial Validation

1. **Analyze your results**:
   - Plot error vectors
   - Create error heatmap
   - Calculate statistics

2. **Expand test grid** (if accuracy is good):
   - Move from ±100cm to ±145cm
   - Test edge regions
   - Document accuracy degradation

3. **Iterate on parameters**:
   - Fine-tune tilt angles
   - Remeasure camera height if needed
   - Test different mounting positions

### For Your Report (Week 5 Deliverable)

Document:
- Camera parameters (height, tilt)
- Test grid layout
- Error statistics (mean, std, max)
- Error plots and heatmaps
- Comparison to manufacturer specs
- Conclusions about usable workspace

## Troubleshooting

### "Camera intrinsics not set" Error
- Bug in initialization, restart tool

### Depth Shows 0.000m
- No valid depth at that pixel
- Point may be outside depth range
- Try clicking nearby

### Large Systematic Errors (>10cm)
- Verify camera height measurement
- Check pitch angle estimate
- Confirm origin position on floor
- Review coordinate axes definition

### Tool Won't Start
- Check RealSense camera is connected
- Verify pyrealsense2 installed
- Ensure camera permissions (if Linux)

## Code Integration

### Using in Your Own Scripts

```python
from coordinate_transform import CoordinateTransformer

# Initialize
transformer = CoordinateTransformer(
    camera_height_m=2.21,
    pitch_deg=3.0
)

# Set intrinsics from your RealSense pipeline
transformer.set_intrinsics(intrinsics)

# Transform a point
result = transformer.pixel_to_world_coords(320, 240, depth_meters)
world_x_cm = result['world_coords_2d'][0] * 100
world_y_cm = result['world_coords_2d'][1] * 100

print(f"Position: ({world_x_cm:.1f}, {world_y_cm:.1f}) cm")
```

### For Real-Time Tracking

Once validated, integrate into your detection pipeline:
1. Detect object (robot, ball) in image → pixel location
2. Get depth at that pixel
3. Transform to world coordinates
4. Update world state

## Questions?

This system is the foundation for your entire overhead perception project. Master it now, and everything else gets easier!

Ready to test? Go place those blue tape markers! 🎯
