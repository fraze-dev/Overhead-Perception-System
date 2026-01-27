# Intel RealSense D435i Camera Specifications

## Hardware Specifications

### Depth Technology
- **Technology:** Stereo depth sensing with active IR projector
- **Depth Range:** 0.3m to ~10m (optimal range: 0.5m - 5m)
- **Depth Accuracy:** <2% at 2m distance
- **Baseline:** 50mm (distance between stereo cameras)

### Field of View
- **Depth FOV:** 87° × 58° (H × V)
- **RGB FOV:** 69° × 42° (H × V)

### Physical Specifications
- **Dimensions:** 90mm × 25mm × 25mm
- **Weight:** ~72g
- **Connection:** USB 3.1 Gen 1
- **Power Consumption:** Typical 1.5W - 3W

### IMU (Inertial Measurement Unit)
- **Type:** 6-axis IMU (BMI055)
- **Components:** 3-axis accelerometer + 3-axis gyroscope
- **Use Case:** Camera motion tracking and orientation

---

## Available Stream Configurations

### Depth Stream Options
Check which resolutions and frame rates are available in RealSense Viewer:

- [X ] 1280 × 720 @ 30 fps
- [X ] 848 × 480 @ 30 fps, 60 fps, 90 fps
- [X ] 640 × 480 @ 30 fps, 60 fps, 90 fps
- [X ] 640 × 360 @ 30 fps, 60 fps, 90 fps
- [X ] 424 × 240 @ 30 fps, 60 fps, 90 fps

**Currently Using:** ___640 x 480 @ 30fps____

**Notes:**
- Higher resolution = more detail but more processing required
- Higher frame rate = smoother motion but more bandwidth/processing


### RGB/Color Stream Options
Available color camera configurations:

- [ X] 1920 × 1080 @ 30 fps
- [ X] 1280 × 720 @ 30 fps
- [ X] 960 × 540 @ 30 fps
- [ X] 848 × 480 @ 30 fps, 60 fps
- [ X] 640 × 480 @ 30 fps, 60 fps
- [ X] 640 × 360 @ 30 fps, 60 fps
- [ X] 424 × 240 @ 30 fps, 60 fps

**Currently Using:** ___640 x 480 @ 30fps___

**Notes:**


### Infrared Streams
The D435i has two infrared cameras (left and right) for stereo depth:

- [ ] 1280 × 720 @ various fps
- [ ] 848 × 480 @ various fps
- [ ] 640 × 480 @ various fps

**Notes:**
- Usually not needed for standard applications
- Useful for debugging depth issues


### IMU/Motion Data Rates
Available sampling rates:

**Accelerometer:**
- [ ] 63 Hz
- [ ] 250 Hz

**Gyroscope:**
- [ ] 200 Hz
- [ ] 400 Hz

**Notes:**
- IMU not required for stationary overhead tracking
- Only needed for moving camera applications
- Disabling saves bandwidth and processing resources

---

## Visual Presets

Presets optimize camera settings for different scenarios:

- [ ] **Default** - Balanced settings for general use
- [ ] **Hand** - Optimized for hand tracking
- [ X] **High Accuracy** - Better precision, shorter range
- [ ] **High Density** - More depth pixels filled
- [ ] **Medium Density** - Balance between accuracy and density
- [ ] **Custom** - User-defined settings

**Best Preset for Overhead Tracking:** ___High Accuracy____

**Reasoning:**
- Indoor controlled environment (you tested in normal/dim lighting)
- Fixed working distance (1.5-2.5m optimal range from your tests)
- High Accuracy preset optimizes for <2m range with minimal noise
- Your repeatability test showed 0.36cm std dev - High Accuracy will maintain this

---

## Post-Processing Filters

Filters improve depth data quality:

### Available Filters:
- [ ] **Decimation Filter**
  - Purpose: Reduces resolution for better performance
  - When to use: When you need faster processing and don't need full resolution

- [ X] **Spatial Filter**
  - Purpose: Edge-preserving spatial smoothing
  - When to use: To reduce depth noise while preserving edges

- [ X] **Temporal Filter**
  - Purpose: Reduces temporal noise across frames
  - When to use: For stationary or slow-moving scenes

- [ ] **Hole Filling Filter**
  - Purpose: Fills holes/gaps in depth data
  - When to use: When you need complete depth coverage
  - Modes: farest_from_around, nearest_from_around, farest_from_left, nearest_from_left

- [ X] **Threshold Filter**
  - Purpose: Limits depth range (min/max distance)
  - When to use: To focus on specific depth ranges

- [ ] **Disparity Transform**
  - Purpose: Converts depth to disparity (linear in depth)
  - When to use: Before spatial/temporal filtering for better results

### Recommended Filter Pipeline:
1. _Threshold Filter__
2. _Spatial Filter_
3. _Temporal Filter_

**Notes on filter performance:**
1. **Threshold Filter** - Set min=0.3m, max=3.5m (focus on your workspace)
2. **Spatial Filter** - Magnitude 2, Alpha 0.5 (smooth the edge noise you saw)
3. **Temporal Filter** - Alpha 0.4, Delta 20 (reduce 0.36cm noise without motion blur)

**Notes on filter performance:**
- Spatial filter addresses left-edge artifacts (15-30cm std dev issue)
- Temporal filter will reduce your 0.36cm repeatability noise to ~0.1-0.15cm
- Avoid hole filling - your depth coverage was already good (center 3×3 stable)

---

## Recommended Configuration for Overhead Tracking Project

### Primary Configuration:
- **Depth Resolution:** ___848x480___
- **Depth Frame Rate:** ___30 fps______
- **RGB Resolution:** ____848 x 480____
- **RGB Frame Rate:** ____30 fps_____
- **Visual Preset:** __High Accuracy____
- **Key Filters Enabled:** _Threshold (0.3-3.5m), Spatial (mag=2, alpha=0.5), Temporal (alpha=0.4, delta=20)__

### Reasoning:
- 848×480 provides ~16ft × 9ft coverage at 2.5m height (ideal for robot arena)
- 30fps sufficient for robot tracking (no high-speed motion expected)
- High Accuracy preset optimized for 0.5-2.5m range where tests showed <1.1% error
- Spatial filter addresses left-edge noise artifacts observed in uniformity testing
- Temporal filter reduces 0.36cm repeatability noise without introducing motion blur
- Threshold filter focuses on operational range, excludes floor/ceiling noise

### Alternative Configurations:

**Configuration 2 (Higher Coverage):**
- **Use Case:** ___Larger arena or higher mounting (3m+)___
- **Settings:** __1280×720 depth @ 30fps, Medium Density preset, increase spatial filter strength__
- **Trade-off:** More coverage but higher processing load, potentially reduced accuracy beyond 2.5m

---

## Performance Notes

### Observations:
- **Effective depth range in your environment:** 50-250cm optimal (<1.1% error), 300cm marginal (3.55% error)
- **Depth accuracy observations:** 
  - Exceptional at 1.5-2.5m: ±1-2.5cm absolute error
  - Consistent +0.5 to +2cm positive bias (reads slightly far)
  - Center 3×3 region very stable (std dev <3cm)
  - Left edge shows high noise (15-30cm std dev) - avoid for critical measurements
- **Frame rate stability:** 30fps maintained consistently across 1000+ frame tests
- **Lighting conditions impact:** Minimal - tested bright/normal/dim with <0.06cm variation in mean depth

### Known Limitations:
- Direct sunlight can interfere with depth sensing (IR-based stereo)
- Transparent/reflective surfaces may not produce good depth data
- Very dark surfaces may have reduced depth accuracy
- Maximum range decreases in outdoor lighting
- **Left edge of FOV shows artifacts** - use center 60-80% for critical measurements
- Accuracy degrades beyond 2.5m (3.55% error at 3m vs 1.1% at 2m)
---

## Python Configuration Reference

Based on your observations, here's how to configure in Python:

```python
import pyrealsense2 as rs
import numpy as np

# Initialize pipeline
pipeline = rs.pipeline()
config = rs.config()

# Configure streams - optimized for overhead tracking
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)

# Start pipeline
profile = pipeline.start(config)

# Get depth sensor and set visual preset
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# Set to High Accuracy preset (preset 3)
depth_sensor.set_option(rs.option.visual_preset, 3)

# Configure post-processing filters
# 1. Threshold filter - focus on workspace range
threshold_filter = rs.threshold_filter()
threshold_filter.set_option(rs.option.min_distance, 0.3)  # 30cm min
threshold_filter.set_option(rs.option.max_distance, 3.5)  # 350cm max

# 2. Spatial filter - edge-preserving smoothing
spatial_filter = rs.spatial_filter()
spatial_filter.set_option(rs.option.filter_magnitude, 2)
spatial_filter.set_option(rs.option.filter_smooth_alpha, 0.5)
spatial_filter.set_option(rs.option.filter_smooth_delta, 20)

# 3. Temporal filter - reduce noise across frames
temporal_filter = rs.temporal_filter()
temporal_filter.set_option(rs.option.filter_smooth_alpha, 0.4)
temporal_filter.set_option(rs.option.filter_smooth_delta, 20)

# Apply filters in pipeline
def process_frames(frames):
    """Apply filter pipeline to depth frames"""
    depth_frame = frames.get_depth_frame()
    
    # Apply filters in order
    depth_frame = threshold_filter.process(depth_frame)
    depth_frame = spatial_filter.process(depth_frame)
    depth_frame = temporal_filter.process(depth_frame)
    
    return depth_frame

# Usage in main loop
while True:
    frames = pipeline.wait_for_frames()
    
    # Get color frame
    color_frame = frames.get_color_frame()
    
    # Process depth with filters
    filtered_depth = process_frames(frames)
    
    if not filtered_depth or not color_frame:
        continue
    
    # Convert to numpy arrays
    depth_image = np.asanyarray(filtered_depth.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    
    # Convert depth to meters
    depth_in_meters = depth_image * depth_scale
    
    # tracking code here...
```

---

## Additional Resources

- **RealSense SDK Documentation:** https://dev.intelrealsense.com/docs
- **Python API Reference:** https://intelrealsense.github.io/librealsense/python_docs/
- **GitHub Examples:** https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python/examples
- **D435i Datasheet:** https://www.intelrealsense.com/depth-camera-d435i/

---

## Change Log

**Date:** _______________  
**Updated by:** _______________  
**Changes:**
- Initial documentation created
- 

