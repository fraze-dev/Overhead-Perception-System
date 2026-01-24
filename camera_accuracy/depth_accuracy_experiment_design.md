# Depth Accuracy Experiment Design
## Intel RealSense D435 Depth Camera

**Researcher:** Aaron Fraze  
**Date:** January 21, 2026  
**Week:** 2 - RealSense Python API Deep Dive

---

## 1. Experiment Objective

To systematically measure and document the depth accuracy of the Intel RealSense D435 camera across its operational range, identifying any systematic errors, noise characteristics, and optimal operating conditions.

---

## 2. Research Questions

1. What is the absolute depth error at various distances?
2. How does depth accuracy vary across the field of view (center vs. edges)?
3. What is the depth precision (repeatability) at fixed distances?
4. How do environmental factors (lighting, surface properties) affect accuracy?
5. Does the camera meet the manufacturer's specification of 2% accuracy at 2 meters?

---

## 3. Experimental Setup

### 3.1 Equipment Required
- Intel RealSense D435 depth camera
- Rigid camera mount (tripod or overhead fixture)
- **Measurement standard:** Tape measure or laser distance meter (±1mm accuracy preferred)
- Flat white target board or wall (non-reflective)
- Optional: Colored targets for tracking specific points
- Ruler or grid pattern for spatial reference

### 3.2 Environmental Conditions
- **Lighting:** Document ambient lighting conditions
- **Temperature:** Room temperature (~20-25°C)
- **Mounting:** Camera should be stable and level

---

## 4. Test Methodology

### Test 1: Distance vs. Ground Truth (Primary Test)
**Purpose:** Measure absolute depth error across the camera's range

**Procedure:**
1. Mount camera facing a flat, perpendicular wall
2. Place camera at known distances: 0.5m, 1.0m, 1.5m, 2.0m, 2.5m, 3.0m, 4.0m
3. At each distance:
   - Measure actual distance with tape measure/laser (ground truth)
   - Capture 100 depth frames
   - Calculate mean and standard deviation of measured depth
4. Record ambient lighting conditions

**Metrics to Calculate:**
- Absolute error: `error = measured_depth - ground_truth`
- Relative error: `relative_error = (measured_depth - ground_truth) / ground_truth × 100%`
- Standard deviation (precision/repeatability)
- Min/Max values across frames

---

### Test 2: Spatial Accuracy Map
**Purpose:** Test if depth accuracy varies across the field of view

**Procedure:**
1. Place camera at fixed distance (e.g., 1.5m) from flat wall
2. Divide camera view into a grid (e.g., 3×3 or 5×5)
3. For each grid cell:
   - Sample depth values from 100 frames
   - Calculate mean and standard deviation
4. Create a heat map showing depth variation across FOV

**Metrics to Calculate:**
- Depth uniformity across frame
- Edge vs. center accuracy
- Identify any systematic distortion patterns

---

### Test 3: Precision/Repeatability Test
**Purpose:** Measure depth noise and repeatability at fixed position

**Procedure:**
1. Position camera at exactly 2.0m from wall (manufacturer spec distance)
2. Capture 1000 consecutive frames without moving camera or target
3. Sample center region of depth frame (e.g., 100×100 pixel ROI)

**Metrics to Calculate:**
- Mean depth value
- Standard deviation (depth noise)
- Histogram of depth measurements
- Range of variation (max - min)

---

### Test 4: Environmental Effects

#### Test 4a: Lighting Conditions
**Procedure:**
1. Position camera at 1.5m from target
2. Test under different lighting:
   - Bright ambient (windows, overhead lights on)
   - Medium ambient (normal room lighting)
   - Low ambient (lights dimmed)
3. Capture 100 frames per condition

#### Test 4b: Surface Properties
**Procedure:**
1. Position camera at 1.5m from various surfaces:
   - White matte surface
   - Dark matte surface
   - Glossy/reflective surface
   - Textured surface
2. Capture 100 frames per surface type

**Metrics:**
- Compare depth accuracy and noise across conditions
- Identify problematic scenarios

---

## 5. Data Collection Format

For each test, record:
```
Test ID: [e.g., Test1_Distance_050cm]
Date/Time: [timestamp]
Ground Truth Distance: [measured in cm]
Sample Size: [number of frames]
Lighting Condition: [description]
Target Surface: [description]

Results:
- Mean Measured Depth: [cm]
- Standard Deviation: [cm]
- Absolute Error: [cm]
- Relative Error: [%]
- Min/Max Measured: [cm]
```

---

## 6. Analysis Plan

### 6.1 Visualization
- Plot measured depth vs. ground truth (with error bars)
- Create error vs. distance graph
- Generate spatial accuracy heat maps
- Show histograms of depth distributions

### 6.2 Statistical Analysis
- Calculate overall accuracy metrics
- Identify systematic biases
- Determine confidence intervals
- Compare to manufacturer specifications

### 6.3 Reporting
- Summarize accuracy across operational range
- Document optimal operating conditions
- Identify limitations and edge cases
- Provide recommendations for project use

---

## 7. Success Criteria

The experiment is successful if:
1. ✓ Depth accuracy is measured at 7+ distances
2. ✓ Statistical metrics (mean, std dev) are calculated for each distance
3. ✓ Results are compared to manufacturer specifications
4. ✓ Spatial uniformity is characterized
5. ✓ Environmental effects are documented
6. ✓ Clear recommendations are provided for project use

---

## 8. Expected Outcomes

- Comprehensive characterization of camera accuracy (±X cm at Y meters)
- Understanding of depth noise characteristics
- Knowledge of optimal operating distance for overhead setup
- Documentation of any systematic errors requiring compensation
- Informed decision about whether calibration is needed

---

## 9. Timeline

- **Day 1:** Setup and initial testing (Test 1)
- **Day 2:** Spatial and precision testing (Tests 2-3)
- **Day 3:** Environmental testing and analysis (Test 4)
- **Day 4:** Data analysis and documentation

---

## 10. Safety and Practical Notes

- Ensure camera is securely mounted to prevent drops
- Use consistent lighting for comparable results
- Keep camera at room temperature (avoid thermal drift)
- Document any anomalies or unexpected observations
- Take photos of physical setup for documentation

---

## 11. Calibration Decision Criteria

**After completing the experiment, calibrate if:**
- Absolute error consistently exceeds 3-4% at 2m distance
- Systematic bias is observed (all measurements consistently high or low)
- Depth noise (std dev) is unusually high (>1cm at 2m)
- Camera has been dropped or received physical impact

**Calibration procedure (if needed):**
1. Open RealSense Viewer
2. Run "On-Chip Calibration" (takes ~5 minutes)
3. Check "Health Score" (>0.15 is good, >0.25 is excellent)
4. If improvement is needed, run "Tare Calibration" with precise ground truth
5. Re-run accuracy tests to verify improvement

---

## References
- Intel RealSense D435 Datasheet
- Intel RealSense Self-Calibration White Paper
- RealSense SDK Documentation
