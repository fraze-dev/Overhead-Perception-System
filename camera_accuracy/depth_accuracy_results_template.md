# Depth Accuracy Test Results
## Intel RealSense D435 Camera

**Researcher:** Aaron Fraze  
**Test Date:** [Insert Date]  
**Camera Serial:** [Insert if known]  
**Firmware Version:** [Insert if known]  

---

## Test Environment

**Physical Setup:**
- Camera mounting: [e.g., tripod, overhead mount]
- Target surface: [e.g., white wall, cardboard, etc.]
- Measurement method: [e.g., tape measure, laser distance meter]

**Environmental Conditions:**
- Room temperature: [°C]
- Lighting: [e.g., bright overhead, natural window light, dimmed]
- Time of day: [e.g., 2:00 PM]

---

## Test 1: Distance vs. Ground Truth

**Objective:** Measure absolute depth error across operational range

### Results Table

| Ground Truth (cm) | Measured (cm) | Std Dev (cm) | Abs Error (cm) | Rel Error (%) | Status |
|-------------------|---------------|--------------|----------------|---------------|--------|
| 50                |               |              |                |               |        |
| 100               |               |              |                |               |        |
| 150               |               |              |                |               |        |
| 200               |               |              |                |               |        |
| 250               |               |              |                |               |        |
| 300               |               |              |                |               |        |
| 400               |               |              |                |               |        |

**Status Key:**
- ✓ Pass: Error < 2% (manufacturer spec)
- ⚠ Warning: Error 2-4%
- ✗ Fail: Error > 4%

### Observations:
[Write detailed observations here]
- Pattern in errors (systematic bias)?
- Accuracy improving or degrading with distance?
- Any anomalies?

### Error Plot:
[Attach or reference error vs. distance plot]

---

## Test 2: Spatial Uniformity

**Objective:** Verify depth consistency across field of view

**Test Distance:** [e.g., 150 cm]  
**Grid Size:** [e.g., 5×5]

### Spatial Depth Map

```
[Results will be filled in from test output]
Row 0: [depth values across]
Row 1: [depth values across]
...
```

### Analysis:
- Center region average: [cm]
- Edge regions average: [cm]
- Maximum variation: [cm]
- Center-to-edge difference: [cm or %]

### Observations:
[Write observations here]
- Is depth uniform across frame?
- Are edges less accurate than center?
- Any distortion patterns visible?

---

## Test 3: Repeatability/Precision

**Objective:** Measure depth noise and consistency

**Test Distance:** [e.g., 200 cm - manufacturer spec distance]  
**Number of Samples:** [e.g., 1000 frames]

### Results:
- Mean depth: [cm]
- Standard deviation: [cm]
- Range (max - min): [cm]
- Coefficient of variation: [std/mean × 100%]

### Depth Distribution:
[Attach or reference histogram]

### Observations:
- Is noise acceptable for project needs?
- Are there outliers or spikes?
- Does depth "drift" over time?

---

## Test 4: Environmental Effects (Optional)

### 4a. Lighting Conditions

| Lighting Condition | Mean Depth (cm) | Std Dev (cm) | Notes |
|-------------------|-----------------|--------------|-------|
| Bright            |                 |              |       |
| Normal            |                 |              |       |
| Dim               |                 |              |       |

### 4b. Surface Properties

| Surface Type      | Mean Depth (cm) | Std Dev (cm) | Notes |
|-------------------|-----------------|--------------|-------|
| White matte       |                 |              |       |
| Dark matte        |                 |              |       |
| Glossy            |                 |              |       |
| Textured          |                 |              |       |

### Observations:
[Impact of environment on depth measurement]

---

## Overall Analysis

### Summary Statistics:
- Overall mean absolute error: [cm]
- Overall mean relative error: [%]
- Best performing distance range: [cm to cm]
- Worst performing distance range: [cm to cm]

### Comparison to Specifications:
- Manufacturer spec: 2% accuracy at 2m
- Our measured accuracy at 2m: [%]
- Meets specification? [Yes/No]

### Key Findings:
1. [Finding 1]
2. [Finding 2]
3. [Finding 3]

### Systematic Errors Identified:
- [e.g., Consistent 1.5% underestimation across all distances]
- [e.g., Increased noise at distances > 3m]

---

## Recommendations

### For Project Use:
1. **Optimal operating distance:** [e.g., 1.5-2.5 meters]
2. **Expected accuracy:** [e.g., ±2-3 cm at project distance]
3. **Calibration needed?** [Yes/No - explain reasoning]

### Calibration Decision:
[Based on results, does camera need calibration?]

**Decision:** [ ] No calibration needed  |  [ ] Run On-Chip Calibration  |  [ ] Run Tare Calibration

**Reasoning:**
[Explain decision based on error magnitudes, systematic bias, etc.]

### Mitigation Strategies:
1. [e.g., Use center 80% of frame for critical measurements]
2. [e.g., Apply -1.5cm offset correction for systematic bias]
3. [e.g., Average 5-10 frames for reduced noise]

---

## Conclusions

### Strengths:
- [e.g., Excellent repeatability (±0.3cm)]
- [e.g., Meets manufacturer specs across entire range]

### Limitations:
- [e.g., Reduced accuracy at edges of frame]
- [e.g., Sensitive to very dark surfaces]

### Impact on Project:
[How will these results affect your overhead tracking system?]

---

## Appendices

### A. Raw Data Files:
- Test 1 data: `[filename.json]`
- Test 2 data: `[filename.json]`
- Test 3 data: `[filename.json]`

### B. Visualization Plots:
- Error vs. distance: `[filename.png]`
- Spatial uniformity heatmap: `[filename.png]`
- Repeatability histogram: `[filename.png]`

### C. Camera Configuration:
```python
Resolution: 640×480
Framerate: 30 fps
Depth format: Z16
Depth scale: [value] meters/unit
```

### D. Additional Notes:
[Any other relevant information, observations, or future tests to consider]

---

**Report Completed:** [Date]  
**Next Steps:** [What to do next based on these results]
