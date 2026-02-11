**Calibration Report** (due Feb 7):
- Document calibration approach
- Include transformation matrices
- Report accuracy measurements
- Provide top-down visualization

**Template**:

### World-Frame Calibration Report

## Setup
- Camera model: Intel RealSense D435
- Camera height: 220 cm
- Camera position: (0, 0) cm
- Camera orientation: tilt= Lens pointing straight down

**notes**
The camera was mounted with lens pointing vertically downward.
A bubble level was used to verify 3 planes
Vertical alignment (Tilt = 0°)
Left-right level (Roll = 0°)
Front-back level (Pan = 0°)

## Calibration Points
(-122, 66, 0) cm
(0, 67, 0) cm
(120, 66, 0) cm
(-70, 0, 0) cm
(0, 0, 0) cm
(73, 0, 0) cm
(-120, -67, 0) cm
(0, -67, 0) cm
(121, -67, 0) cm

## Results
- Mean error: 5.18 ± 1.88 cm
- RMSE: 5.51 cm
- Max error: 0.41 / 6.83 cm

## Transformation Matrix
 [
    [
      1.0,
      0.0,
      0.0,
      0.0
    ],
    [
      0.0,
      -1.0,
      0.0,
      0.0
    ],
    [
      0.0,
      0.0,
      -1.0,
      2.2
    ],
    [
      0.0,
      0.0,
      0.0,
      1.0
    ]
  ]

## Visualization
![Top down view](results/calibration/top_down_visualization.png)

## Conclusions
[Discussion of accuracy, limitations]



## 🔧 Troubleshooting

### If calibration error is high (>5 cm):
1. Re-measure camera height carefully
2. Check if camera is tilted (use level)
3. Verify calibration point measurements
4. Use center 80% of image (avoid edges)

### If Z coordinate is wrong:
- Camera height is likely incorrect
- Re-measure from lens center to ground

### If X, Y are systematically off:
- Camera position (X, Y) is wrong
- Adjust in calibration parameters

---

### 1. Define World Coordinate Frame
**Goal**: Establish your workspace coordinate system

**Action items**:
- [ ] Choose origin location (e.g., corner of arena, center)
- [ ] Define X-axis direction (e.g., toward goal)
- [ ] Define Y-axis direction (perpendicular to X)
- [ ] Mark origin with tape or marker
- [ ] Document the coordinate system with a diagram

**Time**: 30 minutes

---

### 2. Measure Camera Parameters
**Goal**: Get accurate camera position and orientation

**Action items**:
- [ ] Measure camera height from lens to ground (use laser tape measure)
  - Record: 220 cm
- [ ] Measure camera X position relative to origin
  - Record: 0 cm  
- [ ] Measure camera Y position relative to origin
  - Record: 0 cm
- [ ] Estimate camera tilt (use bubble level)
  - Record: 0 degrees
- [ ] Estimate camera pan/roll if not level
  - Record: pan 0 deg, roll 0deg

**Time**: 20 minutes

---

### 3. Run Quick Calibration Test
**Goal**: Verify the system works with basic calibration

**Action items**:
- [ ] Copy files to your project:
  - `world_frame_calibration.py` → `src/`
  - `test_calibration_simple.py` → project root
  - `CALIBRATION_GUIDE.md` → `docs/`
  
- [ ] Edit `test_calibration_simple.py` with your measurements:
  ```python
  calibrator.define_simple_overhead_calibration(
      camera_height_cm=YOUR_HEIGHT,  # From step 2
      camera_x_world=YOUR_X,          # From step 2
      camera_y_world=YOUR_Y,          # From step 2
      camera_tilt_deg=YOUR_TILT       # From step 2
  )
  ```

- [ ] Run test: `python test_calibration_simple.py`
- [ ] Verify center pixel transforms to reasonable world coords
  - Z coordinate should be ~0 cm (ground level)
  - X, Y should be within your workspace bounds

**Time**: 30 minutes

---

### 4. Calibration Points
**Goal**: Set up known reference points for validation

**Process**:
- Placed 9 **blue** tape points at **measured** world coordinates
- Measured using laser tape measure

  - Points:
    - Point 1: (-122, 66, 0) top-left corner
    - Point 2: (0, 67, 0) - top-center above origin
    - Point 3: (120, 66, 0) - top-right corner
    - Point 4: (-70, 0, 0) - center-left -70 cm on x axis
    - Point 5: (0, 0, 0) - Origin center-center
    - Point 6: (73, 0, 0) - center-right 73 cm on x axis
    - Point 7: (-120, -67, 0) - bottom-left corner
    - Point 8: (0, -67, 0) - bottom-center below origin
    - Point 9: (121, -67, 0) - bottom-right corner

- Actual results from realsense d435i
- Rounded to 1 decimal

| Label   | X (cm) | Y (cm) | Z (cm) |   Description   |
|---------|--------|--------|--------|-----------------|
| Point 1 | -122.6 | + 68.9 | - 5.9  | top-left corner |
| Point 2 | -  0.4 | + 66.6 | - 4.9  | top-center      |
| Point 3 | +123.5 | + 68.3 | - 4.9  | top-right       |
| Point 4 | - 71.7 | -  0.4 | - 4.9  | center-left     |
| Point 5 | -  0.4 | -  0.2 | - 4.8  | origin          |
| Point 6 | + 74.3 | -  0.9 | - 3.9  | center-right    |
| Point 7 | -123.8 | - 67.8 | - 3.9  | bottom-left     |
| Point 8 | -  1.8 | - 65.1 | + 5.9  | bottom-center   |
| Point 9 | +123.4 | - 67.1 | - 6.4  | bottom-right    |


---