# Quick Start Guide: Depth Accuracy Testing

**Week 2 Task:** Exploring depth accuracy testing  
**Date:** January 21, 2026

---

## 🎯 Goal

Test your RealSense D435 camera's depth accuracy and document results for your Week 2 meeting.

---

## 📋 What You Have

1. **Experiment Design Document** (`depth_accuracy_experiment_design.md`)
   - Detailed test methodology
   - Research questions
   - Success criteria

2. **Python Test Script** (`depth_accuracy_test.py`)
   - Automated testing
   - Interactive menu
   - Results saved as JSON

3. **Results Template** (`depth_accuracy_results_template.md`)
   - Fill-in-the-blank format
   - Analysis framework
   - Reporting structure

4. **Calibration Guide** (`realsense_calibration_guide.md`)
   - When to calibrate (probably not needed!)
   - How to calibrate if needed
   - Troubleshooting tips

---

## ⚡ Quick Start (30 minutes)

### Setup (5 minutes):
1. Position your RealSense camera on a stable mount (tripod or desk)
2. Face it toward a flat wall
3. Get a tape measure or ruler

### Run Basic Test (15 minutes):
```bash
cd /path/to/your/project
python depth_accuracy_test.py
```

**In the menu:**
- Choose option 2 (Multiple Distance Tests)
- Enter: `50, 100, 150, 200`
- For each distance:
  - Position camera at that distance
  - Measure precisely with tape measure
  - Press ENTER
  - Wait for test to complete (~30 seconds)

### Review Results (10 minutes):
- Check console output for accuracy summary
- Look in `results/depth_accuracy/` for JSON files
- Note the relative error percentages

---

## 🔬 Full Test Suite (2-3 hours)

For comprehensive Week 2 deliverable:

### Day 1: Distance Accuracy
```bash
python depth_accuracy_test.py
```
- Option 2: Multiple distances
- Test at: 50, 100, 150, 200, 250, 300, 400 cm
- Takes ~20 minutes total

### Day 2: Spatial & Precision
```bash
python depth_accuracy_test.py
```
- Option 3: Spatial uniformity (10 minutes)
- Option 4: Repeatability test (15 minutes)

### Day 3: Analysis & Documentation
- Fill in `depth_accuracy_results_template.md`
- Create plots (optional - script saves histograms)
- Write observations and conclusions

---

## 📊 What to Look For

### Good Performance:
✓ Relative error < 2% at 2 meters  
✓ Standard deviation < 1 cm  
✓ Consistent across distances  
✓ Center and edges similar  

### Potential Issues:
⚠ Error > 3-4% consistently  
⚠ Large standard deviation (> 2 cm)  
⚠ Systematic bias (always too high/low)  
⚠ Big difference center vs. edges  

---

## 🔧 Do I Need Calibration?

**Short answer:** Probably not! Test first.

**Calibrate if:**
- Errors consistently > 3-4%
- Systematic bias detected
- Camera was dropped

**How to calibrate:**
1. Open RealSense Viewer
2. Go to "On-Chip Calibration"
3. Follow prompts (takes 5 minutes)
4. Re-run tests to verify improvement

See `realsense_calibration_guide.md` for full details.

---

## 📁 File Structure

After testing, you'll have:

```
Overhead-Perception-System/
├── src/
│   └── depth_accuracy_test.py        # Your test script
├── results/
│   └── depth_accuracy/
│       ├── 20260121_120530_dist_50cm.json
│       ├── 20260121_120630_dist_100cm.json
│       ├── ... (more test results)
│       └── dist_50cm_histogram.png
├── docs/
│   ├── depth_accuracy_experiment_design.md
│   ├── realsense_calibration_guide.md
│   └── depth_accuracy_results_FILLED.md    # Your completed report
```

---

## 💡 Tips for Success

### Accuracy Tips:
- Use a laser distance meter if available (more precise)
- Ensure wall is perpendicular to camera
- Measure from camera lens center to wall
- Keep lighting consistent between tests
- Let camera warm up 2-3 minutes before testing

### Time Savers:
- Start with just 3-4 distances (50, 100, 150, 200 cm)
- Use 100 frames per test (default) - good balance
- Run Option 2 for batch testing (faster than one-by-one)

### Common Mistakes:
- ❌ Measuring from camera body instead of lens
- ❌ Wall not perpendicular (angle introduces error)
- ❌ Moving camera during test
- ❌ Ignoring lighting changes between tests

---

## 📝 For Your Week 2 Meeting

Prepare to discuss:

1. **Test Results:**
   - "I tested distances from [X] to [Y] cm"
   - "Accuracy was [±Z cm] or [±W%]"
   - "Meets/exceeds manufacturer spec of 2% at 2m"

2. **Observations:**
   - Any patterns in errors?
   - Performance at different distances?
   - Noise characteristics?

3. **Calibration Decision:**
   - "Calibration was/wasn't needed because..."
   - If calibrated: before/after comparison

4. **Project Implications:**
   - "For overhead tracking at [H] meters height..."
   - "Expected position accuracy will be..."
   - "Sufficient for tracking robot/ball/obstacles"

---

## 🚀 Next Steps After Testing

Week 2 → Week 3 transition:

1. ✅ Complete depth accuracy tests
2. ✅ Document results in template
3. ✅ Make calibration decision
4. ➡️ Start Week 3: Data processing and coordinate systems
5. ➡️ Use accuracy measurements to set expectations for world-frame mapping

---

## ❓ Quick Troubleshooting

**Problem:** Script won't run  
**Fix:** Check RealSense is connected, SDK installed

**Problem:** No depth data / all zeros  
**Fix:** Check distance (too close? too far?), adjust lighting

**Problem:** Noisy measurements  
**Fix:** Increase num_frames, improve lighting, check surface

**Problem:** Inconsistent results  
**Fix:** Ensure camera is stable, wall is flat, lighting constant

---

## 📞 Need Help?

Resources:
- `depth_accuracy_experiment_design.md` - Full methodology
- `realsense_calibration_guide.md` - Calibration details
- RealSense Viewer - Visual debugging tool
- Intel support: https://support.intelrealsense.com

---

## ✅ Success Checklist

Before Week 2 meeting:

- [ ] Ran distance accuracy tests at 5+ distances
- [ ] Documented results in template
- [ ] Calculated mean errors and standard deviations
- [ ] Compared to manufacturer specifications (2% at 2m)
- [ ] Made calibration decision (with reasoning)
- [ ] Identified optimal operating distance for project
- [ ] Can discuss depth noise characteristics
- [ ] Ready to show test results and plots

---

**Remember:** This is exploratory research! Document what you find, even if results are unexpected. That's valuable data for your project.

**Good luck with your testing!** 🎉
