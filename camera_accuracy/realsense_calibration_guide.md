# Intel RealSense D435 Calibration Guide

**Author:** Aaron Fraze  
**Date:** January 21, 2026  
**Purpose:** Understanding when and how to calibrate the RealSense D435 camera

---

## Do I Need to Calibrate?

### Short Answer:
**Probably not!** RealSense cameras come factory-calibrated and typically maintain accuracy throughout their lifetime.

### When Calibration IS Needed:
- ✗ Camera dropped or received physical shock
- ✗ Severe vibration or mechanical stress
- ✗ Depth measurements consistently off by >3-4%
- ✗ Systematic bias detected (all measurements high or low)
- ✗ "Health Check" score < 0.15 (see On-Chip Calibration section)
- ✗ Significant temperature/humidity cycling

### When Calibration IS NOT Needed:
- ✓ Camera just unboxed (factory calibration is excellent)
- ✓ Normal handling and transport
- ✓ Measurements within 2-3% of ground truth
- ✓ Camera has not been subjected to shock

**Decision Rule:** Run your depth accuracy tests FIRST. Only calibrate if results show systematic problems.

---

## Understanding RealSense Calibration

### What is "Calibration"?

RealSense uses stereo vision with two infrared cameras. Calibration ensures:
1. **Intrinsic parameters** - Each camera's internal properties (focal length, distortion)
2. **Extrinsic parameters** - Relative position between the two cameras
3. **Depth accuracy** - Correct distance measurements

### Factory Calibration:
- Done at Intel factory with specialized equipment
- Achieves specification: ~2% accuracy at 2 meters
- Stored in camera's EEPROM (non-volatile memory)
- Can be restored if calibration goes wrong

---

## Types of Calibration

### 1. On-Chip Calibration ⭐ (Start Here)
**Purpose:** Optimize stereo matching and reduce depth noise  
**Time:** 5-10 minutes  
**Complexity:** Easy - automated in RealSense Viewer  
**When to use:** First line of defense for any depth issues

**What it does:**
- Analyzes current camera performance
- Optimizes internal stereo matching parameters
- Does NOT require any external targets or measurements
- Safe - can always revert to factory calibration

**How to do it:**
1. Open RealSense Viewer
2. Connect camera
3. Go to "More Options" (3 dots)
4. Select "On-Chip Calibration"
5. Follow on-screen instructions (aim at textured surface ~100cm away)
6. Wait for calibration to complete
7. Check "Health Score":
   - < 0.15: Poor - likely physical damage
   - 0.15-0.25: Acceptable - marginal improvement
   - > 0.25: Good - calibration successful
8. Toggle between old/new calibration to compare
9. Apply if improvement seen

**Recommendation:** Run this FIRST if you suspect any issues.

---

### 2. Tare Calibration (Ground Truth Calibration)
**Purpose:** Improve absolute depth accuracy using known distance  
**Time:** 2-3 minutes  
**Complexity:** Easy - requires accurate ground truth measurement  
**When to use:** Systematic depth offset detected (e.g., always 2cm too far)

**What it does:**
- Uses a known precise distance as reference
- Adjusts depth scale to match ground truth
- Corrects systematic offset errors

**Requirements:**
- Accurate measurement of distance to flat target
- Tape measure or laser distance meter (±1mm accuracy)
- Flat, non-reflective target (wall, board)

**How to do it:**
1. Open RealSense Viewer
2. Position camera at precisely measured distance (recommend ~1-2 meters)
3. Measure distance camera-to-wall with high accuracy
4. Go to "Tare Calibration" in viewer
5. Enter your ground truth distance (in mm!)
6. Click "Calibrate"
7. Wait for calibration (5-10 seconds)
8. Verify improvement by checking depth reading

**Pro Tip:** 
- Use center of frame for ground truth measurement
- Ensure wall is perpendicular to camera
- Multiple measurements increase confidence

---

### 3. Dynamic Calibration (Advanced)
**Purpose:** Calibrate BOTH depth and RGB camera alignment  
**Time:** 10-15 minutes  
**Complexity:** Moderate - requires calibration target  
**When to use:** RGB-depth misalignment issues

**What it does:**
- Calibrates relative pose between depth and RGB cameras
- Improves color-depth alignment for overlays
- More comprehensive than On-Chip alone

**Requirements:**
- Dynamic Calibration Tool (separate download from Intel)
- Printed calibration target (provided with tool)
- Or: White board/target that tool can analyze

**Note:** Not typically needed for depth-only applications. Use if you need precise RGB-depth registration.

---

### 4. OEM Calibration (Factory-Level)
**Purpose:** Complete re-calibration like factory  
**Time:** 30+ minutes  
**Complexity:** Advanced - requires special V-shaped target  
**When to use:** Only for severe issues or custom enclosures

**What it does:**
- Full intrinsic and extrinsic recalibration
- Requires large precision calibration target
- Typically only needed if camera internals were modified

**Note:** Not recommended for normal users. Contact Intel support if needed.

---

## Step-by-Step: Recommended Calibration Workflow

### Before Calibrating:
1. ✓ Run your depth accuracy tests
2. ✓ Document current performance
3. ✓ Determine if calibration is actually needed

### Calibration Process:

#### Step 1: On-Chip Calibration (Always try first)
```
1. Open RealSense Viewer
2. Connect D435 camera
3. Start depth stream
4. Click "..." (More Options) → "On-Chip Calibration"
5. Aim at textured surface ~1m away
6. Click "Start"
7. Keep camera still for ~30 seconds
8. Review Health Score
9. Toggle old/new and compare depth
10. "Apply" if better, "Discard" if worse
```

**Expected outcome:** Health score > 0.15, reduced noise

#### Step 2: Verify Improvement
- Rerun your distance accuracy tests
- Compare before/after statistics
- Document improvement (or lack thereof)

#### Step 3: Tare Calibration (If systematic offset remains)
```
1. Measure precise distance to flat wall (e.g., 1500mm)
2. Position camera at exact distance
3. In RealSense Viewer → "Tare Calibration"
4. Enter ground truth: 1500 (in mm!)
5. Click "Calibrate"
6. Verify depth now matches ground truth
```

**Expected outcome:** Systematic offset eliminated

#### Step 4: Re-test and Document
- Run full test suite again
- Compare results to pre-calibration
- Document improvement percentage
- Save new calibration if better

---

## Calibration Best Practices

### DO:
- ✓ Test accuracy BEFORE calibrating
- ✓ Document "before" performance for comparison
- ✓ Use precise ground truth measurements
- ✓ Allow camera to warm up (2-3 minutes) before calibrating
- ✓ Keep camera stable during calibration
- ✓ Save original factory calibration (can always restore)
- ✓ Verify improvement after calibration

### DON'T:
- ✗ Calibrate "just because" without measuring performance first
- ✗ Use estimated/approximate ground truth for Tare
- ✗ Move camera during calibration process
- ✗ Calibrate immediately after powering on (let stabilize)
- ✗ Panic if calibration doesn't help (you can restore factory settings)

---

## Troubleshooting

### Problem: Health Score < 0.15 after On-Chip Calibration
**Likely cause:** Physical damage to camera optics or alignment  
**Solution:** 
1. Restore factory calibration
2. If score still low, camera may be damaged
3. Contact Intel support if under warranty

### Problem: Tare Calibration makes it worse
**Likely cause:** Inaccurate ground truth measurement  
**Solution:**
1. Verify your distance measurement is precise
2. Ensure wall is perpendicular to camera
3. Try measuring at different distance
4. Restore previous calibration if needed

### Problem: Calibration doesn't improve accuracy
**Possible causes:**
- Camera is already well-calibrated
- Issues are environmental (lighting, surface properties)
- Hardware failure (rare)

**Solution:**
1. Check if errors are within manufacturer spec (2% at 2m)
2. Test different surfaces/lighting
3. Restore factory calibration
4. Accept current accuracy and work with it

---

## Restoring Factory Calibration

If you want to undo calibration changes:

1. Open RealSense Viewer
2. Connect camera
3. Go to "..." (More Options)
4. Select "Restore Factory Calibration"
5. Confirm restoration
6. Camera will reboot with factory settings

**This is SAFE and reversible!** Your factory calibration is always preserved.

---

## Quick Reference: Calibration Commands

### Using RealSense Viewer (GUI - Recommended):
```
Open RealSense Viewer → More Options → Calibration
```

### Using Python API (Advanced):
```python
import pyrealsense2 as rs

# Get device
pipeline = rs.pipeline()
profile = pipeline.start()
device = profile.get_device()

# Trigger on-chip calibration
on_chip_calib = rs.on_chip_calib()
# ... (see Intel documentation for full API)
```

---

## For Your Project

**Recommendation:**
1. Run depth accuracy tests first (use provided test script)
2. Document baseline performance
3. If accuracy is within 2-3% at 2m → **No calibration needed**
4. If errors > 3-4% or systematic bias → Try On-Chip Calibration
5. If still problematic → Try Tare Calibration with precise ground truth
6. Document everything for your Week 2 report!

**Expected outcome for your overhead system:**
- Typical accuracy: ±2cm at 2m distance
- After calibration (if needed): ±1-1.5cm at 2m
- Good enough for tracking robot and ball positions

---

## Resources

- **RealSense Viewer:** Included with SDK installation
- **Intel White Papers:** 
  - Self-Calibration for D400 Series
  - Best Known Methods for Camera Performance
- **Support:** https://support.intelrealsense.com
- **Documentation:** https://dev.intelrealsense.com

---

## Summary

1. **Test first, calibrate only if needed**
2. **Most cameras don't need calibration** (factory calibration is excellent)
3. **On-Chip Calibration** is safe and easy to try
4. **Tare Calibration** fixes systematic offsets
5. **Always document before/after** for your report
6. **Factory calibration can always be restored**

**Bottom line:** Don't worry about calibration until your tests show it's actually necessary!
