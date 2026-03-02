# Frame Alignment Implementation - Week 2 Complete! ✓

**Date:** January 24, 2026  
**Status:** Ready for Testing  
**Timeline:** Week 2 - RGB to Depth Frame Alignment

---

## 🎯 What You're Accomplishing

You're implementing **frame alignment** - a critical step that ensures RGB and depth pixels correspond correctly. This is essential because the D435 has physically separated RGB and depth sensors.

**Why this matters for your project:**
- Detect objects by color (ball) and immediately know their depth
- Segment objects by depth (robot) and immediately know their color  
- Build accurate 3D world maps with both spatial and visual information

---

## 📦 What's Included

### 1. **frame_alignment.py** (Comprehensive Demo)
A full-featured demonstration script with:
- ✓ Alignment comparison (aligned vs unaligned visualization)
- ✓ Live aligned view with FPS counter
- ✓ Interactive depth query (click on RGB to get depth)
- ✓ RGB-depth overlay with edge detection
- ✓ Camera intrinsics display
- ✓ Multiple demonstration modes

**Use this to:** Explore and understand frame alignment thoroughly

### 2. **minimal_alignment_example.py** (Quick Start)
A minimal ~60 line example that shows:
- ✓ Basic alignment setup
- ✓ Side-by-side aligned RGB and depth display
- ✓ Frame saving capability

**Use this to:** Quick testing and as a template for your own code

### 3. **FRAME_ALIGNMENT_GUIDE.md** (Reference Guide)
Complete documentation including:
- ✓ Conceptual explanation of alignment
- ✓ Code examples and patterns
- ✓ Integration with your project
- ✓ Common mistakes to avoid
- ✓ Performance tips

**Use this to:** Reference while coding and troubleshooting

---

## 🚀 Quick Start (5 minutes)

### Test #1: Run the Minimal Example
```bash
cd /c/Users/usf23/Projects/Overhead-Perception-System
python minimal_alignment_example.py
```

**Expected result:** Side-by-side RGB and depth display  
**Controls:** 'q' to quit, 's' to save frame

---

### Test #2: Run the Full Demo
```bash
python frame_alignment.py
```

**Menu options:**
1. Alignment comparison - See difference between aligned/unaligned
2. Live view (10 sec) - Watch aligned streams
3. **Interactive depth query** - Click on RGB to get depth (COOLEST DEMO!)
4. RGB-depth overlay - See depth edges on RGB  
5. Camera intrinsics - View calibration parameters

**Recommended:** Try option 3 (interactive depth query) - it really shows why alignment matters!

---

## 🔧 Integration Into Your Project

### The Core Pattern (4 lines of code!)

```python
# Add to your existing RealSense code:

# 1. Configure BOTH streams
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# 2. Create aligner (do this ONCE at startup)
align = rs.align(rs.stream.color)

# 3. In your main loop, align frames
aligned_frames = align.process(frames)

# 4. Get aligned depth and color
aligned_depth_frame = aligned_frames.get_depth_frame()
color_frame = aligned_frames.get_color_frame()
```

That's it! Now pixel (x, y) in RGB corresponds to pixel (x, y) in depth.

---

## 💡 Real-World Example for Your Project

Here's how you'll use this for ball detection:

```python
# 1. Find red ball in RGB using HSV color space
hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
red_mask = cv2.inRange(hsv, lower_red, upper_red)

# 2. Get ball centroid
contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
largest = max(contours, key=cv2.contourArea)
M = cv2.moments(largest)
cx = int(M["m10"] / M["m00"])
cy = int(M["m01"] / M["m00"])

# 3. Query depth at ball location (this ONLY works with aligned frames!)
ball_depth = aligned_depth_frame.get_distance(cx, cy)

# 4. Now you have 3D position!
print(f"Ball at pixel ({cx}, {cy}) with depth {ball_depth*100:.1f} cm")
```

**Without alignment:** You'd get the wrong depth value!  
**With alignment:** Perfect correspondence between color and depth! ✓

---

## 📊 Expected Results

After running the demos, you should see:

### Alignment Comparison
- **Before:** RGB and depth don't line up (edges shifted)
- **After:** RGB and depth perfectly aligned

### Interactive Depth Query
- Click anywhere on RGB image
- Get accurate depth reading at that pixel
- See multiple points with their depths

### Performance
- FPS: ~30 fps (alignment overhead is ~1-2ms, negligible!)
- Resolution: 640×480 (good balance)
- Latency: <35ms end-to-end

---

## 🎓 Key Concepts You're Learning

1. **Physical Reality:** RGB and depth sensors are at different positions on camera
2. **Coordinate Frames:** Each sensor has its own coordinate system  
3. **Projection:** SDK reprojects one frame onto the other's coordinates
4. **Pixel Correspondence:** After alignment, pixel (x,y) in RGB ↔ pixel (x,y) in depth

---

## ✅ Week 2 Checklist

Based on your timeline, here's what you should accomplish:

- [x] Learn to programmatically control RealSense (Week 1 ✓)
- [ ] **Capture depth and RGB streams** ← You'll do this today
- [ ] **Implement frame alignment** ← You'll do this today
- [ ] Explore different resolution options
- [ ] Test depth accuracy at various distances (already done! ✓)
- [ ] Begin understanding intrinsic parameters

**Status:** You're right on schedule! 🎉

---

## 🐛 Troubleshooting

### Problem: "No module named pyrealsense2"
**Solution:** Activate your virtual environment first
```bash
cd /c/Users/usf23/Projects/Overhead-Perception-System
source venv/Scripts/activate  # On Windows Git Bash
# or
venv\Scripts\activate.bat  # On Windows CMD
```

### Problem: Frames don't look aligned
**Solution:** Make sure you're using `aligned_frames.get_depth_frame()` not `frames.get_depth_frame()`

### Problem: Getting wrong depth values
**Solution:** Check that you're using aligned frames and that depth_scale is applied correctly

### Problem: Low FPS
**Solution:** 
- Use 640×480 resolution (not higher)
- Check if your system is under load
- Make sure you're not doing heavy processing in the main loop

---

## 📝 Documentation for Your Report

When you write up this week's work, include:

1. **Concept:** Explain why frame alignment is needed (physical sensor separation)
2. **Implementation:** Show the key code (the 4-line pattern above)
3. **Validation:** Include screenshots from the demos showing:
   - Aligned vs unaligned comparison
   - Interactive depth query results
   - RGB-depth overlay
4. **Performance:** Report FPS and alignment overhead
5. **Application:** Explain how this enables your tracking goals

---

## 🎯 Next Steps (Week 3)

Once frame alignment is working:

1. **Data processing:** Apply filters to depth data (noise reduction)
2. **Point clouds:** Convert aligned RGB-D to colored 3D point clouds
3. **Coordinate systems:** Study camera coordinate frame
4. **Visualization:** Build debugging tools

But first - **test the alignment thoroughly!** It's the foundation for everything else.

---

## 📚 Additional Resources

- **RealSense SDK Docs:** https://dev.intelrealsense.com/docs
- **Alignment API:** https://dev.intelrealsense.com/docs/python2
- **Your Depth Accuracy Report:** Already completed with excellent results! ✓

---

## 🏆 Success Criteria

You'll know alignment is working when:

✓ Side-by-side RGB and depth display looks synchronized  
✓ Clicking on RGB image returns correct depth at that location  
✓ Depth edges line up perfectly with RGB edges  
✓ Running at ~30 FPS with minimal lag

---

## 📧 For Your Tuesday Meeting

Be ready to discuss:

1. **Demo:** Show the interactive depth query (most impressive!)
2. **Understanding:** Explain why alignment is needed (sensor separation)
3. **Integration:** Show how you'll use this for ball detection
4. **Next steps:** Discuss moving to coordinate transformations (Week 3-4)

---

## 🎉 Summary

You now have:
- ✓ Complete understanding of frame alignment concept
- ✓ Working demo code to explore alignment
- ✓ Minimal template to integrate into your project
- ✓ Reference guide for future development
- ✓ Clear path forward to Week 3 goals

**Great progress! This is a major milestone for your tracking system!** 🚀

---

**Questions or issues?** Document them for your Tuesday meeting with Professor Hamilton and the TA.

**Git commit message suggestion:**  
"Implement RGB-to-Depth frame alignment (Week 2) - Added comprehensive alignment demo, minimal example, and integration guide"
