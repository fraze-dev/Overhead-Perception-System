# Frame Alignment Quick Start Guide

## What is Frame Alignment?

The Intel RealSense D435 has **two separate sensors**:
- **RGB sensor** (left side of camera)
- **Depth sensor** (right side of camera)

These sensors have:
- Different physical positions
- Different fields of view
- Different resolutions (potentially)

**Without alignment**, RGB pixel (x, y) does NOT correspond to depth pixel (x, y).

**With alignment**, the SDK projects one stream onto the other's coordinate frame, so pixel coordinates match up perfectly.

## Why Do You Need It?

For your overhead tracking project, you'll need aligned frames to:

1. **Color-based detection with depth filtering**
   - Detect a red ball using RGB → Get its depth value at the same pixel
   
2. **Depth-based segmentation with color info**
   - Find objects using depth → Know what color they are

3. **Accurate 3D localization**
   - Need pixel (x, y) to map to both RGB and depth simultaneously

## Basic Usage

```python
import pyrealsense2 as rs
import numpy as np

# Configure pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start
pipeline.start(config)

# CREATE THE ALIGN OBJECT - This is the key!
align_to = rs.stream.color  # Align depth TO color frame
align = rs.align(align_to)

# In your main loop:
while True:
    # Get frames
    frames = pipeline.wait_for_frames()
    
    # ALIGN THE FRAMES
    aligned_frames = align.process(frames)
    
    # Now get aligned depth and color
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    
    # Convert to numpy
    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    
    # Now depth_image[y, x] corresponds to color_image[y, x]!
    # You can query depth at any RGB pixel:
    depth_at_pixel = aligned_depth_frame.get_distance(x, y)  # in meters
```

## Alignment Options

You can align in two directions:

### 1. Align Depth → Color (RECOMMENDED for most cases)
```python
align = rs.align(rs.stream.color)
```
**Pros:**
- Depth data projected onto RGB frame
- Query depth at any RGB pixel coordinate
- Better for color-based detection with depth filtering

**When to use:** When you're primarily doing color-based detection (finding red ball, etc.)

### 2. Align Color → Depth
```python
align = rs.align(rs.stream.depth)
```
**Pros:**
- RGB data projected onto depth frame
- Depth frame stays "pure" (no interpolation)
- Better for depth-first pipelines

**When to use:** When you're primarily doing depth-based segmentation

## Integration with Your Project

### Example: Find Red Ball with Depth

```python
import pyrealsense2 as rs
import numpy as np
import cv2

# Setup
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# Create aligner
align = rs.align(rs.stream.color)

while True:
    # Get and align frames
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    
    if not aligned_depth_frame or not color_frame:
        continue
    
    # Convert to numpy
    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    
    # Detect red ball using HSV
    hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
    
    # Red color range
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        # Get largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Get centroid
        M = cv2.moments(largest_contour)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            # NOW WE CAN GET DEPTH AT THIS RGB PIXEL!
            depth_at_ball = aligned_depth_frame.get_distance(cx, cy)
            
            print(f"Ball at pixel ({cx}, {cy}) -> Depth: {depth_at_ball*100:.1f} cm")
            
            # Draw on image
            cv2.circle(color_image, (cx, cy), 5, (0, 255, 0), -1)
            cv2.putText(color_image, f"{depth_at_ball*100:.1f}cm", 
                       (cx+10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    cv2.imshow('Red Ball Detection with Depth', color_image)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

pipeline.stop()
cv2.destroyAllWindows()
```

## Common Patterns for Your Project

### Pattern 1: Color Detection → Depth Query
Use when you detect objects by color and need their 3D position.

```python
# 1. Find object in RGB
# 2. Get centroid pixel (cx, cy)
# 3. Query aligned depth
depth = aligned_depth_frame.get_distance(cx, cy)
```

### Pattern 2: Depth Segmentation → Color Query
Use when you detect objects by depth and need their color.

```python
# 1. Segment objects using depth
# 2. For each object, get pixels
# 3. Query aligned RGB at those pixels
color_of_object = color_image[y, x]
```

### Pattern 3: Combined RGB-D Filter
Use when you need both criteria (e.g., "red objects within 2 meters").

```python
# 1. Create color mask
red_mask = cv2.inRange(hsv, lower_red, upper_red)

# 2. Create depth mask (objects between 0.5-2.5 meters)
depth_meters = depth_image * depth_scale
depth_mask = ((depth_meters > 0.5) & (depth_meters < 2.5)).astype(np.uint8) * 255

# 3. Combine masks
combined_mask = cv2.bitwise_and(red_mask, depth_mask)

# 4. Find objects in combined mask
contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
```

## Performance Tips

1. **Stream Resolution**: Use 640x480 for good balance of accuracy and speed
2. **Frame Rate**: 30 FPS is standard, 60 FPS if you need fast motion tracking
3. **Alignment is Fast**: ~1-2ms overhead, negligible for real-time
4. **Pre-filter Before Alignment**: If possible, but usually not necessary

## Common Mistakes to Avoid

❌ **Mistake 1: Using unaligned frames**
```python
frames = pipeline.wait_for_frames()
depth = frames.get_depth_frame()  # WRONG! Not aligned
color = frames.get_color_frame()
# depth[x,y] does NOT match color[x,y]
```

✅ **Correct:**
```python
frames = pipeline.wait_for_frames()
aligned_frames = align.process(frames)  # ALIGN FIRST
depth = aligned_frames.get_depth_frame()
color = aligned_frames.get_color_frame()
```

❌ **Mistake 2: Mixing aligned and unaligned**
```python
aligned_frames = align.process(frames)
depth = aligned_frames.get_depth_frame()  # Aligned
color = frames.get_color_frame()  # Unaligned - WRONG!
```

✅ **Correct:**
```python
aligned_frames = align.process(frames)
depth = aligned_frames.get_depth_frame()  # Aligned
color = aligned_frames.get_color_frame()  # Also aligned
```

❌ **Mistake 3: Assuming alignment fixes all issues**
- Alignment fixes coordinate correspondence
- Does NOT fix depth accuracy issues
- Does NOT fix temporal sync issues (those are already handled)

## Next Steps

1. **Test the demo script**: Run `frame_alignment.py` to see alignment in action
2. **Integrate into your project**: Add alignment to your existing RealSense code
3. **Try the examples**: Test color detection with depth queries
4. **Build your tracker**: Combine aligned RGB-D for robust object detection

## For Your Overhead Project

You'll use alignment to:
- ✓ Detect colored ball in RGB → Get depth → Convert to world coordinates
- ✓ Segment robot using depth → Get color to identify it
- ✓ Detect walls/obstacles using depth edges → Verify with RGB
- ✓ Create accurate overhead map with both color and depth info

The alignment you implement this week will be the foundation for all your tracking work in Weeks 3-10!
