import pyrealsense2 as rs
import numpy as np

# Test to verify camera is working with Python
# Should receive 5 lines depth data
pipeline = rs.pipeline()

pipeline.start()

try:
    for _ in range(5):
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        if depth_frame:
            depth_image = np.asanyarray(depth_frame.get_data())
            print(f"Depth frame received: {depth_image.shape}")

finally:
    pipeline.stop()