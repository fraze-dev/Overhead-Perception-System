import pyrealsense2 as rs
import numpy as np
import cv2

pipeline = rs.pipeline()

config = rs.config()
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)

pipeline.start(config)

colorizer = rs.colorizer()



try:
    print("Streaming... Press 'q' to exit")
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # Convert to numpy arrays
        depth_image = np.asanyarray(colorizer.colorize(depth_frame).get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Display images
        cv2.imshow('Color Image', color_image)
        cv2.imshow('Depth Image', depth_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()