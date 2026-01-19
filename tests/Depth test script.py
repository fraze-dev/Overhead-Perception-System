import pyrealsense2 as rs
import numpy as np
import cv2

# Create a pipeline
pipeline = rs.pipeline()

# Configure streams
config = rs.config()
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Create colorizer for depth visualization
colorizer = rs.colorizer()

# Variables for mouse click
click_x, click_y = 424, 240  # Start at center


def mouse_callback(event, x, y, flags, param):
    global click_x, click_y
    if event == cv2.EVENT_LBUTTONDOWN:
        click_x, click_y = x, y


# Set up mouse callback
cv2.namedWindow('Color Image')
cv2.setMouseCallback('Color Image', mouse_callback)

try:
    print("Streaming... Press 'q' to exit")
    print("Click on the color image to check depth at that point")

    while True:
        # Wait for frames
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # Convert to numpy arrays
        depth_image = np.asanyarray(colorizer.colorize(depth_frame).get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Get depth at center point
        center_x, center_y = 424, 240
        center_depth = depth_frame.get_distance(center_x, center_y)

        # Get depth at clicked point
        click_depth = depth_frame.get_distance(click_x, click_y)

        # Draw crosshairs and text on color image
        cv2.circle(color_image, (center_x, center_y), 5, (0, 255, 0), 2)
        cv2.circle(color_image, (click_x, click_y), 5, (0, 0, 255), 2)

        # Add text with depth values
        cv2.putText(color_image, f"Center: {center_depth:.2f}m",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(color_image, f"Click ({click_x},{click_y}): {click_depth:.2f}m",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Display images
        cv2.imshow('Color Image', color_image)
        cv2.imshow('Depth Image', depth_image)

        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()