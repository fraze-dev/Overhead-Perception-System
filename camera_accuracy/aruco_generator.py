import numpy as np
import cv2

ARUCO_DICT = {
    "DICT_4x4_50": cv2.aruco.DICT_4X4_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
}

# Configuration
a_type = "DICT_4x4_50"
marker_id = 5
marker_size = 300  # Size in pixels

# Get the ArUco dictionary (new API)
aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[a_type])

# Generate the marker (new API)
marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)

# Display the marker
cv2.imshow(f"ArUco Marker - {a_type} ID:{marker_id}", marker_image)
print(f"Displaying marker. Press any key to save and exit...")
cv2.waitKey(0)
cv2.destroyAllWindows()

# Save the marker
filename = f"aruco_marker_{a_type}_id{marker_id}.png"
cv2.imwrite(filename, marker_image)
print(f"Marker saved as: {filename}")