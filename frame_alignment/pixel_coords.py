import numpy as np
import cv2
import pyrealsense2 as rs


class CoordinateConverter:
    """
    Convert between pixel coordinates and centered coordinate systems.
    """

    def __init__(self, width=640, height=480):
        """
        Initialize converter with image dimensions.

        Args:
            width: Image width in pixels
            height: Image height in pixels
        """
        self.width = width
        self.height = height
        self.cx = width / 2.0  # Center x
        self.cy = height / 2.0  # Center y

    def pixel_to_centered(self, pixel_x, pixel_y):
        """
        Convert pixel coordinates to centered coordinate system.
        Origin (0,0) is at image center.
        +X is right, +Y is down (image convention)

        Args:
            pixel_x: Pixel x coordinate (0 to width-1)
            pixel_y: Pixel y coordinate (0 to height-1)

        Returns:
            (x, y) in centered coordinates
        """
        x = pixel_x - self.cx
        y = pixel_y - self.cy
        return x, y

    def centered_to_pixel(self, x, y):
        """
        Convert centered coordinates back to pixel coordinates.

        Args:
            x: Centered x coordinate
            y: Centered y coordinate

        Returns:
            (pixel_x, pixel_y) in pixel coordinates
        """
        pixel_x = x + self.cx
        pixel_y = y + self.cy
        return int(pixel_x), int(pixel_y)

    def pixel_to_centered_cartesian(self, pixel_x, pixel_y):
        """
        Convert to centered coordinates with Cartesian convention.
        Origin (0,0) at center, +X right, +Y UP (math convention)

        Args:
            pixel_x: Pixel x coordinate
            pixel_y: Pixel y coordinate

        Returns:
            (x, y) in Cartesian centered coordinates
        """
        x = pixel_x - self.cx
        y = -(pixel_y - self.cy)  # Flip Y axis
        return x, y

    def create_coordinate_grid_visualization(self, image=None):
        """
        Create a visualization showing the coordinate grid overlay.

        Args:
            image: Optional background image (BGR format)

        Returns:
            Image with coordinate grid overlay
        """
        if image is None:
            # Create blank image
            vis = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        else:
            vis = image.copy()

        # Draw grid lines every 50 pixels
        grid_spacing = 50

        # Vertical lines
        for x in range(0, self.width, grid_spacing):
            cv2.line(vis, (x, 0), (x, self.height), (50, 50, 50), 1)

        # Horizontal lines
        for y in range(0, self.height, grid_spacing):
            cv2.line(vis, (0, y), (self.width, y), (50, 50, 50), 1)

        # Draw axes through center
        center_x = int(self.cx)
        center_y = int(self.cy)

        # X axis (red) - horizontal through center
        cv2.line(vis, (0, center_y), (self.width, center_y), (0, 0, 255), 2)
        # Y axis (green) - vertical through center
        cv2.line(vis, (center_x, 0), (center_x, self.height), (0, 255, 0), 2)

        # Draw origin
        cv2.circle(vis, (center_x, center_y), 5, (255, 255, 255), -1)
        cv2.putText(vis, "(0,0)", (center_x + 10, center_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Label corners with centered coordinates
        corners = [
            (0, 0, "top-left"),
            (self.width - 1, 0, "top-right"),
            (0, self.height - 1, "bottom-left"),
            (self.width - 1, self.height - 1, "bottom-right")
        ]

        for px, py, label in corners:
            cx, cy = self.pixel_to_centered(px, py)
            text = f"({cx:.0f},{cy:.0f})"
            cv2.putText(vis, text, (px + 5, py + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

        return vis


# Example usage
if __name__ == "__main__":
    # Initialize converter for 640x480 image
    converter = CoordinateConverter(640, 480)

    # Example: Convert some pixel coordinates
    print("Pixel to Centered Coordinate Conversion:")
    print("=" * 50)

    test_points = [
        (0, 0, "Top-left corner"),
        (640, 0, "Top-right corner"),
        (0, 480, "Bottom-left corner"),
        (640, 480, "Bottom-right corner"),
        (320, 240, "Center"),
        (420, 340, "Example point")
    ]

    for px, py, desc in test_points:
        cx, cy = converter.pixel_to_centered(px, py)
        cx_cart, cy_cart = converter.pixel_to_centered_cartesian(px, py)
        print(f"{desc:20s} | Pixel: ({px:3d},{py:3d}) → "
              f"Centered: ({cx:6.1f},{cy:6.1f}) | "
              f"Cartesian: ({cx_cart:6.1f},{cy_cart:6.1f})")

    # Create visualization
    print("\nCreating coordinate grid visualization...")
    grid_vis = converter.create_coordinate_grid_visualization()

    cv2.imshow("Coordinate Grid", grid_vis)
    print("Press any key to close...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()