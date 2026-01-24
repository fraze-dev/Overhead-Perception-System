"""
Depth Accuracy Testing Script for Intel RealSense D435
Author: Aaron Fraze
Date: January 21, 2026
Purpose: Systematically test depth accuracy and document results
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import time
import json
from datetime import datetime
from pathlib import Path
import matplotlib.pyplot as plt


class DepthAccuracyTester:
    """
    Automated depth accuracy testing for RealSense camera.
    """
    
    def __init__(self, output_dir="results/depth_accuracy"):
        """
        Initialize the depth accuracy tester.
        
        Args:
            output_dir: Directory to save results and figures
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Configure streams (640x480 is a good balance for accuracy testing)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # Start pipeline
        print("Starting RealSense camera...")
        self.profile = self.pipeline.start(self.config)
        
        # Get depth sensor and set options
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        
        print(f"Depth scale: {self.depth_scale} meters/unit")
        print("Camera initialized successfully!")
        
        # Allow camera to stabilize
        print("Warming up camera (3 seconds)...")
        for _ in range(90):  # Discard first 90 frames (3 seconds at 30fps)
            self.pipeline.wait_for_frames()
        
        print("Ready to test!\n")
        
    def capture_depth_samples(self, num_frames=100, roi_center=True, roi_size=(100, 100)):
        """
        Capture multiple depth frames and extract measurements.
        
        Args:
            num_frames: Number of frames to capture
            roi_center: If True, measure center region. If False, measure entire frame
            roi_size: Size of ROI in pixels (width, height)
            
        Returns:
            dict with depth statistics and raw measurements
        """
        print(f"Capturing {num_frames} depth frames...")
        depth_measurements = []
        
        for i in range(num_frames):
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            
            if not depth_frame:
                continue
            
            # Convert to numpy array
            depth_image = np.asanyarray(depth_frame.get_data())
            depth_meters = depth_image * self.depth_scale
            
            # Extract ROI or full frame
            if roi_center:
                h, w = depth_image.shape
                cx, cy = w // 2, h // 2
                roi_w, roi_h = roi_size[0] // 2, roi_size[1] // 2
                roi = depth_meters[cy-roi_h:cy+roi_h, cx-roi_w:cx+roi_w]
            else:
                roi = depth_meters
            
            # Get valid (non-zero) depth values
            valid_depths = roi[roi > 0]
            
            if len(valid_depths) > 0:
                depth_measurements.append(np.mean(valid_depths))
            
            # Progress indicator
            if (i + 1) % 20 == 0:
                print(f"  {i + 1}/{num_frames} frames captured")
        
        # Calculate statistics
        measurements_array = np.array(depth_measurements)
        
        stats = {
            'mean_meters': np.mean(measurements_array),
            'mean_cm': np.mean(measurements_array) * 100,
            'std_meters': np.std(measurements_array),
            'std_cm': np.std(measurements_array) * 100,
            'min_meters': np.min(measurements_array),
            'max_meters': np.max(measurements_array),
            'num_samples': len(measurements_array),
            'raw_measurements': measurements_array.tolist()
        }
        
        print(f"  Mean depth: {stats['mean_cm']:.2f} cm (±{stats['std_cm']:.2f} cm)")
        return stats
    
    def test_distance_accuracy(self, ground_truth_cm, num_frames=100, test_name=""):
        """
        Test depth accuracy at a specific distance against ground truth.
        
        Args:
            ground_truth_cm: Actual measured distance in centimeters
            num_frames: Number of frames to capture
            test_name: Optional name for this test
            
        Returns:
            dict with test results
        """
        print(f"\n{'='*60}")
        print(f"TEST: Distance Accuracy - {ground_truth_cm} cm")
        if test_name:
            print(f"Name: {test_name}")
        print(f"{'='*60}")
        
        # Capture samples
        stats = self.capture_depth_samples(num_frames=num_frames, roi_center=True)
        
        # Calculate errors
        absolute_error_cm = stats['mean_cm'] - ground_truth_cm
        relative_error_pct = (absolute_error_cm / ground_truth_cm) * 100
        
        # Calculate L1 and L2 losses
        measurements_cm = np.array(stats['raw_measurements']) * 100
        errors = measurements_cm - ground_truth_cm
        l1_loss_mae = np.mean(np.abs(errors))  # Mean Absolute Error
        l2_loss_rmse = np.sqrt(np.mean(errors ** 2))  # Root Mean Squared Error
        
        # Prepare results
        results = {
            'test_name': test_name,
            'timestamp': datetime.now().isoformat(),
            'ground_truth_cm': ground_truth_cm,
            'measured_depth_cm': stats['mean_cm'],
            'std_dev_cm': stats['std_cm'],
            'absolute_error_cm': absolute_error_cm,
            'relative_error_pct': relative_error_pct,
            'l1_loss_mae_cm': l1_loss_mae,
            'l2_loss_rmse_cm': l2_loss_rmse,
            'min_depth_cm': stats['min_meters'] * 100,
            'max_depth_cm': stats['max_meters'] * 100,
            'num_samples': stats['num_samples'],
            'raw_measurements_cm': [m * 100 for m in stats['raw_measurements']]
        }
        
        # Print summary
        print(f"\nRESULTS:")
        print(f"  Ground Truth:     {ground_truth_cm:.2f} cm")
        print(f"  Measured:         {stats['mean_cm']:.2f} ± {stats['std_cm']:.2f} cm")
        print(f"  Absolute Error:   {absolute_error_cm:.2f} cm")
        print(f"  Relative Error:   {relative_error_pct:.2f}%")
        print(f"  L1 Loss (MAE):    {l1_loss_mae:.2f} cm")
        print(f"  L2 Loss (RMSE):   {l2_loss_rmse:.2f} cm")
        print(f"  Range:            [{results['min_depth_cm']:.2f}, {results['max_depth_cm']:.2f}] cm")
        
        # Save results
        self._save_test_results(results)
        
        return results
    
    def test_spatial_uniformity(self, num_frames=100, grid_size=(5, 5)):
        """
        Test depth uniformity across the field of view.
        
        Args:
            num_frames: Number of frames to average
            grid_size: Grid dimensions (rows, cols)
            
        Returns:
            dict with spatial depth map
        """
        print(f"\n{'='*60}")
        print(f"TEST: Spatial Uniformity - {grid_size[0]}x{grid_size[1]} grid")
        print(f"{'='*60}")
        
        print(f"Capturing {num_frames} frames for spatial analysis...")
        
        # Accumulate depth frames
        depth_accumulator = None
        count = 0
        
        for i in range(num_frames):
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            
            if not depth_frame:
                continue
            
            depth_image = np.asanyarray(depth_frame.get_data())
            depth_meters = depth_image * self.depth_scale
            
            if depth_accumulator is None:
                depth_accumulator = np.zeros_like(depth_meters, dtype=np.float64)
            
            depth_accumulator += depth_meters
            count += 1
            
            if (i + 1) % 20 == 0:
                print(f"  {i + 1}/{num_frames} frames captured")
        
        # Calculate average depth map
        avg_depth_map = depth_accumulator / count
        
        # Sample grid
        h, w = avg_depth_map.shape
        rows, cols = grid_size
        
        grid_results = []
        
        print("\nSpatial Depth Map (cm):")
        print("-" * (cols * 10))
        
        for r in range(rows):
            row_values = []
            for c in range(cols):
                # Define grid cell boundaries
                y_start = int(r * h / rows)
                y_end = int((r + 1) * h / rows)
                x_start = int(c * w / cols)
                x_end = int((c + 1) * w / cols)
                
                # Extract cell
                cell = avg_depth_map[y_start:y_end, x_start:x_end]
                valid_cell = cell[cell > 0]
                
                if len(valid_cell) > 0:
                    mean_depth_cm = np.mean(valid_cell) * 100
                    std_depth_cm = np.std(valid_cell) * 100
                else:
                    mean_depth_cm = 0
                    std_depth_cm = 0
                
                grid_results.append({
                    'row': r,
                    'col': c,
                    'mean_cm': mean_depth_cm,
                    'std_cm': std_depth_cm
                })
                
                row_values.append(mean_depth_cm)
            
            # Print row
            print(" | ".join([f"{v:6.2f}" for v in row_values]))
        
        results = {
            'test_name': 'spatial_uniformity',
            'timestamp': datetime.now().isoformat(),
            'grid_size': grid_size,
            'grid_data': grid_results,
            'num_frames': count
        }
        
        self._save_test_results(results)
        
        return results
    
    def test_repeatability(self, num_frames=1000):
        """
        Test depth measurement repeatability at a fixed position.
        
        Args:
            num_frames: Number of frames to capture (more = better statistics)
            
        Returns:
            dict with repeatability statistics
        """
        print(f"\n{'='*60}")
        print(f"TEST: Repeatability/Precision - {num_frames} frames")
        print(f"{'='*60}")
        
        stats = self.capture_depth_samples(num_frames=num_frames, roi_center=True)
        
        # Calculate additional statistics
        measurements = np.array(stats['raw_measurements']) * 100  # Convert to cm
        
        results = {
            'test_name': 'repeatability',
            'timestamp': datetime.now().isoformat(),
            'mean_cm': stats['mean_cm'],
            'std_dev_cm': stats['std_cm'],
            'min_cm': np.min(measurements),
            'max_cm': np.max(measurements),
            'range_cm': np.max(measurements) - np.min(measurements),
            'num_samples': len(measurements),
            'measurements_cm': measurements.tolist()
        }
        
        print(f"\nREPEATABILITY RESULTS:")
        print(f"  Mean:       {results['mean_cm']:.3f} cm")
        print(f"  Std Dev:    {results['std_dev_cm']:.3f} cm")
        print(f"  Range:      {results['range_cm']:.3f} cm")
        print(f"  Min/Max:    [{results['min_cm']:.3f}, {results['max_cm']:.3f}] cm")
        
        self._save_test_results(results)
        
        return results
    
    def _save_test_results(self, results):
        """Save test results to JSON file."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        test_name = results.get('test_name', 'test')
        filename = f"{timestamp}_{test_name}.json"
        filepath = self.output_dir / filename
        
        with open(filepath, 'w') as f:
            json.dump(results, f, indent=2)
        
        print(f"\n✓ Results saved to: {filepath}")
    
    def visualize_test_results(self, results_file):
        """
        Create visualization plots from test results.
        
        Args:
            results_file: Path to JSON results file
        """
        with open(results_file, 'r') as f:
            results = json.load(f)
        
        test_name = results.get('test_name', 'test')
        
        if 'raw_measurements_cm' in results:
            # Plot histogram of measurements
            measurements = np.array(results['raw_measurements_cm'])
            
            plt.figure(figsize=(10, 6))
            plt.hist(measurements, bins=50, edgecolor='black', alpha=0.7)
            plt.axvline(results['ground_truth_cm'], color='r', linestyle='--', 
                       linewidth=2, label=f"Ground Truth: {results['ground_truth_cm']:.2f} cm")
            plt.axvline(results['measured_depth_cm'], color='g', linestyle='--', 
                       linewidth=2, label=f"Mean: {results['measured_depth_cm']:.2f} cm")
            
            plt.xlabel('Measured Depth (cm)')
            plt.ylabel('Frequency')
            plt.title(f'Depth Measurement Distribution - {test_name}')
            plt.legend()
            plt.grid(alpha=0.3)
            
            plot_path = self.output_dir / f"{test_name}_histogram.png"
            plt.savefig(plot_path, dpi=150, bbox_inches='tight')
            print(f"✓ Plot saved to: {plot_path}")
            plt.close()
    
    def shutdown(self):
        """Stop the camera pipeline."""
        print("\nShutting down camera...")
        self.pipeline.stop()
        print("Done!")


# Example usage
if __name__ == "__main__":
    print("="*60)
    print("RealSense D435 Depth Accuracy Testing")
    print("="*60)
    print()
    
    # Initialize tester
    tester = DepthAccuracyTester()
    
    try:
        print("\n" + "="*60)
        print("EXPERIMENT GUIDE")
        print("="*60)
        print("\n1. Position your camera to face a flat wall")
        print("2. Measure the distance from camera to wall using tape measure")
        print("3. Run tests for each distance you want to evaluate")
        print()
        print("Recommended test distances: 50, 100, 150, 200, 250, 300 cm")
        print("="*60)
        
        # Interactive mode
        while True:
            print("\n" + "="*60)
            print("TEST MENU")
            print("="*60)
            print("1. Distance Accuracy Test (single distance)")
            print("2. Multiple Distance Tests (automated)")
            print("3. Spatial Uniformity Test")
            print("4. Repeatability/Precision Test")
            print("5. Exit")
            print("="*60)
            
            choice = input("\nEnter choice (1-5): ").strip()
            
            if choice == '1':
                # Single distance test
                distance_str = input("Enter ground truth distance (cm): ").strip()
                try:
                    distance_cm = float(distance_str)
                    tester.test_distance_accuracy(distance_cm, num_frames=100)
                except ValueError:
                    print("Invalid distance. Please enter a number.")
            
            elif choice == '2':
                # Multiple distances (batch mode)
                print("\nEnter distances to test (comma-separated in cm)")
                print("Example: 50, 100, 150, 200")
                distances_str = input("Distances: ").strip()
                
                try:
                    distances = [float(d.strip()) for d in distances_str.split(',')]
                    print(f"\nWill test {len(distances)} distances: {distances}")
                    input("Press ENTER to start (position camera at first distance)...")
                    
                    results_all = []
                    for i, dist in enumerate(distances, 1):
                        print(f"\n\n*** Distance {i}/{len(distances)}: {dist} cm ***")
                        input(f"Position camera at {dist} cm and press ENTER...")
                        result = tester.test_distance_accuracy(dist, num_frames=100,
                                                               test_name=f"dist_{dist}cm")
                        results_all.append(result)
                    
                    print("\n" + "="*60)
                    print("ALL TESTS COMPLETE - SUMMARY")
                    print("="*60)
                    for r in results_all:
                        print(f"{r['ground_truth_cm']:6.1f} cm: "
                              f"Measured {r['measured_depth_cm']:6.2f} cm | "
                              f"Error {r['absolute_error_cm']:+6.2f} cm ({r['relative_error_pct']:+5.2f}%) | "
                              f"MAE {r['l1_loss_mae_cm']:5.2f} cm | RMSE {r['l2_loss_rmse_cm']:5.2f} cm")
                    
                except ValueError:
                    print("Invalid input. Please use comma-separated numbers.")
            
            elif choice == '3':
                # Spatial uniformity
                distance_str = input("Enter approximate distance to wall (cm): ").strip()
                tester.test_spatial_uniformity(num_frames=100)
            
            elif choice == '4':
                # Repeatability test
                distance_str = input("Enter approximate distance to wall (cm): ").strip()
                tester.test_repeatability(num_frames=1000)
            
            elif choice == '5':
                break
            
            else:
                print("Invalid choice. Please enter 1-5.")
    
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user.")
    
    finally:
        tester.shutdown()
        print("\nThank you for testing!")
