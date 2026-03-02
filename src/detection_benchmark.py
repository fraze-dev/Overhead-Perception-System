"""
Detection Methods Benchmarking Script
Author: Aaron Fraze
Date: February 16, 2026
Purpose: Compare performance and accuracy of different detection methods
"""

import sys
import time
import numpy as np
import cv2
import json
from datetime import datetime
from pathlib import Path

# Add project path to import the overhead perceptor
sys.path.insert(0, '/mnt/project')
from hsv_v2_with_depth_FIXED import OverheadPerceptor


class DetectionBenchmark:
    """
    Benchmark suite for testing detection methods.
    """
    
    def __init__(self, output_dir="results/benchmarks"):
        """Initialize benchmarking system."""
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Initialize perceptor
        print("Initializing camera...")
        self.perceptor = OverheadPerceptor()
        
        # Results storage
        self.results = {
            'timestamp': datetime.now().isoformat(),
            'methods': {}
        }
    
    def benchmark_fps(self, method_name, detection_func, num_frames=100):
        """
        Measure frames per second (FPS) for a detection method.
        
        Args:
            method_name: Name of the method being tested
            detection_func: Function that performs detection on a frame
            num_frames: Number of frames to test
            
        Returns:
            dict with timing statistics
        """
        print(f"\n{'='*60}")
        print(f"BENCHMARKING: {method_name} - FPS Test")
        print(f"{'='*60}")
        print(f"Processing {num_frames} frames...")
        
        frame_times = []
        successful_detections = 0
        
        for i in range(num_frames):
            # Get frame
            frames_data = self.perceptor.get_frame()
            if frames_data is None:
                continue
            
            # Time the detection
            start_time = time.perf_counter()
            result = detection_func(frames_data)
            end_time = time.perf_counter()
            
            frame_time = (end_time - start_time) * 1000  # Convert to ms
            frame_times.append(frame_time)
            
            # Check if detection was successful
            if result is not None:
                if isinstance(result, dict) and result.get('detected'):
                    successful_detections += 1
                elif isinstance(result, tuple):  # For depth segmentation
                    detected_objects = result[0]
                    if len(detected_objects) > 0:
                        successful_detections += 1
                elif isinstance(result, list) and len(result) > 0:
                    successful_detections += 1
            
            # Progress indicator
            if (i + 1) % 20 == 0:
                print(f"  Progress: {i + 1}/{num_frames} frames")
        
        # Calculate statistics
        frame_times_array = np.array(frame_times)
        mean_time_ms = np.mean(frame_times_array)
        std_time_ms = np.std(frame_times_array)
        min_time_ms = np.min(frame_times_array)
        max_time_ms = np.max(frame_times_array)
        
        # Calculate FPS
        mean_fps = 1000.0 / mean_time_ms if mean_time_ms > 0 else 0
        detection_rate = (successful_detections / num_frames) * 100
        
        results = {
            'method': method_name,
            'num_frames': num_frames,
            'mean_time_ms': mean_time_ms,
            'std_time_ms': std_time_ms,
            'min_time_ms': min_time_ms,
            'max_time_ms': max_time_ms,
            'mean_fps': mean_fps,
            'successful_detections': successful_detections,
            'detection_rate_pct': detection_rate,
            'frame_times_ms': frame_times
        }
        
        # Print summary
        print(f"\nRESULTS:")
        print(f"  Mean processing time: {mean_time_ms:.2f} ± {std_time_ms:.2f} ms")
        print(f"  Range: [{min_time_ms:.2f}, {max_time_ms:.2f}] ms")
        print(f"  Mean FPS: {mean_fps:.1f}")
        print(f"  Detection success rate: {detection_rate:.1f}% ({successful_detections}/{num_frames})")
        
        return results
    
    def benchmark_accuracy(self, method_name, detection_func, ground_truth_positions, num_measurements=10):
        """
        Measure spatial accuracy of a detection method.
        
        Args:
            method_name: Name of the method being tested
            detection_func: Function that performs detection
            ground_truth_positions: List of known (x, y) positions in cm
            num_measurements: Number of repeated measurements per position
            
        Returns:
            dict with accuracy statistics
        """
        print(f"\n{'='*60}")
        print(f"BENCHMARKING: {method_name} - Accuracy Test")
        print(f"{'='*60}")
        print(f"Testing {len(ground_truth_positions)} positions with {num_measurements} measurements each")
        
        all_errors = []
        position_results = []
        
        for pos_idx, gt_position in enumerate(ground_truth_positions):
            gt_x, gt_y = gt_position
            print(f"\n  Position {pos_idx + 1}/{len(ground_truth_positions)}: Ground Truth = ({gt_x:.1f}, {gt_y:.1f}) cm")
            input(f"    Place object at ({gt_x:.1f}, {gt_y:.1f}) cm and press ENTER...")
            
            measurements = []
            errors = []
            
            for meas_idx in range(num_measurements):
                frames_data = self.perceptor.get_frame()
                if frames_data is None:
                    continue
                
                result = detection_func(frames_data)
                
                if result is not None:
                    # Extract 3D position from result
                    point_3d = None
                    
                    if isinstance(result, dict) and 'point_3d' in result:
                        point_3d = result['point_3d']
                    elif isinstance(result, tuple):  # Depth segmentation
                        detected_objects = result[0]
                        if len(detected_objects) > 0:
                            # Take first detected object
                            point_3d = detected_objects[0]['point_3d']
                    elif isinstance(result, list) and len(result) > 0:
                        point_3d = result[0].get('point_3d')
                    
                    if point_3d is not None:
                        # Convert to world coordinates (cm)
                        measured_x = point_3d[0] * 100
                        measured_y = -point_3d[1] * 100  # Flip Y-axis
                        
                        # Calculate error
                        error_x = measured_x - gt_x
                        error_y = measured_y - gt_y
                        error_magnitude = np.sqrt(error_x**2 + error_y**2)
                        
                        measurements.append((measured_x, measured_y))
                        errors.append(error_magnitude)
                        
                        print(f"    Measurement {meas_idx + 1}: ({measured_x:.1f}, {measured_y:.1f}) cm | Error: {error_magnitude:.2f} cm")
            
            if len(measurements) > 0:
                # Calculate statistics for this position
                measurements_array = np.array(measurements)
                mean_x = np.mean(measurements_array[:, 0])
                mean_y = np.mean(measurements_array[:, 1])
                std_x = np.std(measurements_array[:, 0])
                std_y = np.std(measurements_array[:, 1])
                
                errors_array = np.array(errors)
                mean_error = np.mean(errors_array)
                std_error = np.std(errors_array)
                
                position_results.append({
                    'ground_truth_cm': gt_position,
                    'mean_measured_cm': (mean_x, mean_y),
                    'std_measured_cm': (std_x, std_y),
                    'mean_error_cm': mean_error,
                    'std_error_cm': std_error,
                    'num_measurements': len(measurements),
                    'all_errors_cm': errors
                })
                
                all_errors.extend(errors)
                
                print(f"    Position summary: Mean = ({mean_x:.1f}, {mean_y:.1f}) cm | Error = {mean_error:.2f} ± {std_error:.2f} cm")
        
        # Overall statistics
        if len(all_errors) > 0:
            all_errors_array = np.array(all_errors)
            overall_mean_error = np.mean(all_errors_array)
            overall_std_error = np.std(all_errors_array)
            overall_rmse = np.sqrt(np.mean(all_errors_array**2))
            overall_max_error = np.max(all_errors_array)
            
            results = {
                'method': method_name,
                'num_positions': len(ground_truth_positions),
                'measurements_per_position': num_measurements,
                'mean_error_cm': overall_mean_error,
                'std_error_cm': overall_std_error,
                'rmse_cm': overall_rmse,
                'max_error_cm': overall_max_error,
                'position_results': position_results
            }
            
            print(f"\n{'='*60}")
            print(f"OVERALL ACCURACY RESULTS:")
            print(f"  Mean error: {overall_mean_error:.2f} ± {overall_std_error:.2f} cm")
            print(f"  RMSE: {overall_rmse:.2f} cm")
            print(f"  Max error: {overall_max_error:.2f} cm")
            print(f"{'='*60}")
            
            return results
        else:
            print("\n⚠ WARNING: No successful measurements recorded!")
            return None
    
    def run_full_benchmark_suite(self):
        """
        Run complete benchmark suite on all detection methods.
        """
        print("\n" + "="*60)
        print("FULL DETECTION BENCHMARK SUITE")
        print("="*60)
        
        # Define detection methods to test
        detection_methods = {
            'hsv_ball': {
                'name': 'HSV Color Segmentation (Ball)',
                'func': lambda frames: self.perceptor.detect_ball_hsv(
                    frames['color_image'], 
                    frames['depth_image'], 
                    'orange_ball'
                )
            },
            'depth_segmentation': {
                'name': 'Depth-Based Segmentation',
                'func': lambda frames: self.perceptor.detect_elevated_objects_depth(
                    frames['depth_image'],
                    height_threshold_cm=10,
                    min_area=1000
                )
            }
        }
        
        # FPS Benchmarks
        print("\n" + "="*60)
        print("PHASE 1: FPS/SPEED BENCHMARKS")
        print("="*60)
        
        for method_key, method_info in detection_methods.items():
            fps_results = self.benchmark_fps(
                method_info['name'],
                method_info['func'],
                num_frames=100
            )
            self.results['methods'][method_key] = {'fps': fps_results}
            
            # Small delay between tests
            time.sleep(1)
        
        # Accuracy Benchmarks
        print("\n" + "="*60)
        print("PHASE 2: ACCURACY BENCHMARKS")
        print("="*60)
        print("You will be prompted to place objects at specific positions.")
        print("This allows measurement of spatial accuracy.")
        
        # Define test positions (you can modify these)
        test_positions = [
            (0, 0),      # Center
            (50, 0),     # Right
            (-50, 0),    # Left
            (0, 50),     # Forward
            (0, -50),    # Back
        ]
        
        proceed = input("\nProceed with accuracy testing? (y/n): ").strip().lower()
        
        if proceed == 'y':
            print("\n⚠ ACCURACY TEST PROTOCOL:")
            print("  1. You'll be prompted to place object at specific positions")
            print("  2. Measure positions carefully with tape measure")
            print("  3. Press ENTER when object is in position")
            print("  4. System will take multiple measurements")
            
            for method_key, method_info in detection_methods.items():
                accuracy_results = self.benchmark_accuracy(
                    method_info['name'],
                    method_info['func'],
                    test_positions,
                    num_measurements=10
                )
                
                if accuracy_results is not None:
                    self.results['methods'][method_key]['accuracy'] = accuracy_results
                
                time.sleep(1)
        else:
            print("Skipping accuracy tests.")
        
        # Save results
        self._save_results()
        self._generate_comparison_table()
        
        print("\n" + "="*60)
        print("BENCHMARK SUITE COMPLETE")
        print("="*60)
    
    def _save_results(self):
        """Save benchmark results to JSON file."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"detection_benchmark_{timestamp}.json"
        filepath = self.output_dir / filename
        
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(self.results, f, indent=2)
        
        print(f"\n✓ Benchmark results saved to: {filepath}")
    
    def _generate_comparison_table(self):
        """Generate markdown comparison table from results."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"detection_comparison_{timestamp}.md"
        filepath = self.output_dir / filename
        
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write("# Detection Methods Comparison\n\n")
            f.write(f"**Date:** {datetime.now().strftime('%B %d, %Y')}\n\n")
            f.write("---\n\n")
            
            # FPS Comparison Table
            f.write("## Performance Comparison (Speed)\n\n")
            f.write("| Method | Mean FPS | Mean Time (ms) | Detection Rate | Status |\n")
            f.write("|--------|----------|----------------|----------------|--------|\n")
            
            for method_key, method_data in self.results['methods'].items():
                if 'fps' in method_data:
                    fps_data = method_data['fps']
                    status = "✓ Fast" if fps_data['mean_fps'] >= 25 else "⚠ Slow"
                    
                    f.write(f"| {fps_data['method']} | "
                           f"{fps_data['mean_fps']:.1f} | "
                           f"{fps_data['mean_time_ms']:.2f} ± {fps_data['std_time_ms']:.2f} | "
                           f"{fps_data['detection_rate_pct']:.1f}% | "
                           f"{status} |\n")
            
            f.write("\n**Target:** ≥25 FPS for real-time tracking\n\n")
            f.write("---\n\n")
            
            # Accuracy Comparison Table
            if any('accuracy' in method_data for method_data in self.results['methods'].values()):
                f.write("## Accuracy Comparison (Spatial)\n\n")
                f.write("| Method | Mean Error (cm) | RMSE (cm) | Max Error (cm) | Status |\n")
                f.write("|--------|-----------------|-----------|----------------|--------|\n")
                
                for method_key, method_data in self.results['methods'].items():
                    if 'accuracy' in method_data:
                        acc_data = method_data['accuracy']
                        status = "✓ Good" if acc_data['mean_error_cm'] < 10 else "⚠ Poor"
                        
                        f.write(f"| {acc_data['method']} | "
                               f"{acc_data['mean_error_cm']:.2f} ± {acc_data['std_error_cm']:.2f} | "
                               f"{acc_data['rmse_cm']:.2f} | "
                               f"{acc_data['max_error_cm']:.2f} | "
                               f"{status} |\n")
                
                f.write("\n**Target:** <10 cm error for robot/ball tracking\n\n")
                f.write("---\n\n")
            
            # Method Summaries
            f.write("## Method Details\n\n")
            
            f.write("### 1. HSV Color Segmentation\n")
            f.write("- **Use Case:** Ball detection\n")
            f.write("- **Pros:** Fast, color-specific, robust once tuned\n")
            f.write("- **Cons:** Requires HSV tuning, sensitive to similar colors\n")
            if 'hsv_ball' in self.results['methods'] and 'fps' in self.results['methods']['hsv_ball']:
                fps = self.results['methods']['hsv_ball']['fps']['mean_fps']
                f.write(f"- **Performance:** {fps:.1f} FPS\n")
            f.write("\n")
            
            f.write("### 2. Depth-Based Segmentation\n")
            f.write("- **Use Case:** Robot/obstacle detection\n")
            f.write("- **Pros:** Color-independent, height-based filtering\n")
            f.write("- **Cons:** Noisy at edges, requires flat floor\n")
            if 'depth_segmentation' in self.results['methods'] and 'fps' in self.results['methods']['depth_segmentation']:
                fps = self.results['methods']['depth_segmentation']['fps']['mean_fps']
                f.write(f"- **Performance:** {fps:.1f} FPS\n")
            f.write("\n")
            
            f.write("---\n\n")
            f.write("## Recommendations\n\n")
            f.write("Based on benchmark results:\n\n")
            f.write("1. **Ball Detection:** Use HSV color segmentation\n")
            f.write("2. **Robot Detection:** Consider ArUco markers (Week 6) for robust pose estimation\n")
            f.write("3. **Obstacle Detection:** Use depth-based segmentation for static obstacles\n")
            f.write("4. **Multi-object Tracking:** Combine multiple methods for robustness\n\n")
        
        print(f"✓ Comparison table saved to: {filepath}")
    
    def shutdown(self):
        """Clean up resources."""
        self.perceptor.shutdown()


def main():
    print("="*60)
    print("Detection Methods Benchmark Suite")
    print("="*60)
    
    benchmark = DetectionBenchmark()
    
    try:
        print("\nBENCHMARK OPTIONS:")
        print("1. Quick FPS test only (2-3 minutes)")
        print("2. Full benchmark suite with accuracy (10-15 minutes)")
        print("3. FPS test for specific method")
        
        choice = input("\nEnter choice (1-3): ").strip()
        
        if choice == '1':
            # Quick FPS test
            print("\nRunning quick FPS tests...")
            
            # HSV ball detection
            hsv_results = benchmark.benchmark_fps(
                'HSV Color Segmentation (Ball)',
                lambda frames: benchmark.perceptor.detect_ball_hsv(
                    frames['color_image'], 
                    frames['depth_image'], 
                    'orange_ball'
                ),
                num_frames=100
            )
            benchmark.results['methods']['hsv_ball'] = {'fps': hsv_results}
            
            # Depth segmentation
            depth_results = benchmark.benchmark_fps(
                'Depth-Based Segmentation',
                lambda frames: benchmark.perceptor.detect_elevated_objects_depth(
                    frames['depth_image'],
                    height_threshold_cm=5,
                    min_area=500
                ),
                num_frames=100
            )
            benchmark.results['methods']['depth_segmentation'] = {'fps': depth_results}
            
            benchmark._save_results()
            benchmark._generate_comparison_table()
            
        elif choice == '2':
            # Full benchmark suite
            benchmark.run_full_benchmark_suite()
            
        elif choice == '3':
            print("\nSelect method to benchmark:")
            print("1. HSV Color Segmentation")
            print("2. Depth-Based Segmentation")
            
            method_choice = input("Enter choice (1-2): ").strip()
            
            if method_choice == '1':
                results = benchmark.benchmark_fps(
                    'HSV Color Segmentation (Ball)',
                    lambda frames: benchmark.perceptor.detect_ball_hsv(
                        frames['color_image'], 
                        frames['depth_image'], 
                        'orange_ball'
                    ),
                    num_frames=100
                )
                benchmark.results['methods']['hsv_ball'] = {'fps': results}
            elif method_choice == '2':
                results = benchmark.benchmark_fps(
                    'Depth-Based Segmentation',
                    lambda frames: benchmark.perceptor.detect_elevated_objects_depth(
                        frames['depth_image'],
                        height_threshold_cm=5,
                        min_area=500
                    ),
                    num_frames=100
                )
                benchmark.results['methods']['depth_segmentation'] = {'fps': results}
            
            benchmark._save_results()
            benchmark._generate_comparison_table()
        
        else:
            print("Invalid choice. Exiting.")
    
    except KeyboardInterrupt:
        print("\n\nBenchmark interrupted by user.")
    
    finally:
        benchmark.shutdown()
        print("\nBenchmark complete!")


if __name__ == "__main__":
    main()
