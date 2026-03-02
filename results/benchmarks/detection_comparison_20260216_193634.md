# Detection Methods Comparison

**Date:** February 16, 2026

---

## Performance Comparison (Speed)

| Method | Mean FPS | Mean Time (ms) | Detection Rate | Status |
|--------|----------|----------------|----------------|--------|
| HSV Color Segmentation (Ball) | 306.8 | 3.26 ± 0.46 | 100.0% | ✓ Fast |
| Depth-Based Segmentation | 111.0 | 9.01 ± 1.23 | 100.0% | ✓ Fast |

**Target:** ≥25 FPS for real-time tracking

---

## Accuracy Comparison (Spatial)

| Method | Mean Error (cm) | RMSE (cm) | Max Error (cm) | Status |
|--------|-----------------|-----------|----------------|--------|
| HSV Color Segmentation (Ball) | 3.31 ± 2.27 | 4.01 | 7.11 | ✓ Good |
| Depth-Based Segmentation | 2.85 ± 1.73 | 3.33 | 5.16 | ✓ Good |

**Target:** <10 cm error for robot/ball tracking

---

## Method Details

### 1. HSV Color Segmentation
- **Use Case:** Ball detection
- **Pros:** Fast, color-specific, robust once tuned
- **Cons:** Requires HSV tuning, sensitive to similar colors
- **Performance:** 306.8 FPS

### 2. Depth-Based Segmentation
- **Use Case:** Robot/obstacle detection
- **Pros:** Color-independent, height-based filtering
- **Cons:** Noisy at edges, requires flat floor
- **Performance:** 111.0 FPS

---

## Recommendations

Based on benchmark results:

1. **Ball Detection:** Use HSV color segmentation
2. **Robot Detection:** Consider ArUco markers (Week 6) for robust pose estimation
3. **Obstacle Detection:** Use depth-based segmentation for static obstacles
4. **Multi-object Tracking:** Combine multiple methods for robustness

