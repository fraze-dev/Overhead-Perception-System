# Detection Methods Comparison

**Date:** February 19, 2026

---

## Performance Comparison (Speed)

| Method | Mean FPS | Mean Time (ms) | Detection Rate | Status |
|--------|----------|----------------|----------------|--------|
| Depth-Based Segmentation | 89.6 | 11.16 ± 6.35 | 100.0% | ✓ Fast |

**Target:** ≥25 FPS for real-time tracking

---

## Method Details

### 1. HSV Color Segmentation
- **Use Case:** Ball detection
- **Pros:** Fast, color-specific, robust once tuned
- **Cons:** Requires HSV tuning, sensitive to similar colors

### 2. Depth-Based Segmentation
- **Use Case:** Robot/obstacle detection
- **Pros:** Color-independent, height-based filtering
- **Cons:** Noisy at edges, requires flat floor
- **Performance:** 89.6 FPS

---

## Recommendations

Based on benchmark results:

1. **Ball Detection:** Use HSV color segmentation
2. **Robot Detection:** Consider ArUco markers (Week 6) for robust pose estimation
3. **Obstacle Detection:** Use depth-based segmentation for static obstacles
4. **Multi-object Tracking:** Combine multiple methods for robustness

