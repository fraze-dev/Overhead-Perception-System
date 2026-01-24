import json
import matplotlib.pyplot as plt
import numpy as np

# Load your repeatability test results
with open('results/depth_accuracy/20260124_151205_repeatability.json', 'r') as f:
    data = json.load(f)

measurements = np.array(data['measurements_cm'])

# Create histogram
plt.figure(figsize=(10, 6))
plt.hist(measurements, bins=50, edgecolor='black', alpha=0.7)
plt.axvline(data['mean_cm'], color='r', linestyle='--', linewidth=2,
            label=f"Mean: {data['mean_cm']:.2f} cm")
plt.axvline(data['mean_cm'] - data['std_dev_cm'], color='g', linestyle=':', linewidth=1.5,
            label=f"±1 Std Dev: {data['std_dev_cm']:.2f} cm")
plt.axvline(data['mean_cm'] + data['std_dev_cm'], color='g', linestyle=':', linewidth=1.5)

plt.xlabel('Measured Depth (cm)', fontsize=12)
plt.ylabel('Frequency', fontsize=12)
plt.title('Depth Measurement Distribution - Repeatability Test', fontsize=14)
plt.legend()
plt.grid(alpha=0.3)

plt.savefig('repeatability_histogram.png', dpi=150, bbox_inches='tight')
print("Histogram saved!")
plt.show()