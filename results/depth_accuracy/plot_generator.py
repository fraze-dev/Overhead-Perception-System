import matplotlib.pyplot as plt

# Your test data
ground_truth = [50, 100, 150, 200, 250, 300]
abs_error = [0.44, 0.81, 0.75, 2.20, 2.37, 10.65]
rel_error = [0.89, 0.81, 0.5, 1.10, 0.95, 3.55]

# Create figure with two subplots
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

# Plot 1: Absolute Error
ax1.plot(ground_truth, abs_error, 'bo-', linewidth=2, markersize=8)
ax1.axhline(y=2, color='r', linestyle='--', label='2% at 200cm (4cm)')
ax1.set_xlabel('Distance (cm)', fontsize=12)
ax1.set_ylabel('Absolute Error (cm)', fontsize=12)
ax1.set_title('Absolute Error vs. Distance')
ax1.grid(True, alpha=0.3)
ax1.legend()

# Plot 2: Relative Error
ax2.plot(ground_truth, rel_error, 'go-', linewidth=2, markersize=8)
ax2.axhline(y=2, color='r', linestyle='--', label='2% Spec')
ax2.set_xlabel('Distance (cm)', fontsize=12)
ax2.set_ylabel('Relative Error (%)', fontsize=12)
ax2.set_title('Relative Error vs. Distance')
ax2.grid(True, alpha=0.3)
ax2.legend()

plt.tight_layout()
plt.savefig('depth_accuracy_error_plot.png', dpi=150, bbox_inches='tight')
print("Plot saved as 'depth_accuracy_error_plot.png'")
plt.show()