import matplotlib.pyplot as plt
import numpy as np

# Your spatial uniformity data (5x5 grid at 150 cm)
spatial_data = np.array([
    [159.66, 158.59, 155.20, 150.16, 147.02],
    [159.61, 156.81, 153.85, 149.47, 147.11],
    [158.07, 153.86, 150.90, 147.92, 145.74],
    [155.21, 150.78, 149.28, 146.49, 144.36],
    [153.28, 149.51, 147.85, 146.11, 142.98]
])

# Create heatmap
fig, ax = plt.subplots(figsize=(10, 8))
im = ax.imshow(spatial_data, cmap='RdYlGn_r', aspect='auto')

# Add colorbar
cbar = plt.colorbar(im, ax=ax)
cbar.set_label('Depth (cm)', rotation=270, labelpad=20, fontsize=12)

# Add text annotations
for i in range(5):
    for j in range(5):
        text = ax.text(j, i, f'{spatial_data[i, j]:.1f}',
                      ha="center", va="center", color="black", fontsize=10)

# Labels
ax.set_xticks(np.arange(5))
ax.set_yticks(np.arange(5))
ax.set_xticklabels(['Left', '', 'Center', '', 'Right'])
ax.set_yticklabels(['Top', '', 'Center', '', 'Bottom'])
ax.set_xlabel('Horizontal Position', fontsize=12)
ax.set_ylabel('Vertical Position', fontsize=12)
ax.set_title('Spatial Uniformity - Depth Map at 150 cm', fontsize=14)

plt.tight_layout()
plt.savefig('spatial_uniformity_heatmap.png', dpi=150, bbox_inches='tight')
print("✓ Heatmap saved as 'spatial_uniformity_heatmap.png'")
plt.show()