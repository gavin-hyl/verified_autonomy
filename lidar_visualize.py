#!/usr/bin/env python3
"""
Visualize LiDAR data saved by lidar_relay.py.

Usage:
  1. scp pi@192.168.12.1:~/lidar_capture.npz .
  2. python3 lidar_visualize.py lidar_capture.npz

Run on your PC (Windows or WSL) — requires numpy + matplotlib.
"""
import struct
import sys
import numpy as np

if len(sys.argv) < 2:
    print(f"Usage: {sys.argv[0]} <lidar_capture.npz>")
    print(f"  Copy from Pi first: scp pi@192.168.12.1:~/lidar_capture.npz .")
    sys.exit(1)

filepath = sys.argv[1]

print(f"Loading {filepath}...")

with open(filepath, 'rb') as f:
    n_points = struct.unpack('<I', f.read(4))[0]
    raw = f.read(n_points * 3 * 4)  # float32 x,y,z
    points = np.frombuffer(raw, dtype=np.float32).reshape(-1, 3)

print(f"  Points: {len(points)}")
print(f"  X range: [{points[:,0].min():.2f}, {points[:,0].max():.2f}] m")
print(f"  Y range: [{points[:,1].min():.2f}, {points[:,1].max():.2f}] m")
print(f"  Z range: [{points[:,2].min():.2f}, {points[:,2].max():.2f}] m")
print(f"  Max range: {np.max(np.linalg.norm(points, axis=1)):.2f} m")

# Filter out zero points
mask = np.any(points != 0, axis=1)
points = points[mask]
print(f"  Non-zero: {len(points)}")

# Downsample for plotting if too many
if len(points) > 100000:
    idx = np.random.choice(len(points), 100000, replace=False)
    plot_pts = points[idx]
    print(f"  Downsampled to 100k for display")
else:
    plot_pts = points

import matplotlib
matplotlib.use('Agg')  # Non-interactive backend (no display needed)
import matplotlib.pyplot as plt

fig, axes = plt.subplots(1, 3, figsize=(18, 5))

# Top-down (X-Y)
ax = axes[0]
ax.scatter(plot_pts[:, 0], plot_pts[:, 1], c=plot_pts[:, 2],
           cmap='viridis', s=0.1, alpha=0.3)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_title(f'Top-Down ({len(points)} pts)')
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)

# Side view (X-Z)
ax = axes[1]
ax.scatter(plot_pts[:, 0], plot_pts[:, 2], c=plot_pts[:, 1],
           cmap='viridis', s=0.1, alpha=0.3)
ax.set_xlabel('X (m)')
ax.set_ylabel('Z (m)')
ax.set_title('Side View (X-Z)')
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)

# Front view (Y-Z)
ax = axes[2]
ax.scatter(plot_pts[:, 1], plot_pts[:, 2], c=plot_pts[:, 0],
           cmap='viridis', s=0.1, alpha=0.3)
ax.set_xlabel('Y (m)')
ax.set_ylabel('Z (m)')
ax.set_title('Front View (Y-Z)')
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)

plt.suptitle(f'Livox Mid-360 — {len(points)} points from relay capture', fontsize=14)
plt.tight_layout()

# Save PNG
png_path = filepath.replace('.npz', '.png')
plt.savefig(png_path, dpi=150)
print(f"\n  Saved PNG → {png_path}")

plt.show()
print("Done.")
