#!/usr/bin/env python3
"""
Quick LiDAR data test - listens on the point cloud UDP port,
saves data to .npy, and prints stats. Run this INSTEAD of
the ROS livox driver (stop the driver first).

Usage:
  1. Kill livox driver: pkill -f livox_ros_driver
  2. python3 ~/lidar_test.py
  3. View result: the script saves pointcloud.npy + a PNG image

Run from WSL (not Docker) since they share the network.
"""
import socket
import struct
import sys
import time
import numpy as np

PORT = 56301
DURATION = 5  # seconds
POINT_SIZE = 14  # int32 x,y,z + uint8 reflectivity + uint8 tag

print("=" * 55)
print("  Livox LiDAR Quick Data Test")
print("=" * 55)

# Check if port is available
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.settimeout(2.0)

try:
    sock.bind(("0.0.0.0", PORT))
    print(f"  ✓ Bound to 0.0.0.0:{PORT}")
except OSError as e:
    print(f"  ✗ Cannot bind to port {PORT}: {e}")
    print(f"    → Stop the ROS driver first: pkill -f livox_ros_driver")
    sys.exit(1)

print(f"  Listening for {DURATION}s...\n")

all_points = []
reflectivities = []
packet_count = 0
parse_ok = 0
start = time.time()

while time.time() - start < DURATION:
    try:
        data, addr = sock.recvfrom(65535)
        packet_count += 1

        if packet_count == 1:
            print(f"  ✓ First packet: {len(data)} bytes from {addr}")
            print(f"    Header (hex): {data[:24].hex()}")
            print(f"    data_type={data[4]}, version={data[0]}")
            print()

        # Try common header sizes
        for hdr in [18, 24, 28, 20, 22, 26]:
            payload = data[hdr:]
            if len(payload) >= POINT_SIZE and len(payload) % POINT_SIZE == 0:
                n = len(payload) // POINT_SIZE
                for i in range(n):
                    off = i * POINT_SIZE
                    x, y, z = struct.unpack_from('<iii', payload, off)
                    ref = payload[off + 12]
                    if abs(x) < 200000 and abs(y) < 200000 and abs(z) < 200000:
                        all_points.append((x / 1000.0, y / 1000.0, z / 1000.0))
                        reflectivities.append(ref)
                if n > 0:
                    parse_ok += 1
                break

        if packet_count % 1000 == 0:
            elapsed = time.time() - start
            print(f"  {packet_count} packets, {len(all_points)} points ({elapsed:.1f}s)")

    except socket.timeout:
        if packet_count == 0:
            print("  ⏳ No data yet... Is the Pi relay running?")

sock.close()

if packet_count == 0:
    print("\n  ✗ No UDP packets received!")
    print("    Check: relay running? Firewall open? Connected to Go1 hotspot?")
    sys.exit(1)

if not all_points:
    print(f"\n  Received {packet_count} packets but could not parse any points.")
    print("  Run: python3 ~/lidar_view.py --dump")
    sys.exit(1)

points = np.array(all_points)
refs = np.array(reflectivities)

print(f"\n{'='*55}")
print(f"  RESULTS")
print(f"{'='*55}")
print(f"  Packets received:  {packet_count}")
print(f"  Packets parsed:    {parse_ok}")
print(f"  Total points:      {len(points)}")
print(f"  Points/sec:        {len(points)/DURATION:.0f}")
print(f"  X range: [{points[:,0].min():.2f}, {points[:,0].max():.2f}] m")
print(f"  Y range: [{points[:,1].min():.2f}, {points[:,1].max():.2f}] m")
print(f"  Z range: [{points[:,2].min():.2f}, {points[:,2].max():.2f}] m")
print(f"  Max range: {np.max(np.linalg.norm(points, axis=1)):.2f} m")

# Save raw data
outfile = "/tmp/pointcloud.npy"
np.save(outfile, points)
print(f"\n  Saved → {outfile}")

# Also save to Windows-accessible location
win_out = "/mnt/c/Users/willi/.vscode/pointcloud.npy"
np.save(win_out, points)
print(f"  Saved → {win_out}")

# Try to generate a PNG (top-down view)
try:
    import matplotlib
    matplotlib.use('Agg')  # No display needed
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 3, figsize=(18, 5))

    # Top-down (X-Y)
    ax = axes[0]
    ax.scatter(points[::3, 0], points[::3, 1], c=points[::3, 2],
               cmap='viridis', s=0.1, alpha=0.3)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'Top-Down View ({len(points)} pts)')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # Side view (X-Z)
    ax = axes[1]
    ax.scatter(points[::3, 0], points[::3, 2], c=points[::3, 1],
               cmap='viridis', s=0.1, alpha=0.3)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Z (m)')
    ax.set_title('Side View (X-Z)')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # Front view (Y-Z)
    ax = axes[2]
    ax.scatter(points[::3, 1], points[::3, 2], c=points[::3, 0],
               cmap='viridis', s=0.1, alpha=0.3)
    ax.set_xlabel('Y (m)')
    ax.set_ylabel('Z (m)')
    ax.set_title('Front View (Y-Z)')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    plt.suptitle(f'Livox Mid-360 — {len(points)} points in {DURATION}s', fontsize=14)
    plt.tight_layout()

    png_path = "/mnt/c/Users/willi/.vscode/pointcloud.png"
    plt.savefig(png_path, dpi=150)
    print(f"  Saved → {png_path}")
    print(f"\n  Open pointcloud.png in VS Code to see the result!")

except Exception as e:
    print(f"\n  (Could not generate PNG: {e})")
    print(f"  Open pointcloud.npy in Python to visualize manually.")

print(f"\n{'='*55}")
print(f"  ✓ LiDAR data is flowing!")
print(f"{'='*55}")
