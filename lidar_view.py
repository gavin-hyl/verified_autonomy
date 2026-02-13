#!/usr/bin/env python3
"""
Livox LiDAR Point Cloud Viewer - Run on your PC (WSL or Docker)
Receives raw UDP point cloud packets from the Pi relay and visualizes them.
No Livox SDK or ROS required — just listens for forwarded UDP data.

Usage: python3 lidar_view.py [--port 56301] [--duration 5]

Dependencies: pip3 install matplotlib numpy
Optional:     pip3 install open3d  (for interactive 3D view)
"""
import socket
import struct
import sys
import time
import argparse
import threading
import numpy as np

# Livox SDK2 packet header (before point data)
# The exact header varies but the point data offset can be detected.
# For MID-360 high-res Cartesian (pcl_data_type=1):
#   Each point: x(int32 mm), y(int32 mm), z(int32 mm), reflectivity(uint8), tag(uint8) = 14 bytes

POINT_SIZE_HIGH = 14   # High-res Cartesian: 3x int32 + uint8 + uint8
POINT_SIZE_LOW  = 8    # Low-res Cartesian: 3x int16 + uint8 + uint8


def parse_livox_packet(data):
    """
    Parse a Livox SDK2 UDP point cloud packet.
    Returns numpy array of (x, y, z) in meters, or None on failure.
    """
    if len(data) < 28:
        return None

    # Try to find the point data section.
    # Livox SDK2 packet format:
    #   Byte 0:    version
    #   Byte 1-2:  length (uint16 LE)
    #   Byte 3:    time_type
    #   Byte 4:    data_type (1=high Cartesian, 2=low Cartesian, etc.)
    #   Byte 5-12: timestamp (uint64 LE)
    #   ... additional header fields
    #   Then point data
    #
    # The header size varies by SDK version. We try common offsets.

    data_type = data[4]

    # Try known header sizes (18, 24, 28 bytes are common)
    for header_size in [18, 24, 28, 20, 22, 26]:
        payload = data[header_size:]
        payload_len = len(payload)

        if data_type == 1 or data_type == 0:
            # High-res Cartesian
            if payload_len >= POINT_SIZE_HIGH and payload_len % POINT_SIZE_HIGH == 0:
                n_points = payload_len // POINT_SIZE_HIGH
                points = []
                for i in range(n_points):
                    offset = i * POINT_SIZE_HIGH
                    x, y, z = struct.unpack_from('<iii', payload, offset)
                    # Convert from mm to meters
                    points.append((x / 1000.0, y / 1000.0, z / 1000.0))
                if points:
                    return np.array(points)

        elif data_type == 2:
            # Low-res Cartesian
            if payload_len >= POINT_SIZE_LOW and payload_len % POINT_SIZE_LOW == 0:
                n_points = payload_len // POINT_SIZE_LOW
                points = []
                for i in range(n_points):
                    offset = i * POINT_SIZE_LOW
                    x, y, z = struct.unpack_from('<hhh', payload, offset)
                    # Convert from cm to meters
                    points.append((x / 100.0, y / 100.0, z / 100.0))
                if points:
                    return np.array(points)

    # Fallback: try high-res with every possible offset
    for header_size in range(10, 40):
        payload = data[header_size:]
        payload_len = len(payload)
        if payload_len >= POINT_SIZE_HIGH and payload_len % POINT_SIZE_HIGH == 0:
            n_points = payload_len // POINT_SIZE_HIGH
            points = []
            valid = True
            for i in range(min(n_points, 5)):  # Sanity check first 5 points
                offset = i * POINT_SIZE_HIGH
                x, y, z = struct.unpack_from('<iii', payload, offset)
                # Points should be within reasonable range (< 200m)
                if abs(x) > 200000 or abs(y) > 200000 or abs(z) > 200000:
                    valid = False
                    break
                points.append((x / 1000.0, y / 1000.0, z / 1000.0))
            if valid and points:
                # Parse all points
                points = []
                for i in range(n_points):
                    offset = i * POINT_SIZE_HIGH
                    x, y, z = struct.unpack_from('<iii', payload, offset)
                    points.append((x / 1000.0, y / 1000.0, z / 1000.0))
                return np.array(points)

    return None


def collect_points(port, duration, bind_ip="0.0.0.0"):
    """Collect point cloud data from UDP for a given duration."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(2.0)

    try:
        sock.bind((bind_ip, port))
    except OSError as e:
        print(f"Cannot bind to {bind_ip}:{port} - {e}")
        print("If another process holds the port, try: pkill -9 -f livox")
        return None

    print(f"Listening on {bind_ip}:{port} for {duration}s...")

    all_points = []
    packet_count = 0
    parse_fail = 0
    start = time.time()

    while time.time() - start < duration:
        try:
            data, addr = sock.recvfrom(65535)
            packet_count += 1
            pts = parse_livox_packet(data)
            if pts is not None and len(pts) > 0:
                all_points.append(pts)
            else:
                parse_fail += 1

            if packet_count == 1:
                print(f"  First packet: {len(data)} bytes from {addr}")
                print(f"  Header bytes: {data[:20].hex()}")
                print(f"  data_type byte: {data[4]}")

            if packet_count % 500 == 0:
                total = sum(len(p) for p in all_points)
                elapsed = time.time() - start
                print(f"  {packet_count} packets, {total} points ({elapsed:.1f}s)")

        except socket.timeout:
            if packet_count == 0:
                print("  No data received yet...")
            continue

    sock.close()

    if not all_points:
        print(f"\nReceived {packet_count} packets but parsed 0 points ({parse_fail} parse failures)")
        if packet_count > 0:
            print("The packet format may differ. Try running with --dump to inspect raw data.")
        return None

    combined = np.vstack(all_points)
    total_pts = len(combined)
    print(f"\nCollected {total_pts} points from {packet_count} packets ({parse_fail} parse failures)")
    return combined


def dump_packets(port, count=5, bind_ip="0.0.0.0"):
    """Dump raw packet hex for debugging the format."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(5.0)
    sock.bind((bind_ip, port))

    print(f"Dumping {count} packets from port {port}...\n")

    for i in range(count):
        try:
            data, addr = sock.recvfrom(65535)
            print(f"--- Packet {i+1}: {len(data)} bytes from {addr} ---")
            print(f"  Full hex ({min(len(data), 80)} bytes): {data[:80].hex()}")
            print(f"  Byte 0 (version):   {data[0]}")
            print(f"  Byte 1-2 (length):  {struct.unpack_from('<H', data, 1)[0]}")
            print(f"  Byte 4 (data_type): {data[4]}")

            # Try different header offsets
            for hdr in [18, 24, 28]:
                rem = (len(data) - hdr) % POINT_SIZE_HIGH
                npts = (len(data) - hdr) // POINT_SIZE_HIGH
                if rem == 0 and npts > 0:
                    payload = data[hdr:]
                    x, y, z = struct.unpack_from('<iii', payload, 0)
                    print(f"  Header={hdr}: {npts} points, first=({x/1000:.3f}, {y/1000:.3f}, {z/1000:.3f})m")

            print()
        except socket.timeout:
            print("  Timeout waiting for packet")
            break

    sock.close()


def visualize_matplotlib(points, max_points=50000):
    """Simple 3D scatter plot with matplotlib."""
    import matplotlib
    matplotlib.use('TkAgg')  # Use TkAgg for WSL2/WSLg
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    if len(points) > max_points:
        idx = np.random.choice(len(points), max_points, replace=False)
        points = points[idx]
        print(f"Downsampled to {max_points} points for display")

    # Filter out zero/invalid points
    mask = np.any(points != 0, axis=1)
    points = points[mask]

    # Color by height (z)
    z = points[:, 2]

    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    scatter = ax.scatter(points[:, 0], points[:, 1], points[:, 2],
                        c=z, cmap='viridis', s=0.5, alpha=0.6)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(f'Livox Mid-360 Point Cloud ({len(points)} points)')
    plt.colorbar(scatter, label='Height (m)', shrink=0.6)

    # Set equal aspect ratio
    max_range = np.max(np.abs(points)) * 1.1
    if max_range > 0:
        ax.set_xlim(-max_range, max_range)
        ax.set_ylim(-max_range, max_range)
        ax.set_zlim(-max_range, max_range)

    plt.tight_layout()
    print("Close the plot window to exit.")
    plt.show()


def visualize_open3d(points, max_points=500000):
    """Interactive 3D view with Open3D."""
    import open3d as o3d

    if len(points) > max_points:
        idx = np.random.choice(len(points), max_points, replace=False)
        points = points[idx]

    # Filter zeros
    mask = np.any(points != 0, axis=1)
    points = points[mask]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # Color by height
    z = points[:, 2]
    z_norm = (z - z.min()) / (z.max() - z.min() + 1e-8)
    colors = np.zeros((len(points), 3))
    colors[:, 0] = z_norm          # Red channel
    colors[:, 2] = 1.0 - z_norm    # Blue channel
    pcd.colors = o3d.utility.Vector3dVector(colors)

    print(f"Displaying {len(points)} points. Controls:")
    print("  Mouse drag: rotate | Scroll: zoom | Shift+drag: pan")
    print("  Q or Esc: close")
    o3d.visualization.draw_geometries([pcd],
                                       window_name="Livox Mid-360 Point Cloud",
                                       width=1280, height=720)


def main():
    parser = argparse.ArgumentParser(description="Livox LiDAR Point Cloud Viewer")
    parser.add_argument("--port", type=int, default=56301,
                        help="UDP port for point cloud data (default: 56301)")
    parser.add_argument("--duration", type=float, default=3.0,
                        help="Seconds to collect data (default: 3)")
    parser.add_argument("--bind", type=str, default="0.0.0.0",
                        help="IP to bind to (default: 0.0.0.0)")
    parser.add_argument("--viewer", type=str, default="auto",
                        choices=["matplotlib", "open3d", "auto", "save"],
                        help="Visualization method (default: auto)")
    parser.add_argument("--dump", action="store_true",
                        help="Dump raw packets for debugging instead of visualizing")
    parser.add_argument("--output", type=str, default="pointcloud.npy",
                        help="Output file when using --viewer save (default: pointcloud.npy)")
    args = parser.parse_args()

    print("=" * 50)
    print("  Livox LiDAR Point Cloud Viewer")
    print("=" * 50)
    print(f"  Port: {args.port}")
    print(f"  Bind: {args.bind}")
    print()

    if args.dump:
        dump_packets(args.port, count=5, bind_ip=args.bind)
        return

    points = collect_points(args.port, args.duration, bind_ip=args.bind)

    if points is None or len(points) == 0:
        print("\nNo points collected. Check that:")
        print("  1. Pi relay is running (python3 lidar_relay.py <your_ip>)")
        print("  2. LiDAR is powered on")
        print("  3. No other process is using port 56301")
        print("\nTry: python3 lidar_view.py --dump   (to see raw packets)")
        sys.exit(1)

    # Stats
    print(f"\nPoint cloud stats:")
    print(f"  X range: [{points[:,0].min():.2f}, {points[:,0].max():.2f}] m")
    print(f"  Y range: [{points[:,1].min():.2f}, {points[:,1].max():.2f}] m")
    print(f"  Z range: [{points[:,2].min():.2f}, {points[:,2].max():.2f}] m")

    if args.viewer == "save":
        np.save(args.output, points)
        print(f"\nSaved {len(points)} points to {args.output}")
        return

    if args.viewer == "auto":
        try:
            import open3d
            args.viewer = "open3d"
        except ImportError:
            args.viewer = "matplotlib"

    print(f"\nUsing {args.viewer} for visualization...")

    if args.viewer == "open3d":
        visualize_open3d(points)
    else:
        visualize_matplotlib(points)


if __name__ == "__main__":
    main()
