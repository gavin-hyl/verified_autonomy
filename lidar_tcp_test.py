#!/usr/bin/env python3
"""
LiDAR TCP Bridge Data Test
===========================
Run this INSIDE Docker (or WSL) to check if data arrives
from the TCP bridge at 127.0.0.1:9000.

This script does TWO things:
  1. Connects to the Windows TCP bridge (like wsl_udp_bridge.py does)
  2. Listens on the local UDP ports (like the Livox driver would)

This tells you whether data is flowing through the bridge WITHOUT
needing the Livox driver (which is stuck at "get free index").

Usage (inside Docker):
    python3 /workspace/lidar_tcp_test.py [WINDOWS_IP]

If no IP given, auto-detects via host.docker.internal or default gateway.
"""
import socket
import struct
import sys
import threading
import time

# ── Configuration ──
TCP_PORT = 9000
LISTEN_DURATION = 15  # seconds

# Ports the bridge sends data on
DATA_PORTS = {
    56101: "Cmd Response",
    56201: "Push Msg",
    56301: "Point Cloud",
    56401: "IMU Data",
    56501: "Log Data",
}

# Command ports the driver normally sends on
CMD_PORTS = {
    56000: "Discovery",
    56100: "Command",
}


def get_windows_ip():
    """Auto-detect Windows host IP."""
    # Docker: host.docker.internal
    try:
        ip = socket.gethostbyname("host.docker.internal")
        return ip
    except Exception:
        pass
    # WSL: default gateway
    try:
        with open("/proc/net/route") as f:
            for line in f:
                fields = line.strip().split()
                if fields[1] == "00000000":
                    gw_hex = fields[2]
                    gw = socket.inet_ntoa(bytes.fromhex(gw_hex)[::-1]
                                          if sys.byteorder == 'little'
                                          else bytes.fromhex(gw_hex))
                    return gw
    except Exception:
        pass
    return None


def test_tcp_connection(win_ip):
    """Test 1: Can we connect to the Windows TCP bridge?"""
    print(f"\n── Test 1: TCP connection to {win_ip}:{TCP_PORT} ──")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        sock.connect((win_ip, TCP_PORT))
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print(f"  ✓ Connected to TCP bridge at {win_ip}:{TCP_PORT}")
        return sock
    except ConnectionRefusedError:
        print(f"  ✗ Connection REFUSED — is udp_forward.py running on Windows?")
        return None
    except socket.timeout:
        print(f"  ✗ Connection TIMEOUT — firewall blocking port {TCP_PORT}?")
        return None
    except Exception as e:
        print(f"  ✗ Error: {e}")
        return None


def test_tcp_data(tcp_sock):
    """Test 2: Is data coming through the TCP tunnel?"""
    print(f"\n── Test 2: Data arriving via TCP tunnel ({LISTEN_DURATION}s) ──")

    stats = {}
    total = [0]
    first_data_time = [None]

    def receiver():
        buf = b""
        while True:
            try:
                data = tcp_sock.recv(262144)
                if not data:
                    print("  [TCP] Connection closed")
                    break
                buf += data

                while len(buf) >= 6:
                    port, plen = struct.unpack("!HI", buf[:6])
                    if len(buf) < 6 + plen:
                        break
                    payload = buf[6:6 + plen]
                    buf = buf[6 + plen:]

                    total[0] += 1
                    if first_data_time[0] is None:
                        first_data_time[0] = time.time()

                    name = DATA_PORTS.get(port, f"Unknown:{port}")
                    if port not in stats:
                        stats[port] = {"count": 0, "bytes": 0, "first_payload": payload}
                        print(f"  ✓ First packet on port {port} ({name}): {plen} bytes")
                    stats[port]["count"] += 1
                    stats[port]["bytes"] += plen

            except socket.timeout:
                continue
            except Exception as e:
                print(f"  [TCP ERR] {e}")
                break

    tcp_sock.settimeout(1.0)
    t = threading.Thread(target=receiver, daemon=True)
    t.start()

    # Wait and print periodic updates
    start = time.time()
    last_total = 0
    while time.time() - start < LISTEN_DURATION:
        time.sleep(2)
        if total[0] == 0:
            elapsed = time.time() - start
            print(f"  ⏳ No data yet... ({elapsed:.0f}s)")
        elif total[0] != last_total:
            last_total = total[0]
            elapsed = time.time() - start
            print(f"  📊 {total[0]} packets so far ({elapsed:.0f}s)")

    print()
    if total[0] == 0:
        print("  ✗ NO data received through TCP bridge!")
        print("    Possible causes:")
        print("    - Pi relay (lidar_relay.py) not running on Go1")
        print("    - Pi relay not forwarding to Windows IP")
        print("    - Windows udp_forward.py not receiving UDP from Pi")
        return False, stats
    else:
        print(f"  ✓ Received {total[0]} packets via TCP bridge")
        print()
        print(f"  {'Port':<8} {'Name':<16} {'Packets':>8} {'Bytes':>10}")
        print(f"  {'─'*8} {'─'*16} {'─'*8} {'─'*10}")
        for port in sorted(stats):
            name = DATA_PORTS.get(port, f"Unknown:{port}")
            s = stats[port]
            print(f"  {port:<8} {name:<16} {s['count']:>8} {s['bytes']:>10}")
        return True, stats


def test_udp_local():
    """Test 3: Check if local UDP ports are in use (by the Livox driver)."""
    print(f"\n── Test 3: Local UDP port availability ──")
    all_ports = list(DATA_PORTS.keys()) + list(CMD_PORTS.keys())
    for port in sorted(all_ports):
        name = DATA_PORTS.get(port, CMD_PORTS.get(port, "Unknown"))
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("0.0.0.0", port))
            sock.close()
            print(f"  Port {port} ({name}): FREE — nothing listening")
        except OSError:
            print(f"  Port {port} ({name}): IN USE — something is bound here")


def test_parse_point_cloud(stats):
    """Test 4: Try to parse point cloud data from captured packets."""
    if 56301 not in stats:
        print(f"\n── Test 4: Point cloud parsing ──")
        print("  ✗ No point cloud data received, skipping")
        return

    print(f"\n── Test 4: Point cloud data parsing ──")
    payload = stats[56301]["first_payload"]
    print(f"  Payload size: {len(payload)} bytes")
    print(f"  Header hex: {payload[:32].hex()}")

    POINT_SIZE = 14  # int32 x, int32 y, int32 z, uint8 ref, uint8 tag
    for hdr_size in [18, 24, 28, 20, 22, 26]:
        body = payload[hdr_size:]
        if len(body) >= POINT_SIZE and len(body) % POINT_SIZE == 0:
            n = len(body) // POINT_SIZE
            points = []
            for i in range(min(n, 5)):
                off = i * POINT_SIZE
                x, y, z = struct.unpack_from('<iii', body, off)
                ref = body[off + 12]
                points.append((x / 1000.0, y / 1000.0, z / 1000.0, ref))
            if points and any(abs(p[0]) < 200 and abs(p[1]) < 200 for p in points):
                print(f"  ✓ Parsed with header_size={hdr_size}: {n} points/packet")
                print(f"    Sample points (x, y, z, ref):")
                for p in points:
                    print(f"      ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}, ref={p[3]})")
                return
    print("  ✗ Could not parse point cloud data with any known header size")


def main():
    print("=" * 60)
    print("  LiDAR TCP Bridge Data Test")
    print("=" * 60)

    # Determine Windows IP
    win_ip = sys.argv[1] if len(sys.argv) > 1 else get_windows_ip()
    if not win_ip:
        print("\n  ✗ Cannot detect Windows host IP")
        print(f"  Usage: {sys.argv[0]} <WINDOWS_IP>")
        sys.exit(1)
    print(f"  Windows IP: {win_ip}")

    # Test 3: Check local port status first
    test_udp_local()

    # Test 1: TCP connection
    tcp_sock = test_tcp_connection(win_ip)
    if not tcp_sock:
        print("\n  Cannot proceed without TCP connection.")
        sys.exit(1)

    # Test 2: Data through TCP
    has_data, stats = test_tcp_data(tcp_sock)
    tcp_sock.close()

    # Test 4: Parse point cloud
    if has_data:
        test_parse_point_cloud(stats)

    # Summary
    print()
    print("=" * 60)
    print("  SUMMARY")
    print("=" * 60)
    if has_data:
        print("  ✓ Data IS flowing through the TCP bridge")
        print()
        print("  The Livox driver's 'get free index' issue is likely because")
        print("  the driver's COMMAND packets (discovery/handshake on port")
        print("  56000/56100) are NOT reaching the LiDAR through the reverse")
        print("  path. The data path (LiDAR → Pi → Windows → TCP → Docker)")
        print("  works, but the command path (Docker → TCP → Windows → Pi →")
        print("  LiDAR) may be broken.")
        print()
        print("  OPTIONS:")
        print("  1. Use this test script's raw data directly (no ROS driver)")
        print("  2. Fix the command path so the Livox driver can handshake")
        print("     - Inside Docker, the driver sends to 192.168.1.127")
        print("     - This needs to be routed back through the TCP tunnel")
        print("     - wsl_udp_bridge.py tries iptables DNAT for this")
        print("     - Run wsl_udp_bridge.py as ROOT inside Docker")
    else:
        print("  ✗ No data coming through the TCP bridge")
        print()
        print("  Check the full chain:")
        print("  1. Pi relay running? (lidar_relay.py on Go1 Pi)")
        print("  2. Windows bridge running? (py udp_forward.py)")
        print("  3. Pi relay forwarding to correct Windows IP?")
    print("=" * 60)


if __name__ == "__main__":
    main()
