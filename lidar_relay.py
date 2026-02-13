#!/usr/bin/env python3
"""
Livox LiDAR UDP Relay v3 - Run ON THE GO1 Pi
No dependencies beyond Python 3 standard library.

Solves the fundamental problem: the Livox driver in Docker/WSL2 cannot bind
to a 192.168.1.x IP (it doesn't have one), so host_net_info must be 0.0.0.0.
But the SDK embeds those IPs in the handshake protocol to tell the LiDAR
where to send data — 0.0.0.0 is meaningless to the LiDAR.

How it works:
  1. Docker config uses host_net_info IPs = "0.0.0.0" (so SDK bind() works)
  2. Docker config uses lidar_configs.ip = "192.168.1.127" (real LiDAR IP)
  3. SDK sends commands to LiDAR IP, routed via Pi (NAT masquerade)
  4. iptables DNAT on Pi intercepts these commands, redirects to local relay
  5. Relay REWRITES 0.0.0.0 → 192.168.1.50 in the Livox handshake protocol
  6. Forwards rewritten commands to the real LiDAR at 192.168.1.127
  7. LiDAR now sends data streams to 192.168.1.50 (the Pi)
  8. Relay catches data on Pi → forwards to PC over WiFi

MID360_config.json should have:
  host_net_info IPs = "0.0.0.0"        (bind locally in Docker — no bind failure)
  lidar_configs ip  = "127.0.0.1"     (caught by wsl_udp_bridge → TCP → Windows → Pi)

Usage: python3 lidar_relay.py <PC_WIFI_IP>
Example: python3 lidar_relay.py 192.168.12.223
"""
import socket
import struct
import sys
import threading
import os
import time

# Global list to store raw point cloud packets for saving
captured_packets = []
capture_lock = threading.Lock()
AUTOSAVE_INTERVAL = 10000  # save every N packets
last_save_count = 0


def save_captured_data(label="auto"):
    """Save captured point cloud packets to ~/lidar_capture.npz. Safe to call anytime."""
    global last_save_count
    with capture_lock:
        n_packets = len(captured_packets)
        if n_packets == 0 or n_packets == last_save_count:
            return
        raw_packets = list(captured_packets)

    POINT_SIZE = 14  # int32 x, int32 y, int32 z, uint8 ref, uint8 tag
    all_points = []

    for pkt in raw_packets:
        for hdr in [18, 24, 28, 20, 22, 26]:
            payload = pkt[hdr:]
            if len(payload) >= POINT_SIZE and len(payload) % POINT_SIZE == 0:
                n = len(payload) // POINT_SIZE
                for i in range(n):
                    off = i * POINT_SIZE
                    x, y, z = struct.unpack_from('<iii', payload, off)
                    if abs(x) < 200000 and abs(y) < 200000 and abs(z) < 200000:
                        all_points.append((x / 1000.0, y / 1000.0, z / 1000.0))
                break

    save_path = os.path.expanduser("~/lidar_capture.npz")
    try:
        import array
        pts = array.array('f')
        for p in all_points:
            pts.append(p[0])
            pts.append(p[1])
            pts.append(p[2])
        with open(save_path, 'wb') as f:
            f.write(struct.pack('<I', len(all_points)))
            f.write(pts.tobytes())
        last_save_count = n_packets
        print(f"  [{label}] Saved {len(all_points)} points ({n_packets} pkts) → {save_path}")
    except Exception as e:
        print(f"  [SAVE ERROR] {e}")

if len(sys.argv) < 2:
    print(f"Usage: {sys.argv[0]} <PC_WIFI_IP> [--port-offset N]")
    print(f"Example: {sys.argv[0]} 192.168.12.223")
    print(f"  With Windows UDP forwarder: {sys.argv[0]} 192.168.12.223 --port-offset 1000")
    sys.exit(1)

PC_IP = sys.argv[1]
PC_PORT_OFFSET = 0
if "--port-offset" in sys.argv:
    idx = sys.argv.index("--port-offset")
    PC_PORT_OFFSET = int(sys.argv[idx + 1])

LIDAR_IP = "192.168.1.127"
PI_ETH_IP = "192.168.1.50"
BUF_SIZE = 65535

# Livox SDK2 embeds host IP as 4 raw bytes in command payloads (HostIpInfoValue).
# When config has 0.0.0.0, the SDK sends {0,0,0,0} literally.
# We replace with Pi's IP so the LiDAR sends data back to us.
ZERO_IP_BYTES = b'\x00\x00\x00\x00'
PI_IP_BYTES = bytes(int(x) for x in PI_ETH_IP.split('.'))


def rewrite_host_ip(data):
    """
    Rewrite 0.0.0.0 → PI_ETH_IP in Livox SDK2 command packets.

    The Livox SDK2 protocol embeds the host IP as 4 raw bytes in
    HostIpInfoValue structs within command payloads. Each struct is:
      host_ip[4] + host_port(uint16_LE) + lidar_port(uint16_LE) = 8 bytes

    We scan for: 4 zero bytes followed by a uint16 LE port in range 50000-60000.
    This identifies Livox host IP fields without false positives.
    """
    result = bytearray(data)
    modified = False
    i = 0
    while i <= len(result) - 8:
        if result[i:i+4] == ZERO_IP_BYTES:
            host_port = struct.unpack_from('<H', result, i + 4)[0]
            if 50000 <= host_port <= 60000:  # Livox ports are in 56xxx range
                result[i:i+4] = PI_IP_BYTES
                modified = True
                i += 8  # skip past this HostIpInfoValue
                continue
        i += 1

    if modified:
        print(f"    [REWRITE] 0.0.0.0 → {PI_ETH_IP} in command ({len(data)} bytes)")
    return bytes(result)


def command_relay(name, listen_port, dest_ip, dest_port):
    """
    Relay commands from Docker/WSL2 → LiDAR, rewriting host IPs in handshake.
    Also relay responses back to the sender.

    Listens on 0.0.0.0 (any interface) because iptables DNAT redirects packets
    originally destined for the LiDAR to this local port.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", listen_port))
    except OSError as e:
        print(f"  [FAIL] {name}: cannot bind 0.0.0.0:{listen_port} - {e}")
        return

    print(f"  [OK] {name}: 0.0.0.0:{listen_port} → {dest_ip}:{dest_port} (DNAT + IP rewrite)")

    last_sender = None

    while True:
        try:
            data, addr = sock.recvfrom(BUF_SIZE)
            if addr[0] == dest_ip:
                # Response from LiDAR → relay back to Docker/WSL2
                if last_sender:
                    sock.sendto(data, last_sender)
            else:
                # Command from Docker/WSL2 → rewrite host IP → forward to LiDAR
                last_sender = addr
                rewritten = rewrite_host_ip(data)
                sock.sendto(rewritten, (dest_ip, dest_port))
        except Exception as e:
            print(f"  [ERR] {name}: {e}")


def lidar_to_pc_relay(name, listen_port, pc_port, save_packets=False):
    """
    LiDAR sends data to Pi on listen_port, we forward to PC on pc_port.
    If save_packets=True, also stores raw packets for later saving.
    """
    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        recv_sock.bind((PI_ETH_IP, listen_port))
    except OSError as e:
        print(f"  [FAIL] {name}: cannot bind {PI_ETH_IP}:{listen_port} - {e}")
        return

    suffix = " + SAVING" if save_packets else ""
    actual_pc_port = pc_port + PC_PORT_OFFSET
    print(f"  [OK] {name}: {PI_ETH_IP}:{listen_port} → {PC_IP}:{actual_pc_port}{suffix}")

    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    count = 0

    while True:
        try:
            data, addr = recv_sock.recvfrom(BUF_SIZE)
            send_sock.sendto(data, (PC_IP, actual_pc_port))
            count += 1

            if save_packets:
                with capture_lock:
                    captured_packets.append(data)
                if count % AUTOSAVE_INTERVAL == 0:
                    save_captured_data(f"autosave @{count}")

            if count == 1 or count % 1000 == 0:
                print(f"    [{name}] forwarded {count} packets ({len(data)} bytes)")
        except Exception as e:
            print(f"  [ERR] {name}: {e}")


def main():
    print("=" * 60)
    print("  Livox LiDAR UDP Relay v3 (DNAT + handshake IP rewrite)")
    print("=" * 60)
    print(f"  LiDAR:  {LIDAR_IP}")
    print(f"  Pi eth: {PI_ETH_IP}")
    print(f"  PC:     {PC_IP}")
    if PC_PORT_OFFSET:
        print(f"  Port offset: +{PC_PORT_OFFSET} (sending to 57xxx on PC)")
    print()

    # Enable IP forwarding
    os.system("sudo sysctl -w net.ipv4.ip_forward=1 > /dev/null 2>&1")
    # Ensure masquerade for robot control + general routing
    os.system("sudo iptables -t nat -C POSTROUTING -o eth0 -j MASQUERADE 2>/dev/null || "
              "sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE")

    # === iptables DNAT: intercept commands destined for LiDAR ===
    # The SDK sends commands to LIDAR_IP (192.168.1.127) on ports 56000/56100/56200.
    # These come in on wlan1 from the PC. We DNAT them to PI_ETH_IP so our local
    # relay (bound on 0.0.0.0) can intercept, rewrite the handshake, and forward
    # to the real LiDAR.
    print("Setting up iptables DNAT rules...")
    CMD_PORTS = [56000, 56100, 56200]
    for port in CMD_PORTS:
        # Remove old rule if exists, then add
        os.system(f"sudo iptables -t nat -D PREROUTING -i wlan1 -d {LIDAR_IP} "
                  f"-p udp --dport {port} -j DNAT --to-destination {PI_ETH_IP}:{port} 2>/dev/null")
        ret = os.system(f"sudo iptables -t nat -A PREROUTING -i wlan1 -d {LIDAR_IP} "
                        f"-p udp --dport {port} -j DNAT --to-destination {PI_ETH_IP}:{port}")
        if ret == 0:
            print(f"  [OK] DNAT: wlan1 → {LIDAR_IP}:{port} redirected to {PI_ETH_IP}:{port}")
        else:
            print(f"  [WARN] DNAT rule for port {port} failed (may need sudo)")

    print()

    threads = []

    print("Starting relays...")
    print()

    # === Commands: SDK → (DNAT) → Pi relay → LiDAR (with IP rewrite) ===
    # SDK sends to LIDAR_IP (192.168.1.127), iptables DNAT redirects to Pi,
    # relay intercepts, rewrites 0.0.0.0 → PI_ETH_IP in handshake, forwards to LiDAR

    # Discovery (port 56000)
    t = threading.Thread(target=command_relay, daemon=True,
        args=("Discovery", 56000, LIDAR_IP, 56000))
    threads.append(t)

    # Commands (port 56100)
    t = threading.Thread(target=command_relay, daemon=True,
        args=("Command", 56100, LIDAR_IP, 56100))
    threads.append(t)

    # Push messages (port 56200)
    t = threading.Thread(target=command_relay, daemon=True,
        args=("Push Msg Cmd", 56200, LIDAR_IP, 56200))
    threads.append(t)

    # === Data: LiDAR → Pi → PC ===
    # After rewritten handshake tells LiDAR "send data to 192.168.1.50:56x01",
    # the LiDAR streams to us. We forward to PC over WiFi.

    # Command response (56101)
    t = threading.Thread(target=lidar_to_pc_relay, daemon=True,
        args=("Cmd Response", 56101, 56101))
    threads.append(t)

    # Push messages (56201)
    t = threading.Thread(target=lidar_to_pc_relay, daemon=True,
        args=("Push Msg", 56201, 56201))
    threads.append(t)

    # Point cloud data (56301) — with packet saving
    t = threading.Thread(target=lidar_to_pc_relay, daemon=True,
        args=("Point Cloud", 56301, 56301, True))
    threads.append(t)

    # IMU data (56401)
    t = threading.Thread(target=lidar_to_pc_relay, daemon=True,
        args=("IMU Data", 56401, 56401))
    threads.append(t)

    # Log data (56501)
    t = threading.Thread(target=lidar_to_pc_relay, daemon=True,
        args=("Log Data", 56501, 56501))
    threads.append(t)

    for t in threads:
        t.start()

    print()
    print("=" * 60)
    print("  All relays running!")
    print("=" * 60)
    print()
    print("  MID360_config.json should have:")
    print(f'    host_net_info IPs = "0.0.0.0"        (bind locally, no failure)')
    print(f'    lidar_configs ip  = "127.0.0.1"     (caught by wsl_udp_bridge)')
    print()
    print(f"  iptables DNAT intercepts commands from PC → {LIDAR_IP}")
    print(f"  Relay rewrites 0.0.0.0 → {PI_ETH_IP} in handshake")
    print(f"  LiDAR sends data to {PI_ETH_IP} → relay forwards to {PC_IP}")
    print()
    print("  Press Ctrl+C to stop")

    try:
        for t in threads:
            t.join()
    except KeyboardInterrupt:
        print("\n")
        print("=" * 60)

        # Save captured point cloud packets
        save_captured_data("final")

        print()
        print("Cleaning up iptables DNAT rules...")
        for port in CMD_PORTS:
            os.system(f"sudo iptables -t nat -D PREROUTING -i wlan1 -d {LIDAR_IP} "
                      f"-p udp --dport {port} -j DNAT --to-destination {PI_ETH_IP}:{port} 2>/dev/null")
        print("Stopping relays...")
        print("=" * 60)
        sys.exit(0)


if __name__ == "__main__":
    main()
