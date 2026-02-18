#!/usr/bin/env python3
"""
WSL UDP Bridge — connects to the Windows TCP bridge and re-emits
LiDAR data as local UDP for the Livox driver.

Run INSIDE WSL (or Docker) — requires sudo for one iptables rule:
    sudo python3 ~/wsl_udp_bridge.py

Also forwards driver commands (local UDP) back through the TCP
tunnel to the Windows side, which sends them as UDP to the Pi relay.

Key insight: MID360_config.json has lidar_configs.ip = "127.0.0.1",
so the SDK sends commands to 127.0.0.1:56000/56100/56200.

Port 56000 (discovery) is special: the SDK ALSO binds 0.0.0.0:56000
to receive responses, so we can't bind it directly. We use one
iptables rule to redirect 127.0.0.1:56000 → 127.0.0.1:61000.

Ports 56100/56200 are fine — the SDK uses 56101/56201 for receiving,
so we bind 56100/56200 directly with no conflict.

Flow (inbound data):
    Windows TCP:9000 ──TCP──▶ this script ──UDP──▶ 127.0.0.1:56xxx
                                                       ▲
                                                  Livox driver

Flow (outbound commands):
    Livox driver ──UDP 127.0.0.1:56000/56100──▶ this script ──TCP──▶ Windows ──UDP──▶ Pi relay
"""
import os
import socket
import struct
import sys
import threading
import time

# Windows host IP — auto-detected from WSL's default gateway
def get_windows_ip():
    """Get the Windows host IP from WSL's default gateway."""
    try:
        with open("/proc/net/route") as f:
            for line in f:
                fields = line.strip().split()
                if fields[1] == "00000000":  # default route
                    # Gateway is in hex, little-endian
                    gw_hex = fields[2]
                    gw = socket.inet_ntoa(bytes.fromhex(gw_hex)[::-1] if sys.byteorder == 'little'
                                          else bytes.fromhex(gw_hex))
                    return gw
    except Exception:
        pass
    # Fallback: try resolving host.docker.internal (works in Docker)
    try:
        return socket.gethostbyname("host.docker.internal")
    except Exception:
        pass
    return None


TCP_PORT = 9000

# Livox data ports (Windows → WSL, re-emit locally)
DATA_PORTS = [56101, 56201, 56301, 56401, 56501]

# Command ports to intercept (driver sends to 127.0.0.1, we catch and tunnel)
# Port 56000: SDK binds 0.0.0.0:56000 for discovery responses, so we CANNOT
#             bind it directly. Use iptables to redirect → 61000.
# Port 56100: SDK binds 56101 (not 56100), so we CAN bind 56100 directly.
# Port 56200: SDK binds 56201 (not 56200), so we CAN bind 56200 directly.
DIRECT_CMD_PORTS = [
    (56100, "Command"),
    (56200, "Push Msg"),
]
REDIRECTED_CMD_PORTS = [
    (56000, 61000, "Discovery"),  # (original_port, listen_port, name)
]

# Outbound UDP sockets keyed by port
udp_out_socks = {}


def get_udp_sock(port):
    """Get or create a UDP socket for sending to localhost:port."""
    if port not in udp_out_socks:
        udp_out_socks[port] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return udp_out_socks[port]


def tcp_receiver(tcp_sock, stats):
    """Receive framed packets from Windows, re-emit as local UDP."""
    buf = b""
    while True:
        try:
            data = tcp_sock.recv(262144)
            if not data:
                print("  [TCP] Connection closed by Windows side")
                break
            buf += data

            while len(buf) >= 6:
                port, plen = struct.unpack("!HI", buf[:6])
                if len(buf) < 6 + plen:
                    break
                payload = buf[6:6 + plen]
                buf = buf[6 + plen:]

                # Re-emit as local UDP
                sock = get_udp_sock(port)
                sock.sendto(payload, ("127.0.0.1", port))

                stats["total"] += 1
                key = f"port_{port}"
                if key not in stats:
                    stats[key] = 0
                    print(f"  [DATA] First packet for port {port} ({plen} bytes)")
                stats[key] += 1

                if stats["total"] % 5000 == 0:
                    print(f"  [DATA] {stats['total']} total packets re-emitted locally")

        except ConnectionResetError:
            print("  [TCP] Connection reset")
            break
        except Exception as e:
            print(f"  [TCP] Error: {e}")
            break


def cmd_interceptor(tcp_sock, listen_port, tunnel_port, name, tcp_lock):
    """
    Listen on 127.0.0.1:<listen_port> for driver commands, forward via TCP tunnel.

    For ports where the SDK doesn't bind (56100, 56200), listen_port == tunnel_port
    and we bind directly. For port 56000 (discovery), the SDK binds 0.0.0.0:56000,
    so iptables redirects 127.0.0.1:56000 → 127.0.0.1:61000. We listen on 61000
    but frame the packet as port 56000 for the tunnel.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", listen_port))
    except OSError as e:
        print(f"  [FAIL] {name}: cannot bind 0.0.0.0:{listen_port} - {e}")
        return

    if listen_port != tunnel_port:
        print(f"  [CMD] {name}: listening on :{listen_port} (iptables redirect from :{tunnel_port}) → TCP tunnel")
    else:
        print(f"  [CMD] {name}: listening on 127.0.0.1:{listen_port} → TCP tunnel")

    count = 0
    while True:
        try:
            data, addr = sock.recvfrom(65535)
            # Always frame with the original port so the Pi relay gets it on the right port
            frame = struct.pack("!HI", tunnel_port, len(data)) + data
            with tcp_lock:
                tcp_sock.sendall(frame)
            count += 1
            if count == 1:
                print(f"  [CMD] First {name} packet ({len(data)} B) from {addr}")
            elif count % 100 == 0:
                print(f"  [CMD] {count} {name} packets forwarded")
        except Exception as e:
            print(f"  [CMD ERR] {name}: {e}")
            break


def main():
    # Detect Windows host IP
    win_ip = get_windows_ip()
    if not win_ip:
        # Try common WSL2 gateway patterns
        for candidate in ["172.17.0.1", "10.255.255.254"]:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(2)
                s.connect((candidate, TCP_PORT))
                s.close()
                win_ip = candidate
                break
            except Exception:
                continue

    if len(sys.argv) > 1:
        win_ip = sys.argv[1]

    if not win_ip:
        print("ERROR: Cannot detect Windows host IP.")
        print(f"Usage: {sys.argv[0]} [WINDOWS_IP]")
        print("  Try your default gateway: ip route | grep default")
        sys.exit(1)

    print("=" * 60)
    print("  WSL UDP Bridge (TCP tunnel from Windows)")
    print("=" * 60)
    print(f"  Windows host: {win_ip}:{TCP_PORT}")
    print()

    # Connect to Windows TCP bridge
    while True:
        try:
            tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            tcp_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            tcp_sock.connect((win_ip, TCP_PORT))
            print(f"  [OK] Connected to Windows TCP bridge at {win_ip}:{TCP_PORT}")
            break
        except ConnectionRefusedError:
            print(f"  [WAIT] Cannot connect to {win_ip}:{TCP_PORT} — is udp_forward.py running on Windows?")
            time.sleep(3)
        except Exception as e:
            print(f"  [ERR] {e}")
            time.sleep(3)

    tcp_lock = threading.Lock()
    stats = {"total": 0}

    # === Set up command interception ===
    # Port 56000 (discovery): SDK binds 0.0.0.0:56000, so we can't bind it.
    # Use iptables to redirect outgoing packets to a different local port.
    setup_discovery_redirect()

    # Start redirected command interceptors (need iptables)
    for orig_port, listen_port, cmd_name in REDIRECTED_CMD_PORTS:
        t = threading.Thread(
            target=cmd_interceptor,
            args=(tcp_sock, listen_port, orig_port, cmd_name, tcp_lock),
            daemon=True,
        )
        t.start()

    # Start direct command interceptors (no iptables needed)
    for cmd_port, cmd_name in DIRECT_CMD_PORTS:
        t = threading.Thread(
            target=cmd_interceptor,
            args=(tcp_sock, cmd_port, cmd_port, cmd_name, tcp_lock),
            daemon=True,
        )
        t.start()

    print()
    print("  Listening for UDP packets via TCP tunnel...")
    print("  Packets will be re-emitted to 127.0.0.1:<port>")
    print("  Driver commands → TCP tunnel → Pi relay")
    print()
    print("  Now start the Livox driver:")
    print("    ros2 launch livox_ros_driver2 msg_MID360_launch.py")
    print()

    # Start TCP receiver (blocks)
    tcp_receiver(tcp_sock, stats)

    print("\n  Connection lost. Cleaning up...")
    cleanup_discovery_redirect()


def setup_discovery_redirect():
    """
    Set up iptables to redirect discovery packets.

    The SDK sends to 127.0.0.1:56000, but it also binds 0.0.0.0:56000
    for receiving responses. Without this redirect, the SDK receives
    its own discovery packet (loopback) and our bridge never sees it.

    This redirects outbound 127.0.0.1:56000 → 127.0.0.1:61000 so
    our bridge can catch it on port 61000.
    """
    if os.geteuid() != 0:
        print("  [WARN] Not running as root — cannot set up discovery redirect.")
        print("         Run with: sudo python3 wsl_udp_bridge.py")
        print("         (Only needed for one iptables rule on port 56000)")
        return

    # Clean up any old rule first
    os.system(
        "iptables -t nat -D OUTPUT -d 127.0.0.1 -p udp --dport 56000 "
        "-j DNAT --to-destination 127.0.0.1:61000 2>/dev/null"
    )
    ret = os.system(
        "iptables -t nat -A OUTPUT -d 127.0.0.1 -p udp --dport 56000 "
        "-j DNAT --to-destination 127.0.0.1:61000"
    )
    if ret == 0:
        print("  [OK] iptables: 127.0.0.1:56000 → 127.0.0.1:61000 (discovery redirect)")
    else:
        print("  [WARN] Failed to add iptables rule for discovery redirect")


def cleanup_discovery_redirect():
    """Remove the iptables redirect rule."""
    os.system(
        "iptables -t nat -D OUTPUT -d 127.0.0.1 -p udp --dport 56000 "
        "-j DNAT --to-destination 127.0.0.1:61000 2>/dev/null"
    )
    print("  [OK] Cleaned up iptables discovery redirect")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n  Interrupted. Cleaning up...")
        cleanup_discovery_redirect()
