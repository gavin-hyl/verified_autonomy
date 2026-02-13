#!/usr/bin/env python3
"""
Windows UDP↔TCP Bridge for Livox LiDAR
=======================================
Bridges Livox LiDAR UDP packets from the Go1 Pi relay into WSL2
via a TCP connection.

WHY TCP?  WSL2's NAT does NOT forward inbound UDP from Windows.
But WSL2 CAN initiate outbound TCP connections to Windows.
So the WSL-side bridge connects here via TCP, and we push UDP
packets through the TCP tunnel.

Run on WINDOWS:   py udp_forward.py
Run in WSL:        python3 ~/wsl_udp_bridge.py

Flow:
  Pi relay ──UDP──▶ Windows:57xxx ──TCP:9000──▶ WSL bridge
                                                  │ local UDP
                                                  ▼
                                            Livox driver :56xxx
"""
import socket
import struct
import sys
import threading
import time

TCP_PORT = 9000  # WSL bridge connects here

# Listen on offset ports (what Pi relay sends to)
WIN_OFFSET = 1000  # listen on 57xxx

PORTS = [
    (56101, "Cmd Response"),
    (56201, "Push Msg"),
    (56301, "Point Cloud"),
    (56401, "IMU Data"),
    (56501, "Log Data"),
]

# Command ports the driver sends (WSL → Pi)
CMD_PORTS = [
    (56000, "Discovery"),
    (56100, "Command"),
]

# ── TCP client management ──
tcp_clients = []          # list of (socket, lock)
tcp_clients_lock = threading.Lock()

# ── Outbound queue (WSL driver → Pi relay) ──
outbound_queue = []
outbound_lock = threading.Lock()
outbound_event = threading.Event()

PI_IP = "192.168.12.1"


def frame_packet(port, data):
    """Frame a UDP packet for TCP: port(2) + len(4) + payload"""
    return struct.pack("!HI", port, len(data)) + data


def udp_listener(driver_port, name):
    """Listen on 0.0.0.0:(driver_port+offset), push to all TCP clients."""
    win_port = driver_port + WIN_OFFSET
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", win_port))
    except OSError as e:
        print(f"  [FAIL] {name} port {win_port}: {e}")
        return

    print(f"  [OK] UDP {name}: 0.0.0.0:{win_port}")

    count = 0
    while True:
        try:
            data, addr = sock.recvfrom(65535)
            count += 1
            if count == 1:
                print(f"  [{name}] First UDP pkt from {addr[0]}:{addr[1]} ({len(data)} B)")
            elif count % 5000 == 0:
                print(f"  [{name}] {count} packets")

            frame = frame_packet(driver_port, data)
            with tcp_clients_lock:
                dead = []
                for i, (csock, clock) in enumerate(tcp_clients):
                    try:
                        with clock:
                            csock.sendall(frame)
                    except Exception:
                        dead.append(i)
                for i in reversed(dead):
                    tcp_clients.pop(i)
        except Exception as e:
            print(f"  [ERR] {name}: {e}")


def handle_tcp_client(csock, addr):
    """Receive framed packets from WSL (driver commands → forward as UDP to Pi)."""
    print(f"  [TCP] Client connected: {addr[0]}:{addr[1]}")
    buf = b""
    while True:
        try:
            data = csock.recv(65536)
            if not data:
                break
            buf += data
            while len(buf) >= 6:
                port, plen = struct.unpack("!HI", buf[:6])
                if len(buf) < 6 + plen:
                    break
                payload = buf[6:6 + plen]
                buf = buf[6 + plen:]
                with outbound_lock:
                    outbound_queue.append((port, payload))
                outbound_event.set()
        except Exception:
            break
    print(f"  [TCP] Client disconnected: {addr[0]}:{addr[1]}")
    with tcp_clients_lock:
        tcp_clients[:] = [(s, l) for s, l in tcp_clients if s is not csock]
    csock.close()


def tcp_server():
    """Accept TCP connections from WSL bridge."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", TCP_PORT))
    srv.listen(2)
    print(f"  [OK] TCP server on 0.0.0.0:{TCP_PORT}  (waiting for WSL bridge)")

    while True:
        csock, addr = srv.accept()
        csock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        lock = threading.Lock()
        with tcp_clients_lock:
            tcp_clients.append((csock, lock))
        threading.Thread(target=handle_tcp_client, args=(csock, addr), daemon=True).start()


def outbound_udp_sender():
    """Forward queued command packets from WSL driver → Pi relay as UDP."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    count = 0
    while True:
        outbound_event.wait()
        outbound_event.clear()
        with outbound_lock:
            batch = outbound_queue[:]
            outbound_queue.clear()
        for port, data in batch:
            try:
                # The Pi relay intercepts on the LiDAR ports via DNAT,
                # but command packets go to the Pi relay address.
                sock.sendto(data, (PI_IP, port))
                count += 1
                if count == 1:
                    print(f"  [CMD] First outbound pkt → {PI_IP}:{port} ({len(data)} B)")
                elif count % 100 == 0:
                    print(f"  [CMD] {count} outbound packets sent")
            except Exception as e:
                print(f"  [ERR] Outbound → {PI_IP}:{port}: {e}")


def main():
    print("=" * 60)
    print("  Windows UDP ↔ TCP Bridge for Livox LiDAR")
    print("=" * 60)
    print()

    # UDP listeners (Pi relay → Windows)
    print("  Inbound UDP listeners (from Pi relay):")
    for driver_port, name in PORTS:
        threading.Thread(target=udp_listener, args=(driver_port, name), daemon=True).start()
    print()

    # Outbound UDP sender (WSL driver → Pi relay)
    threading.Thread(target=outbound_udp_sender, daemon=True).start()
    for _, name in CMD_PORTS:
        print(f"  [OK] Cmd {name}: via TCP tunnel → UDP to {PI_IP}")
    print()

    # TCP server
    threading.Thread(target=tcp_server, daemon=True).start()
    print()

    print("  ─── NEXT STEPS ───")
    print("  1. In WSL:  python3 ~/wsl_udp_bridge.py")
    print("  2. Start the Livox driver in Docker")
    print()
    print("  Press Ctrl+C to stop.")
    print()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping.")


if __name__ == "__main__":
    main()
