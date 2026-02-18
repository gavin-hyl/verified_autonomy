#!/bin/bash
# Livox LiDAR UDP Relay - Run ON THE GO1 Pi
# Usage: bash lidar_relay.sh <PC_WIFI_IP>
# Example: bash lidar_relay.sh 192.168.12.223
#
# This script runs socat UDP relays on the Pi to forward
# LiDAR data from 192.168.1.127 to your PC over WiFi.

PC_IP="${1:?Usage: $0 <PC_WIFI_IP e.g. 192.168.12.223>}"
LIDAR_IP="192.168.1.127"
PI_ETH="192.168.1.50"

echo "=== Livox LiDAR UDP Relay ==="
echo "LiDAR:  $LIDAR_IP"
echo "Pi eth: $PI_ETH"  
echo "PC:     $PC_IP"
echo ""

# Kill any existing socat relays
pkill -f "socat.*5610\|socat.*5620\|socat.*5630\|socat.*5640\|socat.*5650\|socat.*56000" 2>/dev/null
sleep 1

# Check if socat is installed
if ! command -v socat &>/dev/null; then
    echo "ERROR: socat not installed. Run: sudo apt install socat"
    exit 1
fi

# Enable IP forwarding
sudo sysctl -w net.ipv4.ip_forward=1 >/dev/null

# NAT masquerade for robot control (still needed)
sudo iptables -t nat -C POSTROUTING -o eth0 -j MASQUERADE 2>/dev/null || \
    sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE

echo "Starting UDP relays..."

# === Discovery port 56000 (bidirectional) ===
# PC sends discovery → relay to LiDAR, LiDAR responds → relay to PC
socat UDP4-LISTEN:56000,bind=$PI_ETH,reuseaddr,fork UDP4:$LIDAR_IP:56000 &
echo "  [OK] Discovery relay :56000 → $LIDAR_IP:56000"

# === Command channel ===
# PC sends commands to LiDAR:56100 (via relay on Pi)
socat UDP4-LISTEN:56100,bind=$PI_ETH,reuseaddr,fork UDP4:$LIDAR_IP:56100 &
echo "  [OK] Command relay :56100 → $LIDAR_IP:56100"

# === Host receive ports (LiDAR → Pi → PC) ===
# The LiDAR sends data TO the Pi's IP on these ports.
# We relay them to the PC.

# Command response: LiDAR → Pi:56101 → PC:56101
socat UDP4-LISTEN:56101,bind=$PI_ETH,reuseaddr,fork UDP4:$PC_IP:56101 &
echo "  [OK] Cmd response relay :56101 → $PC_IP:56101"

# Push messages: LiDAR → Pi:56201 → PC:56201
socat UDP4-LISTEN:56201,bind=$PI_ETH,reuseaddr,fork UDP4:$PC_IP:56201 &
echo "  [OK] Push msg relay :56201 → $PC_IP:56201"

# Point cloud data: LiDAR → Pi:56301 → PC:56301
socat UDP4-LISTEN:56301,bind=$PI_ETH,reuseaddr,fork UDP4:$PC_IP:56301 &
echo "  [OK] Point cloud relay :56301 → $PC_IP:56301"

# IMU data: LiDAR → Pi:56401 → PC:56401
socat UDP4-LISTEN:56401,bind=$PI_ETH,reuseaddr,fork UDP4:$PC_IP:56401 &
echo "  [OK] IMU data relay :56401 → $PC_IP:56401"

# Log data: LiDAR → Pi:56501 → PC:56501
socat UDP4-LISTEN:56501,bind=$PI_ETH,reuseaddr,fork UDP4:$PC_IP:56501 &
echo "  [OK] Log data relay :56501 → $PC_IP:56501"

echo ""
echo "=== All relays running ==="
echo "Relay PIDs:"
pgrep -a socat | grep -E "5610|5620|5630|5640|5650|56000"
echo ""
echo "To stop: pkill -f socat"
echo ""
echo "Now on your PC, in the MID360_config.json set:"
echo "  host_net_info IPs → 0.0.0.0 (bind all)"
echo "  lidar_configs ip  → $PI_ETH (connect via Pi relay)"
echo ""
echo "Press Ctrl+C to stop all relays"
wait
