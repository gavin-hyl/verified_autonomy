#!/bin/bash
# Run this script ON THE GO1 Pi (ssh pi@192.168.12.1)
# Usage: bash setup_lidar_forwarding.sh <PC_WIFI_IP>
# Example: bash setup_lidar_forwarding.sh 192.168.12.223

PC_IP="${1:?Usage: $0 <PC_WIFI_IP e.g. 192.168.12.223>}"

echo "=== Go1 Pi: LiDAR UDP Port Forwarding Setup ==="
echo "PC WiFi IP: $PC_IP"
echo "Pi eth0 IP: 192.168.1.50 (LiDAR network)"
echo "Pi wlan1 IP: 192.168.12.1 (WiFi hotspot)"
echo "LiDAR IP: 192.168.1.127"
echo ""

# Enable IP forwarding
sudo sysctl -w net.ipv4.ip_forward=1

# Flush old NAT rules to avoid duplicates
sudo iptables -t nat -F

# NAT masquerade on eth0 (outgoing: PC → LiDAR network + robot control)
sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE

# NAT masquerade on wlan1 (forwarded LiDAR data → PC)
sudo iptables -t nat -A POSTROUTING -o wlan1 -j MASQUERADE

# Forward LiDAR UDP data ports arriving on eth0 (from LiDAR) to PC via wlan1
# These are the ports the LiDAR sends data TO (host ports from config)
echo "Setting up DNAT rules for LiDAR data ports..."

# Command response port
sudo iptables -t nat -A PREROUTING -i eth0 -p udp -s 192.168.1.127 --dport 56101 -j DNAT --to-destination ${PC_IP}:56101
# Push message port
sudo iptables -t nat -A PREROUTING -i eth0 -p udp -s 192.168.1.127 --dport 56201 -j DNAT --to-destination ${PC_IP}:56201
# Point cloud data port
sudo iptables -t nat -A PREROUTING -i eth0 -p udp -s 192.168.1.127 --dport 56301 -j DNAT --to-destination ${PC_IP}:56301
# IMU data port
sudo iptables -t nat -A PREROUTING -i eth0 -p udp -s 192.168.1.127 --dport 56401 -j DNAT --to-destination ${PC_IP}:56401
# Log data port
sudo iptables -t nat -A PREROUTING -i eth0 -p udp -s 192.168.1.127 --dport 56501 -j DNAT --to-destination ${PC_IP}:56501

# Also forward the detection/discovery port (56000)
sudo iptables -t nat -A PREROUTING -i eth0 -p udp -s 192.168.1.127 --dport 56000 -j DNAT --to-destination ${PC_IP}:56000

# Allow forwarding between interfaces
sudo iptables -A FORWARD -i eth0 -o wlan1 -j ACCEPT
sudo iptables -A FORWARD -i wlan1 -o eth0 -j ACCEPT

echo ""
echo "=== NAT rules active ==="
sudo iptables -t nat -L -v --line-numbers
echo ""
echo "=== Forwarding rules ==="
sudo iptables -L FORWARD -v --line-numbers
echo ""
echo "Done! LiDAR data will be forwarded from 192.168.1.127 → Pi → ${PC_IP}"
echo ""
echo "On your PC (Docker), set MID360_config.json:"
echo "  host_net_info IPs = 192.168.1.50  (Pi's eth0 — where LiDAR sends data)"
echo "  lidar_configs ip  = 192.168.1.127 (LiDAR address)"
