# Go1 WiFi Hotspot Control Guide (Windows + WSL)

This guide covers controlling the Unitree Go1 robot from a Windows PC via the robot's WiFi hotspot, using WSL2 (Ubuntu 24.04) and ROS 2 Jazzy.

## Prerequisites

- Windows 10/11 with WSL2 installed
- Ubuntu 24.04 in WSL (`wsl --install Ubuntu-24.04`)
- ROS 2 Jazzy installed in WSL
- Go1 robot powered on

## Network Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    WINDOWS PC                                │
│  ┌───────────────────────────────────────────────────────┐  │
│  │                 WSL2 (Ubuntu 24.04)                    │  │
│  │                                                        │  │
│  │  ┌─────────────┐     ┌─────────────┐                  │  │
│  │  │ go1_cmd.py  │────▶│  ros2_udp   │                  │  │
│  │  │ /high_cmd   │     │   bridge    │                  │  │
│  │  └─────────────┘     └──────┬──────┘                  │  │
│  │                              │ UDP                     │  │
│  └──────────────────────────────│─────────────────────────┘  │
│                                 │                            │
│  WiFi (192.168.12.x) ───────────┘                            │
└─────────────────────────────────────────────────────────────┘
              │
              │ Go1 Hotspot
              ▼
    ┌─────────────────┐         ┌─────────────────┐
    │   Go1 Pi        │  eth0   │  Go1 Control    │
    │ 192.168.12.1    │────────▶│  192.168.123.161│
    │ (IP forwarding) │         │                 │
    └─────────────────┘         └─────────────────┘
```

## One-Time Setup (with internet)

### 1. Install dependencies in WSL

```bash
# Open Ubuntu-24.04 terminal in VS Code
sudo apt-get update
sudo apt-get install -y libboost-all-dev liblcm-dev
```

### 2. Copy and build workspace

```bash
# Copy repo to WSL native filesystem (faster builds)
cp -r /mnt/c/Users/willi/.vscode/verified_autonomy ~/verified_autonomy

# Build
cd ~/verified_autonomy/ros2_unitree_ws
source /opt/ros/jazzy/setup.bash
rm -rf build install log
colcon build --symlink-install
```

## Quick Start (after connecting to Go1 hotspot)

### Step 1: Connect to Go1 WiFi hotspot
- SSID: `Unitree_GoXXXX` (check your robot)
- Your PC will get IP like `192.168.12.x`

### Step 2: Enable IP forwarding on Go1

Open a WSL terminal and run:

```bash
ssh pi@192.168.12.1
# Password: 123
```

Then on the Go1 Pi:

```bash
sudo sysctl -w net.ipv4.ip_forward=1
sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
exit
```

### Step 3: Test connection

```bash
ping -c 2 192.168.123.161
```

You should see replies with TTL=63 (indicating one hop through the Pi):
```
64 bytes from 192.168.123.161: icmp_seq=1 ttl=63 time=2.00 ms
64 bytes from 192.168.123.161: icmp_seq=2 ttl=63 time=2.05 ms
```

> **Note**: The NAT masquerade on the Pi handles routing automatically. You do NOT need to add an explicit route on WSL. If you tried `sudo ip route add 192.168.123.0/24 via 192.168.12.1` and got "Nexthop has invalid gateway", that's okay - the masquerade takes care of it.

### Step 4: Run the UDP bridge

```bash
cd ~/verified_autonomy/ros2_unitree_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=~/verified_autonomy/ros2_unitree_ws/src/unitree_legged_sdk/lib:$LD_LIBRARY_PATH
ros2 run unitree_legged_real ros2_udp HIGHLEVEL
```

Or use the helper script:

```bash
~/verified_autonomy/ros2_unitree_ws/scripts/go1_highlevel_bridge.sh
```

You should see:
```
UDP Initialized. socketfd: 3   Port: 8080
UDP Initialized. socketfd: 4   Port: 8090
high level runing!
```

### Step 5: Send commands (new WSL terminal)

Open another Ubuntu-24.04 terminal in VS Code:

```bash
source /opt/ros/jazzy/setup.bash
source ~/verified_autonomy/ros2_unitree_ws/install/setup.bash

# Stand up
python3 ~/verified_autonomy/ros2_unitree_ws/scripts/go1_cmd.py stand_up

# Walk forward (vx=0.3 m/s)
python3 ~/verified_autonomy/ros2_unitree_ws/scripts/go1_cmd.py walk 0.3 0 0

# Turn in place (yaw=0.5 rad/s)
python3 ~/verified_autonomy/ros2_unitree_ws/scripts/go1_cmd.py walk 0 0 0.5

# Stand down
python3 ~/verified_autonomy/ros2_unitree_ws/scripts/go1_cmd.py stand_down

# Interactive keyboard control
python3 ~/verified_autonomy/ros2_unitree_ws/scripts/go1_control.py
```

## Available Commands

| Command | Description |
|---------|-------------|
| `stand_up` | Robot stands up |
| `stand_down` | Robot lies down |
| `walk <vx> <vy> <yaw>` | Walk with velocities (m/s, m/s, rad/s) |
| `idle` / `stop` | Stop moving |

## ROS 2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/high_cmd` | `ros2_unitree_legged_msgs/HighCmd` | Send commands to robot |
| `/high_state` | `ros2_unitree_legged_msgs/HighState` | Robot state feedback |

## Troubleshooting

### Cannot SSH to 192.168.12.1
- Make sure you're connected to Go1's WiFi hotspot
- Check if Go1 is powered on

### Cannot ping 192.168.123.161
- Re-run Step 2 (IP forwarding on Go1)
- Re-run Step 3 (add route in WSL)
- Check: `ip route | grep 192.168.123`

### ros2_udp fails to start
- Make sure no other instance is running
- Check the bridge is built: `ros2 pkg list | grep unitree`

### Commands don't work
- Ensure ros2_udp bridge is running in another terminal
- Check robot is standing (try `stand_up` first)

## Network Reference

| Device | IP Address |
|--------|------------|
| Go1 Main Board (Pi) | 192.168.12.1 |
| Go1 Control Board | 192.168.123.161 |
| Your PC (on hotspot) | 192.168.12.x (DHCP) |

## Files Modified for WSL Build

The following files were modified to fix build issues:

1. `ros2_unitree_ws/src/unitree_ros2_to_real/CMakeLists.txt`
   - Changed SDK path from `unitree_legged_sdk-master` to `../unitree_legged_sdk`

2. `ros2_unitree_ws/src/unitree_ros2_to_real/include/convert.h`
   - Fixed typo: `position` → `postion` (SDK has typo)

3. `ros2_unitree_ws/src/ros2_unitree_legged_msgs` 
   - Symlinked from `unitree_ros2_to_real/ros2_unitree_legged_msgs`

---

## Full Autonomy Stack (Docker)

For SLAM, navigation, and the complete autonomy stack, use the Docker container:

### Start the Docker container

```bash
cd ~/verified_autonomy/docker
sudo service docker start
./shell.sh
```

### Build inside container (first time)

```bash
source /opt/ros/jazzy/setup.bash
source /unitree_ws/install/setup.bash
cd /workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-skip livox_ros_driver2 arise_slam_mid360 arise_slam_mid360_msgs
```

### Run simulation

```bash
./system_simulation.sh
```

### Note on or-tools symlinks

If `tare_planner` fails with linker errors, fix the symlinks:

```bash
cd /workspace/src/exploration_planner/tare_planner/or-tools/lib
rm -f libortools.so libortools.so.9
ln -sf libortools.so.9.8.3296 libortools.so.9
ln -sf libortools.so.9 libortools.so
```

---

## Livox Mid-360 LiDAR Access

The Go1 has a Livox Mid-360 LiDAR connected to its internal network. You can access the LiDAR while controlling the robot over WiFi.

### LiDAR Network Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                       GO1 INTERNAL NETWORK                       │
│                                                                  │
│  ┌─────────────────┐     ┌─────────────────┐                    │
│  │  Livox Mid-360  │     │  Go1 Control    │                    │
│  │  192.168.1.131  │     │ 192.168.123.161 │                    │
│  └────────┬────────┘     └────────┬────────┘                    │
│           │                       │                              │
│           └───────────┬───────────┘                              │
│                       │                                          │
│              ┌────────┴────────┐                                 │
│              │   Go1 Pi (NAT)  │                                 │
│              │ eth0: 192.168.1.50  (LiDAR + control)             │
│              │ wlan1: 192.168.12.1 (WiFi hotspot)                │
│              └────────┬────────┘                                 │
└───────────────────────│─────────────────────────────────────────┘
                        │ WiFi Hotspot (wlan1)
                        ▼
              ┌─────────────────┐
              │   Your PC       │
              │  192.168.12.x   │
              └─────────────────┘
```

### Find your PC's IP on Go1 hotspot

After connecting to the Go1 WiFi hotspot, find your assigned IP:

```bash
# In WSL
ip addr | grep 192.168.12

# Or in Windows PowerShell
ipconfig | findstr 192.168.12
```

Your PC will be assigned an IP like `192.168.12.x` (e.g., `192.168.12.223`). Note this IP — you'll need it for the LiDAR driver config.

### One-Time Setup (with internet, before going to the robot)

Build the Livox SDK and LiDAR driver package. This must be done **before** disconnecting from the internet, since you need the autonomy stack Docker container.

```bash
cd ~/verified_autonomy/docker
sudo service docker start
./shell.sh
```

Inside the Docker container, first build and install the **Livox SDK 2** (required by the driver):

```bash
cd /workspace/src/utilities/livox_ros_driver2/Livox-SDK2
mkdir -p build && cd build
cmake .. && make -j$(nproc)
sudo make install
```

This installs the shared library to `/usr/local/lib` and headers to `/usr/local/include`.

Then build the ROS 2 workspace **with** `livox_ros_driver2` (do NOT skip it):

```bash
source /opt/ros/jazzy/setup.bash
cd /workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-skip arise_slam_mid360 arise_slam_mid360_msgs
```

> **Note**: Unlike the simulation build, this only skips the SLAM packages, **not** `livox_ros_driver2`.

### Quick Start (LiDAR + Robot Control)

> **Prerequisites**: Complete the Quick Start steps 1–3 above first (connect to Go1 hotspot, enable IP forwarding, test connection to 192.168.123.161). The steps below add LiDAR access on top of robot control.

#### Step 1: Enable routing to LiDAR network

SSH into the Go1 Pi:

```bash
ssh pi@192.168.12.1
# Password: 123
```

On the Go1 Pi, run everything needed for **both** robot control and LiDAR routing:

```bash
# Enable IP forwarding
sudo sysctl -w net.ipv4.ip_forward=1

# NAT masquerade on eth0 — covers BOTH the robot control (192.168.123.x)
# and LiDAR (192.168.1.x) networks since eth0 is on 192.168.1.50/24
sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE

exit
```

> **Note**: The Pi's eth0 is `192.168.1.50` — the same subnet as the LiDAR (`192.168.1.127`). A single masquerade rule on eth0 handles both networks.

#### Step 2: Test connectivity

From your WSL terminal, verify you can reach **both** networks:

```bash
# Test robot control board
ping -c 2 192.168.123.161

# Test LiDAR
ping -c 2 192.168.1.127
```

If the LiDAR ping fails but the robot control works:

```bash
# SSH back to Pi and check if LiDAR is reachable from the Pi itself
ssh pi@192.168.12.1
ping -c 2 192.168.1.127

# If it works from Pi, check iptables rules:
sudo iptables -L -t nat -v
```

#### Step 3: Configure LiDAR driver

Edit the LiDAR config:

```bash
# In Docker container or WSL
nano ~/verified_autonomy/vector_navigation_stack/src/utilities/livox_ros_driver2/config/MID360_config.json
```

> **Important**: There are two conflicting requirements for `host_net_info`:
> - The Livox SDK calls `bind()` on these IPs locally — must be `0.0.0.0` (Docker doesn't have `192.168.1.x`)
> - The SDK embeds these IPs in the handshake protocol to tell the LiDAR where to send data — `0.0.0.0` is meaningless to the LiDAR
>
> The **Pi relay** (`lidar_relay.py`) solves this: it intercepts SDK commands, rewrites `0.0.0.0` → `192.168.1.50` in the handshake payload, then forwards to the real LiDAR. The LiDAR then sends data to the Pi, and the relay forwards it to your PC.
>
> For `lidar_configs.ip`, we use `127.0.0.1` (NOT the real LiDAR IP `192.168.1.127`) because:
> - The real LiDAR IP is unreachable from Docker/WSL2
> - The WSL UDP bridge (`wsl_udp_bridge.py`) binds `127.0.0.1:56000/56100` to catch SDK commands
> - Commands are tunneled: WSL bridge → TCP → Windows → UDP → Pi relay → LiDAR
> - No iptables or root required in WSL/Docker

Set `host_net_info` IPs to `0.0.0.0` (so the SDK can `bind()` inside Docker) and `lidar_configs.ip` to `127.0.0.1` (caught by the WSL UDP bridge):

```json
{
  "lidar_summary_info" : {
    "lidar_type": 8
  },
  "MID360": {
    "lidar_net_info" : {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info" : {
      "cmd_data_ip" : "0.0.0.0",
      "cmd_data_port": 56101,
      "push_msg_ip": "0.0.0.0",
      "push_msg_port": 56201,
      "point_data_ip": "0.0.0.0",
      "point_data_port": 56301,
      "imu_data_ip" : "0.0.0.0",
      "imu_data_port": 56401,
      "log_data_ip" : "",
      "log_data_port": 56501
    }
  },
  "lidar_configs" : [
    {
      "ip" : "127.0.0.1",
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```

> **Why `127.0.0.1`** for `lidar_configs.ip`? The SDK sends discovery/command packets to this address on ports 56000/56100. The WSL UDP bridge (`wsl_udp_bridge.py`) binds those ports on `127.0.0.1`, catches the packets, and tunnels them through TCP to Windows, then UDP to the Pi relay. The Pi relay rewrites `0.0.0.0` → `192.168.1.50` in the handshake and forwards to the real LiDAR. No iptables or root privileges needed in WSL/Docker.

#### Step 4: Run LiDAR driver with robot control

**Terminal 1: Pi relay (SSH to Go1 Pi) — MUST START FIRST**
```bash
ssh pi@192.168.12.1
# Password: 123

# Copy relay script to Pi (first time only)
# From WSL: scp ~/verified_autonomy/lidar_relay.py pi@192.168.12.1:~/
sudo sysctl -w net.ipv4.ip_forward=1 
sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
python3 ~/lidar_relay.py 192.168.12.185
python3 ~/lidar_relay.py 192.168.12.185 --port-offset 1000
# Run the relay (replace with your PC's WiFi IP)
python3 ~/lidar_relay.py 192.168.12.185
```

You should see:
```
  [OK] Discovery: 192.168.1.50:56000 → 192.168.1.127:56000 (IP rewrite)
  [OK] Command: 192.168.1.50:56100 → 192.168.1.127:56100 (IP rewrite)
  [OK] Cmd Response: 192.168.1.50:56101 → 192.168.12.223:56101
  [OK] Point Cloud: 192.168.1.50:56301 → 192.168.12.223:56301
  ...
```

**Terminal 2: Robot control bridge (WSL)**
```bash
~/verified_autonomy/ros2_unitree_ws/scripts/go1_highlevel_bridge.sh
```

**Terminal 3: LiDAR driver (Docker)**
```bash
cd ~/verified_autonomy/docker
./shell.sh

# Inside container
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```
pkill -f livox_ros_driver
# Wait ~5 seconds, then check if the topic exists
ros2 topic list | grep livox

# If it appears, check data rate
ros2 topic hz /livox/lidar
**Terminal 4: Send commands (WSL)**
```bash
source /opt/ros/jazzy/setup.bash
source ~/verified_autonomy/ros2_unitree_ws/install/setup.bash

# Stand up first
python3 ~/verified_autonomy/ros2_unitree_ws/scripts/go1_cmd.py stand_up

# Walk while collecting LiDAR data
python3 ~/verified_autonomy/ros2_unitree_ws/scripts/go1_cmd.py walk 0.2 0 0
```

When the relay is working correctly, you should see `[REWRITE]` messages in the Pi relay terminal when the driver first connects, and `[Point Cloud] forwarded N packets` messages once data starts flowing.

### LiDAR Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | Point cloud data |
| `/livox/imu` | `sensor_msgs/Imu` | IMU from LiDAR |

### LiDAR Network Reference

| Device | IP Address | Interface |
|--------|------------|----------|
| Livox Mid-360 LiDAR | 192.168.1.127 | — |
| Go1 Pi (LiDAR side) | 192.168.1.50 | eth0 |
| Go1 Pi (hotspot side) | 192.168.12.1 | wlan1 |
| Your PC (on hotspot) | 192.168.12.x (DHCP) | WiFi |

### Troubleshooting LiDAR

**Cannot ping 192.168.1.131:**
- SSH to Pi and verify LiDAR is reachable from there first
- Check iptables NAT rules are active: `sudo iptables -L -t nat -v`
- Verify LiDAR is powered (may need robot to be standing)

**LiDAR driver starts but no data:**
- Verify `host_net_info` IPs in `MID360_config.json` match your PC's IP
- Check firewall: `sudo ufw allow 56100:56500/udp`
- Ensure LiDAR IP in config matches actual LiDAR IP

**Point cloud looks wrong:**
- Check extrinsic calibration in `config/livox/livox_mid360_calibration.yaml`
- Verify `pattern_mode` setting (0 = non-repeating, 1 = repeating)

---

## Direct Ethernet LiDAR Connection (Recommended)

The simplest way to get LiDAR data — connect the Livox Mid-360 directly to your PC via Ethernet. No relay scripts, no bridges, no tunneling.

### Network Architecture (Direct)

```
┌──────────────────────────────────────────────────┐
│                 WINDOWS PC                        │
│  ┌────────────────────────────────────────────┐  │
│  │          WSL2 (mirrored networking)         │  │
│  │  ┌──────────────────────────────────────┐  │  │
│  │  │     Docker (--net=host)               │  │  │
│  │  │  livox_ros_driver2                    │  │  │
│  │  │  → /lidar/scan, /imu/data            │  │  │
│  │  └──────────────────────────────────────┘  │  │
│  └────────────────────────────────────────────┘  │
│                                                   │
│  Ethernet adapter: 192.168.1.50 (static)          │
└────────────────────┬─────────────────────────────┘
                     │ Ethernet cable
                     ▼
           ┌─────────────────┐
           │  Livox Mid-360  │
           │  192.168.1.127  │
           └─────────────────┘
```

### Step 1: Set static IP on Windows Ethernet adapter

1. Open **Settings → Network & Internet → Ethernet**
2. Click on your Ethernet adapter (or **Advanced network settings → Ethernet → Edit**)
3. Click **Edit** next to IP assignment
4. Switch from **Automatic (DHCP)** to **Manual**
5. Enable **IPv4** and set:
   - **IP address**: `192.168.1.50`
   - **Subnet mask**: `255.255.255.0` (or prefix length `24`)
   - **Gateway**: leave blank
   - **DNS**: leave blank
6. Click **Save**

Or via PowerShell (admin):

```powershell
# Find your Ethernet adapter name
Get-NetAdapter | Where-Object { $_.InterfaceDescription -like "*Ethernet*" -and $_.Name -notlike "*vEthernet*" }

# Set static IP (replace "Ethernet" with your adapter name)
New-NetIPAddress -InterfaceAlias "Ethernet" -IPAddress 192.168.1.50 -PrefixLength 24
```

### Step 2: Plug in and test

Connect the LiDAR to your PC's Ethernet port, then test:

```powershell
ping 192.168.1.127
```

You should get replies. If not:
- Check the cable
- Verify the static IP is set: `ipconfig | findstr 192.168.1`
- Disable Windows Firewall temporarily: `Set-NetFirewallProfile -Profile Domain,Public,Private -Enabled False`

### Step 3: Verify WSL can see the LiDAR (mirrored networking)

Since WSL is in mirrored mode, it shares your Windows network:

```bash
# In WSL
ping -c 2 192.168.1.127
```

### Step 4: Run the LiDAR driver

Only **one terminal** needed — no relay, no bridge:

```bash
cd ~/verified_autonomy/docker
./shell.sh

# Inside container
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py

```

You should see the driver initialize without `getFreeIndex` errors.

### Step 5: Verify data

In another Docker shell (or WSL terminal):

```bash
# Check topics exist
ros2 topic list | grep -E "lidar|imu"

# Check point cloud data rate (should be ~10 Hz)
ros2 topic hz /lidar/scan

# Check IMU data rate
ros2 topic hz /imu/data
```

### MID360_config.json (Direct Ethernet)

The config should have your PC's static IP as the host and the real LiDAR IP:

```json
{
  "lidar_summary_info": { "lidar_type": 8 },
  "MID360": {
    "lidar_net_info": {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info": {
      "cmd_data_ip": "192.168.1.50",
      "cmd_data_port": 56101,
      "push_msg_ip": "192.168.1.50",
      "push_msg_port": 56201,
      "point_data_ip": "192.168.1.50",
      "point_data_port": 56301,
      "imu_data_ip": "192.168.1.50",
      "imu_data_port": 56401,
      "log_data_ip": "",
      "log_data_port": 56501
    }
  },
  "lidar_configs": [
    {
      "ip": "192.168.1.127",
      "pcl_data_type": 1,
      "pattern_mode": 0,
      "extrinsic_parameter": {
        "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
        "x": 0, "y": 0, "z": 0
      }
    }
  ]
}
```

### ROS 2 Topics (Direct)

| Topic | Type | Description |
|-------|------|-------------|
| `/lidar/scan` | `livox_ros_driver2/CustomMsg` | Point cloud data (remapped from `livox/lidar`) |
| `/imu/data` | `sensor_msgs/Imu` | IMU from LiDAR (remapped from `livox/imu`) |

---

## Visualizing LiDAR Data with Foxglove

[Foxglove](https://foxglove.dev/) provides a web-based 3D viewer for ROS 2 data. Use it to visualize point clouds and IMU data from the LiDAR.

### Install Foxglove Bridge (one-time, inside Docker)

```bash
sudo apt-get update
sudo apt-get install -y ros-jazzy-foxglove-bridge
```

### Run Foxglove Bridge

In a Docker terminal (with the LiDAR driver already running):

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

This starts a WebSocket server on port `8765`.

### Open Foxglove Studio

**Option A: Desktop app** (recommended)
1. Download from [foxglove.dev/download](https://foxglove.dev/download)
2. Open connection → **Foxglove WebSocket**
3. URL: `ws://localhost:8765`

**Option B: Web app**
1. Go to [app.foxglove.dev](https://app.foxglove.dev)
2. Open connection → **Foxglove WebSocket**
3. URL: `ws://localhost:8765`

> With mirrored WSL networking, `localhost:8765` from Windows reaches the Docker container directly.

### Set Up 3D Point Cloud View

1. Add a **3D** panel (click `+` → `3D`)
2. In the panel settings(设置 gear), under **Topics(话题)**, enable `/lidar/scan`
3. Set **Frame** to `livox_frame`
4. You should see the point cloud rendering in real-time

### Set Up IMU View

1. Add a **Plot** panel
2. Subscribe to `/imu/data`
3. Plot `angular_velocity.x`, `angular_velocity.y`, `angular_velocity.z`
4. Or add a **Raw Messages** panel to see the full IMU data

### Recommended Foxglove Layout

| Panel | Topic | Notes |
|-------|-------|-------|
| 3D | `/lidar/scan` | Point cloud visualization, frame = `livox_frame` |
| Plot | `/imu/data` | Angular velocity & linear acceleration |
| Diagnostics | `/lidar/scan` | Message rate, latency |

### Troubleshooting Foxglove

**Cannot connect to `ws://localhost:8765`:**
- Verify bridge is running: `ros2 node list | grep foxglove`
- Check port isn't blocked: `Test-NetConnection localhost -Port 8765` (PowerShell)
- Try `ws://127.0.0.1:8765` instead

**Point cloud shows but looks wrong:**
- Set the correct frame ID (`livox_frame`)
- Adjust point size in the 3D panel settings
- Check `pattern_mode` in config (0 = non-repeating scan for better coverage)

**No data appearing:**
- Confirm driver is publishing: `ros2 topic hz /lidar/scan`
- Make sure the topic is enabled in the Foxglove 3D panel settings

---

## Autonomous Exploration (Direct Ethernet LiDAR + WiFi Robot Control)

Run the full autonomy stack on your PC: LiDAR data comes in via Ethernet, SLAM and exploration planning run in Docker, and velocity commands are sent to the Go1 over WiFi.

### Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         WINDOWS PC                               │
│                                                                  │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │              Docker (--net=host)                          │    │
│  │  livox_ros_driver2 → /lidar/scan, /imu/data              │    │
│  │  arise_slam        → /state_estimation, TF (map→sensor)  │    │
│  │  terrain_analysis  → traversability                       │    │
│  │  local_planner     → /cmd_vel (TwistStamped)              │    │
│  │  tare_planner      → exploration waypoints                │    │
│  │  foxglove_bridge   → ws://localhost:8765                  │    │
│  └─────────────────────────────────────────────────────────┘    │
│                              │ /cmd_vel                          │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │              WSL2 (mirrored networking)                    │    │
│  │  cmd_vel_to_high_cmd.py  → /high_cmd (HighCmd)            │    │
│  │  ros2_udp HIGHLEVEL      → UDP to 192.168.123.161:8082    │    │
│  └─────────────────────────────────────────────────────────┘    │
│                                                                  │
│  Ethernet (192.168.1.50) ────── LiDAR (192.168.1.127)           │
│  WiFi (192.168.12.x)    ────── Go1 Hotspot (192.168.12.1)       │
└─────────────────────────────────────────────────────────────────┘
```

### Prerequisites

1. **LiDAR connected via Ethernet** — USB-C Ethernet adapter, static IP `192.168.1.50` (see Direct Ethernet section above)
2. **Go1 WiFi connected** — your PC connected to `Unitree_GoXXXX` hotspot
3. **Both networks active simultaneously** — Ethernet for LiDAR, WiFi for robot control
4. **IP forwarding on Go1 Pi** (for robot control):
   ```bash
   ssh pi@192.168.12.1  # password: 123
   sudo sysctl -w net.ipv4.ip_forward=1
   sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
   exit
   ```

### One-Time Build (with internet)

Inside the Docker container, build the SLAM dependencies and full stack:

```bash
cd ~/verified_autonomy/docker
sudo service docker start
./shell.sh

# Inside container — install SLAM dependencies
cd /workspace/src/slam/dependency/Sophus
mkdir -p build && cd build && cmake .. -DBUILD_TESTS=OFF && sudo make install

cd /workspace/src/slam/dependency/ceres-solver
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF
make -j4 && sudo make install

cd /workspace/src/slam/dependency/gtsam
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_UNSTABLE=ON -DGTSAM_WITH_TBB=OFF
make -j4 && sudo make install && sudo ldconfig

# Build full workspace including SLAM
source /opt/ros/jazzy/setup.bash
cd /workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Install Foxglove bridge
sudo apt-get update && sudo apt-get install -y ros-jazzy-foxglove-bridge
```

Also copy the bridge script to WSL:

```bash
# In WSL (not Docker)
cp /mnt/c/Users/willi/.vscode/cmd_vel_to_high_cmd.py ~/verified_autonomy/
```

### Quick Start — Autonomous Exploration

#### Terminal 1: Highlevel Bridge (WSL)

```bash
cd ~/verified_autonomy/ros2_unitree_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=~/verified_autonomy/ros2_unitree_ws/src/unitree_legged_sdk/lib:$LD_LIBRARY_PATH
ros2 run unitree_legged_real ros2_udp HIGHLEVEL
```

Expected output:
```
UDP Initialized. socketfd: 3   Port: 8080
UDP Initialized. socketfd: 4   Port: 8090
high level runing!
```

#### Terminal 2: cmd_vel → high_cmd Bridge (WSL)

```bash
source /opt/ros/jazzy/setup.bash
source ~/verified_autonomy/ros2_unitree_ws/install/setup.bash
python3 ~/verified_autonomy/cmd_vel_to_high_cmd.py
```

This translates `/cmd_vel` (TwistStamped from the exploration stack) into `/high_cmd` (HighCmd for the Go1).

#### Terminal 3: Exploration Stack (Docker)

```bash
cd ~/verified_autonomy/docker
./shell.sh

# Inside container
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
ros2 launch vehicle_simulator system_real_robot_with_exploration_planner.launch
```

> **Note**: The launch includes the LiDAR driver. Make sure no other LiDAR driver instance is running. If you get `bind failed`, kill old processes first:
> ```bash
> pkill -f livox_ros_driver2
> ```

#### Terminal 4: Foxglove Bridge (Docker — second shell)

```bash
cd ~/verified_autonomy/docker
./shell.sh

# Inside container
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

Then open **Foxglove Studio** (desktop app) → `ws://localhost:8765`

### Key Topics for Monitoring

| Topic | Type | Description |
|-------|------|-------------|
| `/lidar/scan` | `PointCloud2` | Raw LiDAR point cloud |
| `/imu/data` | `Imu` | LiDAR IMU |
| `/state_estimation` | `Odometry` | SLAM pose estimate |
| `/cmd_vel` | `TwistStamped` | Velocity commands from planner |
| `/high_cmd` | `HighCmd` | Commands sent to Go1 |
| `/terrain_map` | `PointCloud2` | Traversability analysis |
| `/exploring_cells` | `MarkerArray` | TARE exploration frontiers |

### Safety

- The `cmd_vel_to_high_cmd.py` bridge has a **0.5s timeout** — if the exploration stack stops sending commands, the robot stops automatically
- Velocity is **clamped** to: vx=0.4 m/s, vy=0.3 m/s, yaw=0.8 rad/s
- Press **Ctrl+C** on any terminal to stop
- The Go1 will enter `force_stand` mode when commands stop

### Troubleshooting Autonomous Exploration

**LiDAR driver: `bind failed`:**
- Another instance is using the ports. Kill it: `pkill -f livox_ros_driver2`
- Wait 2 seconds, then relaunch

**Robot not moving:**
- Check highlevel bridge is running (Terminal 1)
- Check cmd_vel bridge is running (Terminal 2): `ros2 topic echo /high_cmd --once`
- Verify exploration stack publishes: `ros2 topic hz /cmd_vel`
- Make sure robot is standing first: `python3 ~/verified_autonomy/ros2_unitree_ws/scripts/go1_cmd.py stand_up`

**SLAM not working:**
- Check LiDAR data: `ros2 topic hz /lidar/scan` (should be ~10 Hz)
- Check IMU data: `ros2 topic hz /imu/data`
- Verify TF tree: `ros2 run tf2_tools view_frames`

**Cannot connect to Go1 AND LiDAR simultaneously:**
- WiFi must be on Go1 hotspot (192.168.12.x)
- Ethernet must have static IP 192.168.1.50
- Verify both: `ping 192.168.12.1` (Go1) and `ping 192.168.1.127` (LiDAR)
