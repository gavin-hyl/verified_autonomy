# Troubleshooting Log

## Issue #1: Livox LiDAR Driver "bind failed"

**Date**: 2026-02-07  
**Component**: `livox_ros_driver2` (`ros2 launch livox_ros_driver2 msg_MID360_launch.py`)  
**Environment**: WSL2 Ubuntu 24.04 → Docker container (`network_mode: host`) → Livox SDK 2

### Error Output

```
[livox_ros_driver2_node-1] LdsLidar *GetInstance
[livox_ros_driver2_node-1] config lidar type: 8
[livox_ros_driver2_node-1] successfully parse base config, counts: 1
[livox_ros_driver2_node-1] bind failed
[livox_ros_driver2_node-1] Failed to init livox lidar sdk.
[livox_ros_driver2_node-1] [ERROR] [livox_lidar_publisher]: Init lds lidar fail!
```

### Root Cause Analysis

The Livox SDK calls `bind()` on a UDP socket using the IP specified in `host_net_info` of `MID360_config.json`. This fails when:

1. **Stale process holding ports** — A previous `livox_ros_driver2` instance didn't fully terminate and is still holding the UDP ports. Check with:
   ```bash
   ps aux | grep livox | grep -v grep
   ss -ulnp | grep livox
   ```
   Fix: `kill -9 <pid>` or `pkill -9 -f livox_ros_driver`

2. **Config IP doesn't exist on host** (this was the actual cause) — The config had `host_net_info` IPs set to `192.168.1.5`, which is NOT an IP address on WSL2's network interfaces. The SDK tries to `bind()` to that specific IP and fails because it doesn't exist locally.
   - Config file: `~/verified_autonomy/vector_navigation_stack/src/utilities/livox_ros_driver2/config/MID360_config.json`
   - WSL2 does NOT have a `192.168.1.x` address — it only has its virtual NAT address (e.g., `172.x.x.x`)
   - Even with NAT/masquerade routing through the Go1 Pi, WSL2 can *reach* `192.168.1.131` but cannot *bind* to a `192.168.1.x` address

### Solution

Set all `host_net_info` IPs to `0.0.0.0` (bind to all interfaces) in `MID360_config.json`:

```json
"host_net_info" : {
  "cmd_data_ip" : "0.0.0.0",
  "push_msg_ip": "0.0.0.0",
  "point_data_ip": "0.0.0.0",
  "imu_data_ip" : "0.0.0.0",
  "log_data_ip" : "",
  ...
}
```

**Important**: Using `0.0.0.0` fixes the bind, but the LiDAR still needs to know where to send data back. The Livox SDK sends a "handshake" to the LiDAR telling it the host IP. With `0.0.0.0`, the SDK may auto-detect, OR you may need the WSL2 IP that can actually route to `192.168.1.131`. Check with:

```bash
ip route get 192.168.1.131 | grep -oP 'src \K\S+'
```

### Additional Notes

- **`sshuttle` does NOT help** — it only proxies TCP traffic. The Livox LiDAR communicates entirely over UDP.
- **Docker `network_mode: host`** means the container shares WSL2's network stack, so the bind issue is identical whether running inside or outside Docker.
- The `not found: "/unitree_ws/install/..."` warnings on container startup are harmless — those packages aren't built in the Docker workspace and can be ignored.
- `pkill` exit code 1 means "no matching processes" — it's not an error, just means nothing was running.

### Diagnostic Checklist

```bash
# 1. Check for stale processes
ps aux | grep livox | grep -v grep

# 2. Check for held ports
ss -ulnp | grep -E 'livox|561'

# 3. Check what IPs WSL2 has
ip -4 addr show | grep "inet "

# 4. Check the config file
cat ~/verified_autonomy/vector_navigation_stack/src/utilities/livox_ros_driver2/config/MID360_config.json

# 5. Verify the host_net_info IPs exist on this machine
# If config says 192.168.1.5, run:
ip addr | grep 192.168.1.5
# Empty output = that IP doesn't exist locally = bind will fail
```

### Status

- [x] Identified stale process as initial suspect (killed successfully)
- [x] Confirmed bind still fails after killing processes
- [x] Found real root cause: config `host_net_info` IPs set to `192.168.1.5` which doesn't exist on WSL2
- [x] Fix config to `0.0.0.0` — **RESOLVED**: driver now says `Init lds lidar success!`
- [ ] Verify LiDAR data is actually received after bind succeeds

### Fix Applied

```bash
sed -i 's/"192.168.1.5"/"0.0.0.0"/g' ~/verified_autonomy/vector_navigation_stack/src/utilities/livox_ros_driver2/config/MID360_config.json
```

---

## Issue #2: LiDAR Driver Initializes but No Point Cloud Data (Potential)

**Date**: 2026-02-07  
**Status**: Pending verification  
**Component**: Livox Mid-360 → WSL2 UDP return path

### Problem

Even after the driver initializes successfully, the LiDAR point cloud data may not arrive. The Livox protocol works as follows:

1. Driver (WSL2) sends handshake/command to LiDAR at `192.168.1.131:56100`
2. Pi NAT translates: source becomes `192.168.1.50:<random_port>`
3. LiDAR responds to `192.168.1.50:<random_port>` — Pi conntrack routes this back ✓
4. **BUT**: LiDAR then streams point cloud to `host_net_info` ports (56301, 56401, etc.)
5. These are NEW unsolicited UDP flows — Pi conntrack does NOT recognize them
6. Pi drops the packets ✗

### Diagnosis

```bash
# Inside Docker container, check if data flows:
ros2 topic hz /livox/lidar
# If "no new messages" → return path is broken

# On the Pi, check if LiDAR is sending data:
ssh pi@192.168.12.1
sudo tcpdump -i eth0 src 192.168.1.131 -c 20
# If you see packets → LiDAR is active but data isn't reaching WSL2
```

### Potential Solutions

**Option A: Port forwarding on Pi (simplest)**
```bash
# SSH to Pi
ssh pi@192.168.12.1

# Get your WSL2 IP as seen by the Pi (your WiFi hotspot IP)
# e.g., 192.168.12.223

# Forward LiDAR data ports to your PC
sudo iptables -t nat -A PREROUTING -i eth0 -p udp -s 192.168.1.131 --dport 56101 -j DNAT --to-destination 192.168.12.YOUR_IP:56101
sudo iptables -t nat -A PREROUTING -i eth0 -p udp -s 192.168.1.131 --dport 56201 -j DNAT --to-destination 192.168.12.YOUR_IP:56201
sudo iptables -t nat -A PREROUTING -i eth0 -p udp -s 192.168.1.131 --dport 56301 -j DNAT --to-destination 192.168.12.YOUR_IP:56301
sudo iptables -t nat -A PREROUTING -i eth0 -p udp -s 192.168.1.131 --dport 56401 -j DNAT --to-destination 192.168.12.YOUR_IP:56401
sudo iptables -t nat -A PREROUTING -i eth0 -p udp -s 192.168.1.131 --dport 56501 -j DNAT --to-destination 192.168.12.YOUR_IP:56501
```

**Option B: Run driver on the Pi directly**
- SSH into Pi, install Livox SDK + driver, run directly on `192.168.1.50`
- Forward ROS 2 topics to WSL2 via DDS/Zenoh bridge

**Option C: Run driver on Go1's Nano/NX board**
- The Go1 has Jetson boards on the internal network that can run the driver natively

---

## Issue #3: Robot Doesn't Move Despite Bridge Running

**Date**: 2026-02-07  
**Component**: `ros2_udp HIGHLEVEL` bridge + `go1_cmd.py`

### Symptom

The `ros2_udp HIGHLEVEL` bridge starts successfully (shows "high level runing!"), `go1_cmd.py` runs without errors, but the robot does not move.

### Possible Causes

**1. Bridge and command script in different ROS 2 environments (MOST LIKELY)**

The `go1_highlevel_bridge.sh` is designed to run **inside Docker** (`/unitree_ws`). The Docker container has `ROS_DOMAIN_ID=0`. If `go1_cmd.py` runs in WSL without `ROS_DOMAIN_ID` set, it defaults to `0` too — so DDS domain matches.

However, DDS discovery can still fail between Docker and WSL processes even with `network_mode: host` if multicast doesn't work properly.

**Diagnosis:**
```bash
# In the terminal where go1_cmd.py runs, check if it can see the bridge's topics:
ros2 topic list | grep high
# Should show:
#   /high_cmd
#   /high_state

# Check if bridge is actually receiving messages:
ros2 topic echo /high_state --once
# If this hangs → bridge and your terminal are NOT discovering each other

# Check subscriber count:
ros2 topic info /high_cmd
# Should show 1 publisher (go1_cmd.py) and 1 subscriber (ros2_udp bridge)
```

**Fix**: Run BOTH the bridge AND `go1_cmd.py` from the **same environment** (both in Docker, or both in WSL):

```bash
# Option A: Both in Docker
# Terminal 1 (Docker shell):
source /opt/ros/jazzy/setup.bash
source /unitree_ws/install/setup.bash
ros2 run unitree_legged_real ros2_udp HIGHLEVEL

# Terminal 2 (another Docker shell):
source /opt/ros/jazzy/setup.bash
source /unitree_ws/install/setup.bash
python3 /unitree_ws/scripts/go1_cmd.py stand_up

# Option B: Both in WSL (using WSL-built workspace)
# Terminal 1 (WSL):
cd ~/verified_autonomy/ros2_unitree_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=~/verified_autonomy/ros2_unitree_ws/src/unitree_legged_sdk/lib:$LD_LIBRARY_PATH
ros2 run unitree_legged_real ros2_udp HIGHLEVEL

# Terminal 2 (WSL):
source /opt/ros/jazzy/setup.bash
source ~/verified_autonomy/ros2_unitree_ws/install/setup.bash
python3 ~/verified_autonomy/ros2_unitree_ws/scripts/go1_cmd.py stand_up
```

**2. Robot not in correct state**

The Go1 may need to be standing before it accepts walk commands. Always send `stand_up` first, wait 3 seconds, then send `walk`.

**3. `go1_cmd.py` duration too short**

The script sends commands for only 2 seconds (3 for stand_up/stand_down). If the robot takes time to process, the command window may be too short. Try increasing duration:
```bash
python3 go1_cmd.py walk 0.3 0 0 5   # 5 second duration (last arg)
```

**4. Bridge can't reach control board**

Even though bridge starts, it may fail silently if UDP to `192.168.123.161` is not working.
```bash
# Verify connectivity first
ping -c 2 192.168.123.161
```

### Recommended Startup Order

```bash
# 1. SSH to Pi, enable forwarding + NAT (if not already done)
# 2. Verify ping to 192.168.123.161
# 3. Start bridge (in WSL directly, NOT Docker):
cd ~/verified_autonomy/ros2_unitree_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=$PWD/src/unitree_legged_sdk/lib:$LD_LIBRARY_PATH
ros2 run unitree_legged_real ros2_udp HIGHLEVEL

# 4. In new WSL terminal, send commands:
source /opt/ros/jazzy/setup.bash
source ~/verified_autonomy/ros2_unitree_ws/install/setup.bash
python3 ~/verified_autonomy/ros2_unitree_ws/scripts/go1_cmd.py stand_up
# Wait 3-4 seconds, then:
python3 ~/verified_autonomy/ros2_unitree_ws/scripts/go1_cmd.py walk 0.3 0 0
```

### Update: User confirms both run in WSL, used to work

Both the bridge and `go1_cmd.py` are run in WSL (not Docker). This used to work before the LiDAR investigation. The DDS environment mismatch theory is ruled out.

### Revised Diagnosis (when next at the robot)

The bridge code (`ros2_udp.cpp`) is purely callback-driven. It prints `highCmdCallback is running!` every time it receives a `/high_cmd` message. This is the key diagnostic:

**Step 1: Check if bridge receives ROS messages**
```bash
# Watch the bridge terminal output when you run go1_cmd.py
# It should print "highCmdCallback is running!" repeatedly
```

**Step 2: If NO callbacks printed** → DDS discovery issue. Verify with:
```bash
# Terminal 3: while bridge runs in terminal 1
ros2 topic info /high_cmd
# Should show: 1 publisher, 1 subscriber

ros2 topic echo /high_cmd --once
# Then run go1_cmd.py in terminal 2 — should show the message
```

**Step 3: If callbacks ARE printed but robot doesn't move** → One of:
- Robot in **protection mode** (low battery, overheated, fell over) → power cycle
- Robot sport mode controller crashed → power cycle
- UDP packets not reaching `192.168.123.161:8082` → check with `ping -c2 192.168.123.161`
- Go1 Pi iptables rules got corrupted from all the debugging → re-run:
  ```bash
  ssh pi@192.168.12.1
  sudo iptables -F          # flush all rules
  sudo iptables -t nat -F   # flush NAT rules
  sudo sysctl -w net.ipv4.ip_forward=1
  sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
  ```

### Key Code Details (ros2_udp.cpp)

- Bridge sends UDP to `192.168.123.161:8082` (hardcoded in constructor)
- Bridge listens on local port `8090`
- Subscribes to `/high_cmd` with QoS depth 1
- Only sends UDP inside the callback — no periodic heartbeat
- Prints `highCmdCallback is running!` + counter for each message

### Status

- [x] Ruled out DDS environment mismatch (both in WSL)
- [ ] When at robot: check if bridge prints callbacks
- [ ] When at robot: if yes, check ping to 192.168.123.161
- [ ] When at robot: try power cycling robot
- [ ] When at robot: flush and re-add iptables rules on Pi

---

## Issue #4: LiDAR "bind failed" vs "no data" Dilemma

**Date**: 2026-02-10  
**Component**: `livox_ros_driver2` / `MID360_config.json` / `lidar_relay.py`

### The Dilemma

The `host_net_info` IPs in `MID360_config.json` serve **two conflicting purposes**:

1. **Local `bind()`** — The Livox SDK calls `bind(host_ip, port)` to listen for incoming data. This requires the IP to exist on the local machine.
2. **Livox handshake protocol** — The SDK packs these IPs into a binary command payload and sends it to the LiDAR firmware. The LiDAR reads these IPs and sends data streams to them.

| Config Value | `bind()` | LiDAR Sends Data To | Result |
|-------------|----------|---------------------|--------|
| `192.168.1.50` | ❌ FAIL (IP doesn't exist in Docker/WSL2) | ✅ Pi (reachable) | **bind failed** |
| `0.0.0.0` | ✅ Binds to all interfaces | ❌ Literal `{0,0,0,0}` sent to LiDAR — meaningless | **No data** |
| `192.168.12.x` | ✅ Binds locally | ❌ LiDAR can't route to WiFi subnet | **No data** |

**None of them work alone.**

### Root Cause (confirmed via Livox SDK2 source code)

The Livox SDK2 (`BuildRequest::BuildUpdateMid360LidarCfgRequest`) embeds `host_net_info` IPs as raw 4-byte values in `HostIpInfoValue` structs inside command payloads. When set to `0.0.0.0`, it literally sends `{0, 0, 0, 0}` — there is **no auto-detection** of the outgoing interface IP. The LiDAR firmware receives this and either ignores it or tries to send data to `0.0.0.0`, resulting in no data reaching the driver.

### Solution: Pi Relay with Handshake IP Rewrite

Updated `lidar_relay.py` (v2) to solve both problems simultaneously:

1. **Docker config uses `host_net_info` = `0.0.0.0`** → `bind()` succeeds
2. **Docker config uses `lidar_configs.ip` = `192.168.1.50`** → commands route through Pi relay (not directly to LiDAR)
3. **Pi relay intercepts commands** and scans for the `{0,0,0,0}` pattern followed by a Livox port number (56xxx range)
4. **Relay rewrites `0.0.0.0` → `192.168.1.50`** in the handshake payload before forwarding to the real LiDAR
5. **LiDAR receives corrected handshake** — sends data streams to `192.168.1.50:56x01` (the Pi)
6. **Relay catches data on Pi** → forwards to PC over WiFi

```
Docker/WSL2                     Go1 Pi                        LiDAR
─────────────                   ──────                        ─────
SDK bind(0.0.0.0) ✓
SDK sends cmd with              relay intercepts
  host=0.0.0.0    ──────────►   rewrites 0.0.0.0→1.50  ────► receives host=1.50
                                                              sends data to 1.50
SDK recv on 0.0.0.0 ◄────────  relay fwd to PC  ◄──────────  data → 1.50:56301
```

### Correct Config

```json
{
  "host_net_info" : {
    "cmd_data_ip" : "0.0.0.0",
    "push_msg_ip": "0.0.0.0",
    "point_data_ip": "0.0.0.0",
    "imu_data_ip" : "0.0.0.0",
    "log_data_ip" : ""
  },
  "lidar_configs" : [
    { "ip" : "192.168.1.50" }
  ]
}
```

### Startup Order

1. SSH to Pi → run `python3 lidar_relay.py 192.168.12.YOUR_IP` (MUST be first)
2. Docker → launch `livox_ros_driver2`
3. Relay should print `[REWRITE]` when handshake is intercepted
4. Relay should print `[Point Cloud] forwarded N packets` when data flows

### Status

- [x] Identified bind vs no-data dilemma
- [x] Confirmed via Livox SDK2 source: `0.0.0.0` is sent literally, no auto-detect
- [x] Updated `lidar_relay.py` v2 with handshake IP rewrite
- [x] Updated `MID360_config.json` example in docs
- [ ] Test at robot: verify `[REWRITE]` appears in relay output
- [ ] Test at robot: verify point cloud data arrives in Docker

---

## Issue #5: Robot Doesn't Move After "IMU Initialization Process Finish!"

**Date**: 2026-02-13  
**Component**: `featureExtraction` node in `arise_slam_mid360`  
**Environment**: Full exploration stack via `system_real_robot_with_exploration_planner.launch`

### Symptom

After launching the full autonomy stack, `imu_preintegration_node` prints "IMU Initialization Process Finish!" but the robot never moves. All downstream topics are dead:
- `/state_estimation` ❌
- `/registered_scan` ❌  
- `/terrain_map` ❌
- `/way_point` ❌
- `/path` ❌
- `/cmd_vel` ❌

### Root Cause: Message Type Mismatch (PointCloud2 vs CustomMsg)

The pipeline breaks at the **first SLAM node** (`featureExtraction`), which receives LiDAR data but produces no output (`/feature_info` is never published).

**The mismatch:**

| Component | File | Setting | ROS Message Type |
|---|---|---|---|
| Livox Driver | `msg_MID360_launch.py` | `xfer_format = 0` | `sensor_msgs/msg/PointCloud2` |
| SLAM Feature Extraction | `livox_mid360.yaml` | `sensor: "livox"` | `livox_ros_driver2/msg/CustomMsg` |

In `featureExtraction.cpp` (lines 69-76), when `sensor == LIVOX`, the node subscribes to `livox_ros_driver2/msg/CustomMsg`. But the driver was publishing `sensor_msgs/msg/PointCloud2` (because `xfer_format = 0`).

ROS 2 silently ignores type mismatches — the subscription simply never receives data. No error is logged.

### Diagnosis Method

1. Checked topic rates: `/lidar/scan` ~10Hz ✅, `/imu/data` ~215Hz ✅ → data is flowing
2. Checked SLAM intermediate topics: `/feature_info` ❌ DEAD → pipeline breaks here
3. Read `featureExtraction.cpp` source: sensor type `LIVOX` → subscribes to `CustomMsg`
4. Read `msg_MID360_launch.py`: `xfer_format = 0` → publishes `PointCloud2`
5. Confirmed mismatch: driver outputs PointCloud2, SLAM expects CustomMsg

### Fix

Changed `xfer_format` from `0` to `1` in `msg_MID360_launch.py`:

**File**: `/workspace/src/utilities/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py`  
**Also**: `/workspace/install/livox_ros_driver2/share/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py`

```python
# BEFORE (broken):
xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format

# AFTER (fixed):
xfer_format   = 1    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
```

### Verification

After the fix, the driver correctly reports:
```
[livox_ros_driver2_node-1] livox/lidar publish use livox custom format
```

And `ros2 topic info /lidar/scan` shows:
```
Type: livox_ros_driver2/msg/CustomMsg
Publisher count: 1
```

### Status

- [x] Identified pipeline break at `featureExtraction` (no `/feature_info`)
- [x] Traced to message type mismatch (PointCloud2 vs CustomMsg)
- [x] Fixed `xfer_format` in both source and install copies
- [x] Verified driver publishes CustomMsg after fix
- [ ] Test full stack at robot: verify `/feature_info` is published
- [ ] Test full stack at robot: verify robot moves autonomously
