# System Flow – LiDAR & Autonomous Exploration Stack

> Full data-flow reference for the Unitree Go1 + Livox Mid-360 autonomous exploration system.
> All nodes run inside the **`vector-autonomy-ros`** Docker container (ROS 2 Jazzy).

---

## 1  Physical Topology

```
┌──────────────┐  Ethernet (USB adapter)    ┌─────────────────────┐
│ Livox Mid-360│ ◄──────────────────────────►│  Linux PC            │
│ 192.168.1.1xx│     192.168.1.50/24         │  (Native Linux)      │
└──────────────┘                             │                      │
                                             │  Docker: SLAM +      │
                                             │  Exploration Planner +│
                                             │  cmd bridge +        │
                                             │  ros2_udp highLevel   │
                                             └──────────┬───────────┘
                                                        │ WiFi
                                                        │ 192.168.123.161
                                             ┌──────────▼───────────┐
                                             │   Unitree Go1 Robot  │
                                             │   (sport-mode board) │
                                             └──────────────────────┘
```

| Segment | Protocol | Interface |
|---------|----------|-----------|
| LiDAR ↔ PC | UDP (Livox SDK) | Ethernet adapter (e.g., `enx*`, `eth*`), static IP 192.168.1.50 |
| Docker ↔ Host | ROS 2 DDS (`--net=host`) | Shared network namespace |
| PC → Go1 | UDP (`ros2_udp HIGHLEVEL`) | WiFi to 192.168.123.161:8091 |
| PC ← Go1 | UDP state feedback | WiFi from 192.168.123.161:8082 |

---

## 2  What is `geometry_msgs/TwistStamped`?

`TwistStamped` is the **final output** of the autonomy stack. It is a standard ROS 2 message representing a velocity command with a timestamp:

```
std_msgs/Header header        # stamp + frame_id
geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
        float64 x              # forward/back  (m/s)
        float64 y              # left/right    (m/s)
        float64 z              # up/down       (m/s)  – unused for ground robot
    geometry_msgs/Vector3 angular
        float64 x              # roll rate     – unused
        float64 y              # pitch rate    – unused
        float64 z              # yaw rate      (rad/s)
```

The **`pathFollower`** node publishes this on `/cmd_vel` every control cycle.
Only `linear.x`, `linear.y`, and `angular.z` are meaningful for the Go1.

---

## 3  Node-by-Node Data Flow

The exploration stack is launched by **`system_real_robot_with_exploration_planner.launch`**.
Below is every node in execution order, its subscriptions (→ in) and publications (← out).

---

### 3.1  `livox_ros_driver2` — LiDAR Driver

| Direction | Topic | Message Type | Description |
|-----------|-------|-------------|-------------|
| ← out | `/lidar/scan` | `sensor_msgs/PointCloud2` | Raw 3D point cloud (remapped from `livox/lidar`) |
| ← out | `/imu/data` | `sensor_msgs/Imu` | Built-in 6-axis IMU (remapped from `livox/imu`) |

**Config:** `MID360_config.json` — host IP `192.168.1.50`, LiDAR IP `192.168.1.127`, `xfer_format = 0` (PointCloud2).

---

### 3.2  `arise_slam_mid360` — LiDAR-Inertial SLAM

Three internal nodes sharing a component container:

#### 3.2.1  Feature Extraction (`featureExtraction`)

| Direction | Topic | Message Type |
|-----------|-------|-------------|
| → in | `/lidar/scan` | `sensor_msgs/PointCloud2` |
| → in | `/imu/data` | `sensor_msgs/Imu` |
| ← out | `/feature_info` | `arise_slam_mid360_msgs/LaserFeature` |
| ← out | `/velodyne_cloud_2` | `sensor_msgs/PointCloud2` |
| ← out | `/planner_points` | `sensor_msgs/PointCloud2` |
| ← out | `/edge_points` | `sensor_msgs/PointCloud2` |

#### 3.2.2  Laser Mapping (`laserMapping`)

| Direction | Topic | Message Type |
|-----------|-------|-------------|
| → in | `/feature_info` | `arise_slam_mid360_msgs/LaserFeature` |
| → in | `/integrated_to_init5` | `nav_msgs/Odometry` |
| ← out | **`/registered_scan`** | `sensor_msgs/PointCloud2` | 
| ← out | `/laser_odometry` | `nav_msgs/Odometry` |
| ← out | `/laser_cloud_surround` | `sensor_msgs/PointCloud2` |
| ← out | `/laser_cloud_map` | `sensor_msgs/PointCloud2` |
| ← out | `/overall_map` | `sensor_msgs/PointCloud2` |
| ← out | `/aft_mapped_to_init_incremental` | `nav_msgs/Odometry` |

#### 3.2.3  IMU Pre-integration (`imuPreintegration`)

| Direction | Topic | Message Type |
|-----------|-------|-------------|
| → in | `/imu/data` | `sensor_msgs/Imu` |
| → in | `/laser_odometry` | `nav_msgs/Odometry` |
| ← out | **`/state_estimation`** | `nav_msgs/Odometry` | 
| ← out | `/state_estimation_health` | `std_msgs/Bool` |
| ← out | `/imuodom_path` | `nav_msgs/Path` |

> **Key outputs:** `/registered_scan` (map-aligned point cloud) and `/state_estimation` (fused pose).
> `PROJECT_NAME` is `""` (empty), so topics have no prefix.

---

### 3.3  `sensorScanGeneration` — Scan Synchronizer

Time-synchronises the state estimate with the registered scan.

| Direction | Topic | Message Type |
|-----------|-------|-------------|
| → in | `/state_estimation` | `nav_msgs/Odometry` |
| → in | `/registered_scan` | `sensor_msgs/PointCloud2` |
| ← out | `/state_estimation_at_scan` | `nav_msgs/Odometry` |
| ← out | `/sensor_scan` | `sensor_msgs/PointCloud2` |

---

### 3.4  `terrainAnalysis` — Terrain Classification

Classifies the registered scan into traversable/obstacle terrain.

| Direction | Topic | Message Type |
|-----------|-------|-------------|
| → in | `/state_estimation` | `nav_msgs/Odometry` |
| → in | `/registered_scan` | `sensor_msgs/PointCloud2` |
| → in | `/joy` | `sensor_msgs/Joy` |
| → in | `/map_clearing` | `std_msgs/Float32` |
| ← out | **`/terrain_map`** | `sensor_msgs/PointCloud2` |

---

### 3.5  `terrainAnalysisExt` — Extended Terrain Analysis

Maintains a larger-scale terrain map for the exploration planner.

| Direction | Topic | Message Type |
|-----------|-------|-------------|
| → in | `/state_estimation` | `nav_msgs/Odometry` |
| → in | `/registered_scan` | `sensor_msgs/PointCloud2` |
| → in | `/terrain_map` | `sensor_msgs/PointCloud2` |
| → in | `/joy` | `sensor_msgs/Joy` |
| → in | `/cloud_clearing` | `std_msgs/Float32` |
| ← out | **`/terrain_map_ext`** | `sensor_msgs/PointCloud2` |

---

### 3.6  `tare_planner` (TARE) — Exploration Planner

Technology-Aware Robot Exploration. Decides **where** to explore next.

| Direction | Topic | Message Type |
|-----------|-------|-------------|
| → in | `/state_estimation_at_scan` | `nav_msgs/Odometry` |
| → in | `/registered_scan` | `sensor_msgs/PointCloud2` |
| → in | `/terrain_map` | `sensor_msgs/PointCloud2` |
| → in | `/terrain_map_ext` | `sensor_msgs/PointCloud2` |
| → in | `/joy` | `sensor_msgs/Joy` |
| → in | `/navigation_boundary` | `geometry_msgs/PolygonStamped` |
| → in | `/start_exploration` | `std_msgs/Bool` |
| ← out | **`/way_point`** | `geometry_msgs/PointStamped` |
| ← out | `/global_path` | `nav_msgs/Path` |
| ← out | `/global_path_full` | `nav_msgs/Path` |
| ← out | `/local_path` | `nav_msgs/Path` |
| ← out | `/exploration_path` | `nav_msgs/Path` |
| ← out | `/exploration_finish` | `std_msgs/Bool` |
| ← out | `/runtime` | `std_msgs/Float32` |

**Config:** `indoor_small.yaml` (default). Publishes the next waypoint the robot should navigate toward.

---

### 3.7  `localPlanner` — Local Path Planner

Generates a collision-free **path** from current pose to the waypoint.

| Direction | Topic | Message Type |
|-----------|-------|-------------|
| → in | `/state_estimation` | `nav_msgs/Odometry` |
| → in | `/registered_scan` | `sensor_msgs/PointCloud2` |
| → in | `/terrain_map` | `sensor_msgs/PointCloud2` |
| → in | **`/way_point`** | `geometry_msgs/PointStamped` |
| → in | `/goal_pose` | `geometry_msgs/PoseStamped` |
| → in | `/joy` | `sensor_msgs/Joy` |
| → in | `/speed` | `std_msgs/Float32` |
| → in | `/navigation_boundary` | `geometry_msgs/PolygonStamped` |
| → in | `/added_obstacles` | `sensor_msgs/PointCloud2` |
| → in | `/check_obstacle` | `std_msgs/Bool` |
| → in | `/cancel_goal` | `std_msgs/Bool` |
| ← out | **`/path`** | `nav_msgs/Path` |
| ← out | `/slow_down` | `std_msgs/Int8` |
| ← out | `/goal_reached` | `std_msgs/Bool` |
| ← out | `/free_paths` | `sensor_msgs/PointCloud2` |

---

### 3.8  `pathFollower` — Velocity Controller

Follows the local path and produces velocity commands.

| Direction | Topic | Message Type |
|-----------|-------|-------------|
| → in | `/state_estimation` | `nav_msgs/Odometry` |
| → in | **`/path`** | `nav_msgs/Path` |
| → in | `/joy` | `sensor_msgs/Joy` |
| → in | `/speed` | `std_msgs/Float32` |
| → in | `/stop` | `std_msgs/Int8` |
| → in | `/slow_down` | `std_msgs/Int8` |
| ← out | **`/cmd_vel`** | **`geometry_msgs/TwistStamped`** |

> This is the **final output** of the autonomy stack — a velocity command (linear x/y + angular z).

---

### 3.9  `visualizationTools` — Map Accumulator (optional)

| Direction | Topic | Message Type |
|-----------|-------|-------------|
| → in | `/state_estimation` | `nav_msgs/Odometry` |
| → in | `/registered_scan` | `sensor_msgs/PointCloud2` |
| → in | `/runtime` | `std_msgs/Float32` |
| ← out | `/overall_map` | `sensor_msgs/PointCloud2` |
| ← out | `/explored_areas` | `sensor_msgs/PointCloud2` |
| ← out | `/trajectory` | `sensor_msgs/PointCloud2` |
| ← out | `/explored_volume` | `std_msgs/Float32` |
| ← out | `/traveling_distance` | `std_msgs/Float32` |

---

### 3.10  `ps3_joy` — Joystick (Deadman Switch)

| Direction | Topic | Message Type |
|-----------|-------|-------------|
| ← out | `/joy` | `sensor_msgs/Joy` |

Many nodes use `/joy` as an enable/override. If no joystick is connected, autonomous mode proceeds by default.

---

## 4  Command Bridge — Docker → Go1

These two nodes run **inside** Docker (or optionally on the host if ROS 2 is installed).

### 4.1  `cmd_vel_to_high_cmd` — Velocity Translator

| Direction | Topic | Message Type |
|-----------|-------|-------------|
| → in | `/cmd_vel` | `geometry_msgs/TwistStamped` |
| ← out | `/high_cmd` | `ros2_unitree_legged_msgs/HighCmd` |

**Behaviour:**
- Clamps velocities: `vx ≤ 0.4`, `vy ≤ 0.3`, `yaw_rate ≤ 0.8`
- Sets `mode = 2` (walk), `gait_type = 1` (trot), `foot_raise_height = 0.08`
- **Safety timeout**: if no `/cmd_vel` received for 0.5 s → publishes zero-velocity `HighCmd` (force-stand)

### 4.2  `ros2_udp HIGHLEVEL` — UDP Transport to Robot

| Direction | Topic / Dest | Type |
|-----------|-------------|------|
| → in | `/high_cmd` | `ros2_unitree_legged_msgs/HighCmd` |
| ← out | UDP to `192.168.123.161:8091` | binary HighCmd struct |
| → in | UDP from `192.168.123.161:8082` | binary HighState struct |
| ← out | `/high_state` | `ros2_unitree_legged_msgs/HighState` |

---

## 5  End-to-End Pipeline Diagram

```
 ┌─────────────────────────────────────────────────────────────────────────────────────┐
 │                           Docker: vector-autonomy-ros                               │
 │                                                                                     │
 │  ┌──────────────┐   /lidar/scan    ┌──────────────────┐   /registered_scan          │
 │  │ livox_ros_    │──(PointCloud2)──►│  arise_slam_     │──(PointCloud2)──►┐         │
 │  │ driver2       │                  │  mid360           │                  │         │
 │  │               │   /imu/data      │                  │   /state_         │         │
 │  │               │──(Imu)──────────►│  (feature_ext +  │   estimation      │         │
 │  └──────────────┘                   │   laser_map +    │──(Odometry)──►┐  │         │
 │                                     │   imu_preint)    │               │  │         │
 │                                     └──────────────────┘               │  │         │
 │                                                                        │  │         │
 │         ┌──────────────────────────────────────────────────────────────┘  │         │
 │         │                                                                 │         │
 │         ▼                              ▼                                  │         │
 │  ┌──────────────┐              ┌──────────────────┐                       │         │
 │  │ sensorScan   │              │ terrainAnalysis   │                      │         │
 │  │ Generation   │              │                   │                      │         │
 │  │              │              │  /state_estimation │                      │         │
 │  │ /state_est + │              │  /registered_scan  │                      │         │
 │  │ /reg_scan    │              │         │          │                      │         │
 │  │      │       │              │         ▼          │                      │         │
 │  │      ▼       │              │  /terrain_map      │                      │         │
 │  │ /state_est_  │              │  (PointCloud2)     │                      │         │
 │  │  at_scan     │              └────────┬───────────┘                      │         │
 │  │ /sensor_scan │                       │                                  │         │
 │  └──────┬───────┘                       │    ┌─────────────────────┐       │         │
 │         │                               │    │ terrainAnalysisExt  │       │         │
 │         │                               ├───►│                     │       │         │
 │         │                               │    │  /terrain_map_ext   │       │         │
 │         │                               │    └──────────┬──────────┘       │         │
 │         │                               │               │                  │         │
 │         ▼                               ▼               ▼                  │         │
 │  ┌──────────────────────────────────────────────────────────────────┐      │         │
 │  │                     tare_planner (TARE)                          │      │         │
 │  │  IN:  /state_estimation_at_scan, /registered_scan,               │      │         │
 │  │       /terrain_map, /terrain_map_ext                             │      │         │
 │  │  OUT: /way_point (PointStamped)                                  │      │         │
 │  └────────────────────────────┬─────────────────────────────────────┘      │         │
 │                               │                                            │         │
 │                               ▼ /way_point                                 │         │
 │  ┌──────────────────────────────────────────────────────────────────┐      │         │
 │  │                     localPlanner                                 │      │         │
 │  │  IN:  /state_estimation, /registered_scan, /terrain_map,         │      │         │
 │  │       /way_point                                                 │      │         │
 │  │  OUT: /path (Path)                                               │      │         │
 │  └────────────────────────────┬─────────────────────────────────────┘      │         │
 │                               │                                            │         │
 │                               ▼ /path                                      │         │
 │  ┌──────────────────────────────────────────────────────────────────┐      │         │
 │  │                     pathFollower                                  │      │         │
 │  │  IN:  /state_estimation, /path                                   │      │         │
 │  │  OUT: /cmd_vel (TwistStamped)   ◄── FINAL AUTONOMY OUTPUT       │      │         │
 │  └────────────────────────────┬─────────────────────────────────────┘      │         │
 │                               │                                            │         │
 └───────────────────────────────┼────────────────────────────────────────────┘         │
                                 │ /cmd_vel                                              │
                                 ▼                                                       │
 ┌───────────────────────────────────────────────────────────┐                           │
 │                 WSL: cmd_vel_to_high_cmd                   │                           │
 │  IN:  /cmd_vel (TwistStamped)                             │                           │
 │  OUT: /high_cmd (HighCmd) — clamped, mode=walk, gait=trot │                           │
 └───────────────────────────┬───────────────────────────────┘                           │
                             │ /high_cmd                                                  │
                             ▼                                                            │
 ┌───────────────────────────────────────────────────────────┐                           │
 │                 WSL: ros2_udp HIGHLEVEL                    │                           │
 │  IN:  /high_cmd (HighCmd)                                 │                           │
 │  OUT: UDP → 192.168.123.161:8091 (Go1 sport-mode board)  │                           │
 └───────────────────────────────────────────────────────────┘                           │
```

---

## 6  Key Topics Quick Reference

| Topic | Type | Producer | Consumer(s) |
|-------|------|----------|-------------|
| `/lidar/scan` | PointCloud2 | livox_ros_driver2 | arise_slam (featureExtraction) |
| `/imu/data` | Imu | livox_ros_driver2 | arise_slam (featureExtraction, imuPreintegration) |
| `/registered_scan` | PointCloud2 | arise_slam (laserMapping) | sensorScanGen, terrainAnalysis, terrainAnalysisExt, localPlanner, tare_planner, visualizationTools |
| `/state_estimation` | Odometry | arise_slam (imuPreintegration) | sensorScanGen, terrainAnalysis, terrainAnalysisExt, localPlanner, pathFollower, visualizationTools |
| `/state_estimation_at_scan` | Odometry | sensorScanGeneration | tare_planner |
| `/sensor_scan` | PointCloud2 | sensorScanGeneration | *(available for viz)* |
| `/terrain_map` | PointCloud2 | terrainAnalysis | terrainAnalysisExt, localPlanner, tare_planner |
| `/terrain_map_ext` | PointCloud2 | terrainAnalysisExt | tare_planner |
| `/way_point` | PointStamped | tare_planner | localPlanner |
| `/path` | Path | localPlanner | pathFollower |
| `/slow_down` | Int8 | localPlanner | pathFollower |
| `/cmd_vel` | **TwistStamped** | **pathFollower** | **cmd_vel_to_high_cmd** |
| `/high_cmd` | HighCmd | cmd_vel_to_high_cmd | ros2_udp HIGHLEVEL |
| `/high_state` | HighState | ros2_udp HIGHLEVEL | *(monitoring)* |
| `/joy` | Joy | ps3_joy | terrainAnalysis, terrainAnalysisExt, localPlanner, pathFollower, tare_planner |
| `/overall_map` | PointCloud2 | visualizationTools | Foxglove |
| `/explored_areas` | PointCloud2 | visualizationTools | Foxglove |
| `/trajectory` | PointCloud2 | visualizationTools | Foxglove |

---

## 7  Visualization & Goal Setting

### 7.1  Foxglove Studio (Web-based)

Foxglove Bridge (`ros-jazzy-foxglove-bridge`) runs in Docker on port **8765**.

**Recommended panels:**

| Panel | Topic | What you see |
|-------|-------|-------------|
| 3D | `/lidar/scan` | Raw point cloud |
| 3D | `/registered_scan` | SLAM-aligned cloud |
| 3D | `/terrain_map` | Traversability overlay |
| 3D | `/overall_map` | Accumulated exploration map |
| 3D | `/path` | Local planned path |
| 3D | `/exploration_path` | TARE global exploration path |
| 3D | `/free_paths` | Candidate paths from localPlanner |
| Raw Messages | `/cmd_vel` | Velocity commands |
| Raw Messages | `/state_estimation` | Robot pose |
| Plot | `/explored_volume` | Exploration progress |

**Connection:** Desktop Foxglove → `ws://localhost:8765`

### 7.2  RViz (Native Desktop)

RViz provides a native visualization experience with custom click-to-navigate tools. Launch RViz from Terminal 5 (see Section 8).

**Recommended displays:**
| Display Type | Topic | Description |
|--------------|-------|-------------|
| PointCloud2 | `/registered_scan` | SLAM-aligned point cloud |
| PointCloud2 | `/overall_map` | Accumulated map |
| PointCloud2 | `/terrain_map` | Traversability classification |
| PointCloud2 | `/free_paths` | Candidate navigation paths |
| Path | `/path` | Current planned path |
| Axes | `/vehicle` frame | Robot position/orientation |

**Custom tools (from workspace plugins):**
| Tool | Shortcut | Publishes To | Use Case |
|------|----------|--------------|----------|
| Goalpoint | `w` | `/goal_pose` or `/goal_point` | Route planning (FAR/PCT) |
| Waypoint | `w` | `/way_point` | Direct waypoint to localPlanner |

### 7.3  Sending Goal Waypoints (Route Planner Mode)

When using the **route planner** (`system_real_robot_with_route_planner.launch`), you give the robot a destination.

**Goal topics by planner:**

| Planner | Topic | Type | Notes |
|---------|-------|------|-------|
| **FAR** | `/goal_point` | `PointStamped` | Position only |
| **FAR** | `/goal_pose` | `PoseStamped` | Position + orientation |
| **PCT** | `/goal_pose` | `PoseStamped` | ⚠️ Only topic PCT listens to |
| **PCT** | `/clicked_point` | `PointStamped` | Alternative (RViz Publish Point tool) |

> 💡 **Tip:** Use `/goal_pose` for compatibility with both FAR and PCT planners.

#### Method 1: Foxglove Publish Panel (type coordinates manually)

1. Add panel → select **发布 (Publish)**
2. In the **Topic** field, **type manually**: `/goal_pose` (works with both FAR and PCT)
   - Or `/goal_point` for FAR planner only
3. Set **Message schema** to: `geometry_msgs/msg/PoseStamped`
4. In the message editor, enter:
   ```json
   {
     "header": { "frame_id": "map" },
     "pose": { 
       "position": { "x": 5.0, "y": 3.0, "z": 0.0 },
       "orientation": { "w": 1.0 }
     }
   }
   ```
5. Click **Publish** to send the robot to that (x, y) coordinate

#### Method 2: Foxglove Click-on-Map in 3D Panel

1. Open the **三维 (3D)** panel settings (⚙️ gear icon)
2. Scroll to the **Publish** section
3. Set **Type** to `Pose estimate` and **Topic** to `/goal_pose`
4. Click on the map in the 3D view to place a goal — the robot will navigate there

#### Method 3: RViz with Goalpoint Tool (click-to-navigate) ⭐ Recommended

The workspace includes custom RViz plugins for click-to-navigate functionality.

1. **Start RViz in a new Docker terminal** (see Terminal 5 in Section 8)
2. In RViz, select the **Goalpoint** tool from the toolbar (keyboard shortcut: `w`)
3. Click on the 3D map view to set a goal point
4. The robot will navigate to that location

**Available RViz tools:**
| Tool | Topic Published | Description |
|------|-----------------|-------------|
| **Goalpoint** | `/goal_pose` or `/goal_point` | For route planning (FAR/PCT) - sends destination goal |
| **Waypoint** | `/way_point` | For immediate waypoints - sends directly to localPlanner |

The Goalpoint tool has a configurable property "Use Pose Topic":
- **true** (default): Publishes `PoseStamped` to `/goal_pose` (includes orientation)
- **false**: Publishes `PointStamped` to `/goal_point` (position only)

#### Method 4: Terminal (no GUI needed)

```bash
# From Docker container with RMW set:
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Works with BOTH FAR and PCT planners:
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 3.0, z: 0.0}, orientation: {w: 1.0}}}"

# FAR planner only (does NOT work with PCT):
# ros2 topic pub --once /goal_point geometry_msgs/msg/PointStamped \
#   "{header: {frame_id: 'map'}, point: {x: 5.0, y: 3.0, z: 0.0}}"
```

> **Note:** In **exploration mode** (TARE planner), waypoints are generated automatically — no manual goal is needed.

---

## 8  Launch Sequence (5 Terminals)

### Prerequisites – Network Setup

Before launching, configure your network interfaces:

```bash
# 1. Set static IP for LiDAR Ethernet adapter (find your interface name with: ip link)
sudo ip addr add 192.168.1.50/24 dev <ethernet_interface>
sudo ip link set <ethernet_interface> up

# 2. Connect to Go1 WiFi network (SSID: Unitree_GoXXXXXX)
#    Your PC will get IP 192.168.123.x via DHCP

# 3. Verify connectivity
ping 192.168.1.1xx    # LiDAR IP (check your MID360_config.json)
ping 192.168.123.161  # Go1 robot
```

### Which Terminals Do I Need?

| Terminal | Purpose | Required? |
|----------|---------|-----------|
| **1** | LiDAR + SLAM + Planner | ✅ **Always** |
| **2** | Foxglove Bridge | ⚪ Only for Foxglove visualization |
| **3** | cmd_vel → high_cmd bridge | ✅ **Always** |
| **4** | UDP transport to Go1 | ✅ **Always** |
| **5** | RViz visualization | ⚪ Only for RViz click-to-navigate |

**Minimum setup:** Terminals 1, 3, 4 (+ Terminal 5 for RViz OR Terminal 2 for Foxglove)

---

#### Terminal 1 – Docker: LiDAR + SLAM + Planner (REQUIRED)

```bash
cd ~/verified_autonomy/docker
./shell.sh

# Inside container
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash


# ═══════════════════════════════════════════════════════════════════════════════
# Option A: Exploration planner (autonomous exploration + mapping)
#           Upstream equivalent: ./system_real_robot_with_exploration_planner.sh
# ═══════════════════════════════════════════════════════════════════════════════
ros2 launch vehicle_simulator system_real_robot_with_exploration_planner.launch.py

# ─────────────────────────────────────────────────────────────────────────────────
# SAVING THE MAP (after exploration is complete)
# ─────────────────────────────────────────────────────────────────────────────────
# In a new Docker terminal, save the accumulated point cloud:
#
#   # Method 1: Save via SLAM service (if available)
#   ros2 service call /arise_slam/save_map std_srvs/srv/Trigger
#
#   # Method 2: Save point cloud directly from /overall_map topic
#   cd /workspace/map
#   ros2 run pcl_ros pointcloud_to_pcd --ros-args -r input:=/overall_map
#
# Maps are saved to: /workspace/map/ by default
# Recommended naming: /workspace/map/my_environment.pcd
# ─────────────────────────────────────────────────────────────────────────────────

# ═══════════════════════════════════════════════════════════════════════════════
# Option B: Route planner with FAR (navigate to waypoints using saved map)
#           Upstream equivalent: ./system_real_robot_with_route_planner.sh
# ═══════════════════════════════════════════════════════════════════════════════
#
# PREREQUISITES:
#   - You have a saved .pcd map from Option A exploration
#   - Stop the exploration launch before starting route planning
#
# WORKFLOW:
#   1. Set the map path (no file extension - it's a prefix):
export MAP_PATH=/workspace/map/my_environment

#   2. Launch with FAR planner in localization mode:
ros2 launch vehicle_simulator system_real_robot_with_route_planner.launch.py \
  autonomyMode:=true

#   3. Send waypoints via Foxglove (/goal_point) or terminal - see Section 7
#
# Note: SLAM loads "$MAP_PATH.pcd" for localization (no new mapping)

# ═══════════════════════════════════════════════════════════════════════════════
# Option C: Route planner with PCT (navigate to waypoints using saved map)
#           Upstream equivalent: ros2 launch ... use_pct_planner:=true
# ═══════════════════════════════════════════════════════════════════════════════
#
# PREREQUISITES:
#   - You have a saved .pcd map from Option A exploration
#   - PCT planner also requires a TOMOGRAM file (.pickle) for 3D navigation
#   - Stop the exploration launch before starting route planning
#
# WORKFLOW:
#   1. First, generate the tomogram from your PCD map (one-time step):
#      cd /workspace
#      source install/setup.bash
#      ros2 run pct_planner pcd_to_tomogram.py /workspace/map/my_environment.pcd \
#        -o /workspace/map/my_environment_tomogram.pickle
#
#   2. Set the map path (prefix without extension):
export MAP_PATH=/workspace/map/my_environment

#   3. Launch with PCT planner in localization mode:
ros2 launch vehicle_simulator system_real_robot_with_route_planner.launch.py \
  use_pct_planner:=true \
  autonomyMode:=true

#   4. Send waypoints via Foxglove (/goal_point) or terminal - see Section 7
#
# Note: PCT planner loads both "$MAP_PATH.pcd" (SLAM) and "$MAP_PATH_tomogram.pickle"
#
# PCT PLANNER DEPENDENCIES (install inside Docker if not present):
#   pip3 install cupy-cuda11x open3d numpy
#   cd /workspace/src/route_planner/PCT_planner/pct_planner/planner
#   ./build_thirdparty.sh && ./build.sh

# ═══════════════════════════════════════════════════════════════════════════════
# Option D: Base autonomy only (manual joystick control with obstacle avoidance)
#           Upstream equivalent: ./system_real_robot.sh
# ═══════════════════════════════════════════════════════════════════════════════
ros2 launch vehicle_simulator system_real_robot.launch.py

# No global planner - robot does NOT autonomously explore or navigate to goals
# Use joystick or Foxglove teleop panel to drive the robot manually
# Robot still avoids obstacles automatically (smart joystick mode)
```

#### Terminal 2 – Docker: Foxglove Bridge (OPTIONAL)

```bash
cd ~/verified_autonomy/docker
./shell.sh

# Inside container
source /opt/ros/jazzy/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

Open Foxglove Studio and connect to `ws://localhost:8765` to visualize:
- `/overall_map` – Accumulated SLAM map (PointCloud2)
- `/trajectory` – Robot trajectory (PointCloud2)
- `/explored_areas` – Explored areas (PointCloud2)
- `/cmd_vel` – Velocity commands

#### Terminal 3 – Docker: cmd_vel → high_cmd Bridge (REQUIRED)

```bash
cd ~/verified_autonomy/docker
./shell.sh

# Inside container – build unitree workspace if not already built
cd /unitree_ws
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
colcon build --symlink-install
source install/setup.bash

# Run the velocity translator
python3 /scripts/cmd_vel_to_high_cmd.py
```

#### Terminal 4 – Docker: UDP Transport to Go1 (REQUIRED)

```bash
cd ~/verified_autonomy/docker
./shell.sh

# Inside container
cd /unitree_ws
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source install/setup.bash

# Run the UDP bridge to Go1
export LD_LIBRARY_PATH=/unitree_ws/src/unitree_legged_sdk/lib:$LD_LIBRARY_PATH
ros2 run unitree_legged_real ros2_udp HIGHLEVEL
```

#### Terminal 5 – Docker: RViz with Goalpoint Tool (OPTIONAL – for Route Planning)

```bash
cd ~/verified_autonomy/docker
./shell.sh

# Inside container
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash

# Launch RViz with the FAR planner's pre-configured layout
ros2 run rviz2 rviz2 -d /workspace/src/route_planner/far_planner/rviz/default.rviz
```

**Using RViz for route planning (click-to-navigate):**

1. Select the **Goalpoint** tool from the toolbar (keyboard shortcut: `w`)
2. Click on the 3D map view to set a navigation goal
3. The planner computes a path and the robot navigates there
4. Use the **TeleopPanel** (bottom right) for manual joystick control

**How to tell which planner is running:**

The planner is determined by your **Terminal 1 launch command**, not by RViz:

| Launch Command (Terminal 1) | Active Planner |
|-----------------------------|----------------|
| `ros2 launch vehicle_simulator system_real_robot_with_route_planner.launch.py` | **FAR** |
| `ros2 launch vehicle_simulator system_real_robot_with_route_planner.launch.py use_pct_planner:=true` | **PCT** |

**Quick check via ROS topics (in any Docker terminal):**
```bash
# If this topic exists → PCT planner is running
ros2 topic list | grep tomogram

# If this topic exists → FAR planner is running  
ros2 topic list | grep viz_graph_topic
```

**Important: FAR vs PCT planner topic differences:**

| Planner | Goal Topic | RViz Tool Setting |
|---------|------------|-------------------|
| **FAR** | `/goal_pose` or `/goal_point` | Goalpoint tool with "Use Pose Topic" = true or false |
| **PCT** | `/goal_pose` only | Goalpoint tool with **"Use Pose Topic" = true** (default) |

> ⚠️ **PCT planner does NOT listen on `/goal_point`** — make sure "Use Pose Topic" is checked (true) in the Tool Properties panel when using PCT planner.

**Tool Properties (in Tool Properties panel → expand Goalpoint):**
- **Topic**: The base topic name (default: "goalpoint")
- **Use Pose Topic**: ✅ Keep **true** for PCT planner, can be true/false for FAR planner

---

## 9  Robot Configuration (Go1 + LiDAR)

### 9.1  Sensor Mounting Offsets

The LiDAR position relative to the robot body must be configured correctly for proper obstacle detection and collision avoidance.

**Configuration file:** `/workspace/src/base_autonomy/local_planner/config/unitree/unitree_go1.yaml`

```yaml
# Key parameters to adjust for your LiDAR mounting position:
sensorMountingOffsets:
  ros__parameters:
    sensorOffsetX: 0.05    # Forward offset from robot center (m)
    sensorOffsetY: 0.0     # Lateral offset from robot center (m)
    sensorOffsetZ: -0.45   # Height offset (negative if LiDAR above base)

localPlanner:
  ros__parameters:
    vehicleLength: 0.70    # Robot length + safety margin (m)
    vehicleWidth: 0.35     # Robot width + safety margin (m)
```

### 9.2  Blind Zones (Prevent Self-Detection)

The SLAM feature extraction filters out points from the robot body itself:

```yaml
feature_extraction_node:
  ros__parameters:
    blindFront: 0.35       # Ignore points in front of sensor up to this distance
    blindBack: -0.35       # Ignore points behind sensor
    blindLeft: 0.20        # Ignore points to the left
    blindRight: -0.20      # Ignore points to the right
    blindDiskRadius: 0.25  # Cylindrical exclusion zone radius
```

### 9.3  Terrain Analysis Parameters

**File:** `/workspace/src/base_autonomy/terrain_analysis/launch/terrain_analysis.launch`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `vehicleHeight` | 1.5 | Height above ground to consider clear (m) - reduce for indoor |
| `obstacleHeightThre` | 0.1 | Minimum height to consider as obstacle (m) |
| `minRelZ` | -1.5 | Minimum Z relative to robot to consider points |
| `maxRelZ` | 0.3 | Maximum Z relative to robot to consider points |
| `minDyObsDis` | 0.14 | Minimum distance for dynamic obstacle filtering |

### 9.4  Setting the Robot Config

To use the Go1 configuration, set the environment variable before launching:

```bash
# Inside Docker container, before launching Terminal 1:
export ROBOT_CONFIG_PATH=unitree/unitree_go1
```

Or pass it as a launch argument:
```bash
ros2 launch vehicle_simulator system_real_robot_with_route_planner.launch.py \
  robot_config:=unitree/unitree_go1
```

---

## 10  Safety Notes

| Mechanism | Details |
|-----------|---------|
| **cmd_vel timeout** | `cmd_vel_to_high_cmd` stops the robot if no command arrives for 0.5 s |
| **Velocity clamping** | `vx ≤ 0.4 m/s`, `vy ≤ 0.3 m/s`, `yaw ≤ 0.8 rad/s` |
| **Joystick override** | `/joy` is read by localPlanner, pathFollower, terrainAnalysis — joystick always overrides |
| **Emergency stop** | Power off the Go1 or disconnect WiFi (timeout kicks in) |
| **SLAM health** | `/state_estimation_health` (Bool) — if false, SLAM may be degraded |

---

*Generated from source-level analysis of the `vector-autonomy` workspace.*

