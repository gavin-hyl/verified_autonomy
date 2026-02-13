# System Flow – LiDAR & Autonomous Exploration Stack

> Full data-flow reference for the Unitree Go1 + Livox Mid-360 autonomous exploration system.
> All nodes run inside the **`vector-autonomy-ros`** Docker container (ROS 2 Jazzy) unless marked **(WSL)**.

---

## 1  Physical Topology

```
┌──────────────┐  Ethernet (USB-C adapter)  ┌─────────────────────┐
│ Livox Mid-360│ ◄──────────────────────────►│  Windows PC          │
│ 192.168.1.127│     192.168.1.50/24         │  (WSL2 + Docker)     │
└──────────────┘                             │                      │
                                             │  Docker: SLAM +      │
                                             │  Exploration Planner  │
                                             │                      │
                                             │  WSL: cmd bridge +   │
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
| LiDAR ↔ PC | UDP (Livox SDK) | Ethernet 3 – ASIX AX88179 adapter, static IP 192.168.1.50 |
| PC Docker ↔ WSL | ROS 2 DDS (shared `--net=host` + mirrored WSL networking) | localhost / loopback |
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

## 4  Command Bridge — Docker → Go1  **(WSL)**

These two nodes run **outside** Docker, in WSL where the Unitree message packages are built.

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

## 7  Foxglove Visualization

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

---

## 8  Launch Sequence (4 Terminals)

#### Terminal 1 – Docker: LiDAR + SLAM + Exploration

```bash
cd ~/verified_autonomy/docker
./shell.sh

# Inside container
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
ros2 launch vehicle_simulator system_real_robot_with_exploration_planner.launch
```

#### Terminal 2 – Docker: Foxglove Bridge (second shell)

```bash
cd ~/verified_autonomy/docker
./shell.sh

# Inside container
source /opt/ros/jazzy/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

#### Terminal 3 – WSL: cmd_vel → high_cmd bridge

```bash
source /opt/ros/jazzy/setup.bash
source ~/verified_autonomy/ros2_unitree_ws/install/setup.bash
python3 ~/verified_autonomy/cmd_vel_to_high_cmd.py
```

#### Terminal 4 – WSL: UDP transport to Go1

```bash
cd ~/verified_autonomy/ros2_unitree_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=~/verified_autonomy/ros2_unitree_ws/src/unitree_legged_sdk/lib:$LD_LIBRARY_PATH
ros2 run unitree_legged_real ros2_udp HIGHLEVEL
```

---

## 9  Safety Notes

| Mechanism | Details |
|-----------|---------|
| **cmd_vel timeout** | `cmd_vel_to_high_cmd` stops the robot if no command arrives for 0.5 s |
| **Velocity clamping** | `vx ≤ 0.4 m/s`, `vy ≤ 0.3 m/s`, `yaw ≤ 0.8 rad/s` |
| **Joystick override** | `/joy` is read by localPlanner, pathFollower, terrainAnalysis — joystick always overrides |
| **Emergency stop** | Power off the Go1 or disconnect WiFi (timeout kicks in) |
| **SLAM health** | `/state_estimation_health` (Bool) — if false, SLAM may be degraded |

---

*Generated from source-level analysis of the `vector-autonomy` workspace.*
