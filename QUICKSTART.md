# Quickstart

Setup guide for the verified autonomy stack. Assumes the repo (with submodules) is already cloned.

## Prerequisites

- Docker with NVIDIA Container Toolkit (`nvidia-ctk`)
- NVIDIA GPU with drivers installed
- Verify: `docker run --rm --gpus all nvidia/cuda:12.6.3-base-ubuntu24.04 nvidia-smi`

## 1. Build and Start Docker

```bash
cd docker
./build.sh        # One-time (~10-15 min)
./run.sh          # Start container
./shell.sh        # Enter container
```

## 2. Build

```bash
# Simulation only (skips SLAM + lidar driver):
cb-sim

# Full build (includes SLAM + lidar — needed for real robot):
cb
```

## 3. Run Simulation

```bash
cd /workspace
./system_simulation.sh                          # Base autonomy
./system_simulation_with_route_planner.sh       # + FAR planner
./system_simulation_with_exploration_planner.sh # + TARE exploration

# With PCT planner instead of FAR:
USE_PCT_PLANNER=true ./system_simulation_with_route_planner.sh
```

Click in RViz to set waypoints (`/way_point`).

## 4. Run on Real Robot (Unitree Go1)

### Network Setup (on host, not in Docker)

Two connections are needed:

| Device | Host IP | Device IP |
|--------|---------|-----------|
| Robot  | 192.168.123.162 | 192.168.123.161 |
| Lidar (Mid-360) | 192.168.1.5 | 192.168.1.127 |

```bash
# Configure robot network (on host):
./ros2_unitree_ws/setup_go1.sh network <ethernet-interface>

# Verify:
ping 192.168.123.161   # robot
ping 192.168.1.127     # lidar
```

### Launch (all inside Docker)

**Terminal 1** — UDP bridge to robot:
```bash
cd /unitree_ws
./setup_go1.sh build   # First time only
./setup_go1.sh run
```

**Terminal 2** — Autonomy stack:
```bash
cd /workspace
./system_real_robot.sh
# or: ./system_real_robot_with_route_planner.sh
# or: USE_PCT_PLANNER=true ./system_real_robot_with_route_planner.sh
```

**Terminal 3** — Robot commands:
```bash
source /unitree_ws/install/setup.bash
python3 /unitree_ws/scripts/go1_cmd.py stand_up
python3 /unitree_ws/scripts/go1_cmd.py walk 0.3 0 0    # fwd vel, lateral vel, yaw rate
python3 /unitree_ws/scripts/go1_control.py              # interactive keyboard
```

## Key ROS Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/way_point` | PointStamped | Navigation goal (click in RViz) |
| `/cmd_vel` | TwistStamped | Velocity output |
| `/state_estimation` | Odometry | Robot pose from SLAM |
| `/stop` | Int8 | Safety stop (1=stop, 2=full stop) |
| `/high_cmd` | HighCmd | Direct commands to Go1 |
| `/high_state` | HighState | Go1 state feedback |

## Useful Aliases (inside container)

| Alias | Description |
|-------|-------------|
| `cb` | Full colcon build |
| `cb-sim` | Build without SLAM/lidar packages |
| `ws` | `cd /workspace` |

## Switching Unity Environments

```bash
cd /workspace/src/base_autonomy/vehicle_simulator/mesh/unity
rm -rf environment map.ply map.jpg object_list.txt traversable_area.ply render.jpg
unzip unity_env_models/<env_name>.zip
mv <env_name>/* . && rmdir <env_name>
chmod +x environment/Model.x86_64
```
