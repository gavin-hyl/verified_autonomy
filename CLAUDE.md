# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

This is a verified autonomy stack for robotic navigation, supporting both simulation and real robot deployment (Unitree Go 1). The stack is built on ROS 2 Jazzy and runs inside a Docker container.

Main components:
- **vector_navigation_stack/**: Core autonomy stack (SLAM, local planner, terrain analysis, route/exploration planners)
- **docker/**: Docker environment configuration for Ubuntu 24.04 + ROS 2 Jazzy
- **ros2_unitree_ws/**: Unitree Go 1 ROS 2 workspace (SDK and real robot interface)

## Development Environment

All development happens inside the Docker container. The host's `vector_navigation_stack/` is mounted at `/workspace` inside the container.

### Docker Commands (from `docker/` directory)
```bash
./build.sh      # Build Docker image (one-time, ~10-15 min)
./run.sh        # Start container in background
./shell.sh      # Enter container shell
./stop.sh       # Stop container
./clean.sh      # Remove container, image, and volumes
```

### Build Commands (inside container)
```bash
cb-sim          # Simulation-only build (skips SLAM and lidar driver)
cb              # Full build (includes SLAM for real robot)

# Single package build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select <package_name>

# Clean build
rm -rf build install log && cb-sim
```

### System Launch Scripts (inside container at `/workspace`)
**Simulation:**
- `./system_simulation.sh` - Base autonomy
- `./system_simulation_with_route_planner.sh` - With FAR/PCT planner
- `./system_simulation_with_exploration_planner.sh` - With TARE planner

**Real Robot:**
- `./system_real_robot.sh`
- `./system_real_robot_with_route_planner.sh`
- `./system_real_robot_with_exploration_planner.sh`

## Code Architecture

### Package Structure (in vector_navigation_stack/src/)
- **base_autonomy/**: Core navigation (local_planner, terrain_analysis, vehicle_simulator)
- **slam/**: SLAM with Mid-360 lidar (arise_slam_mid360) and dependencies (ceres, gtsam)
- **route_planner/**: FAR planner and PCT planner for global navigation
- **exploration_planner/**: TARE planner for autonomous exploration
- **utilities/**: Drivers, teleop, RViz plugins, domain_bridge

### Key ROS Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/way_point` | `geometry_msgs/PointStamped` | Navigation goal |
| `/cancel_goal` | `std_msgs/Bool` | Cancel current goal |
| `/state_estimation` | `nav_msgs/Odometry` | Robot pose from SLAM |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | Velocity commands |
| `/stop` | `std_msgs/Int8` | Safety stop (1=stop, 2=full stop) |

### Environment Variables
```bash
export ROBOT_CONFIG_PATH="mechanum_drive"    # Robot configuration
export MAP_PATH="/path/to/map"               # Enable localization mode
```

## Unitree Go 1 Real Robot

See [GO1_CONTROL.md](GO1_CONTROL.md) for comprehensive control documentation (includes Windows/macOS/Linux instructions).

**Quick Start:**
```bash
# 1. On host: Configure network (Linux)
./ros2_unitree_ws/setup_go1.sh network enp118s0

# 2. Start Docker container
cd docker && ./run.sh && ./shell.sh

# 3. Inside container: Build and run
cd /unitree_ws
./setup_go1.sh build
./setup_go1.sh run

# 4. In another container terminal: Send commands
source /unitree_ws/install/setup.bash
python3 /unitree_ws/scripts/go1_cmd.py stand_up
python3 /unitree_ws/scripts/go1_cmd.py walk 0.3 0 0
python3 /unitree_ws/scripts/go1_control.py  # Interactive keyboard
```

**Key Topics:**
- `/high_cmd` - Send commands to robot (HighCmd message)
- `/high_state` - Robot state feedback (HighState message)

## Unity Simulation Environment

Unity environments go in `vector_navigation_stack/src/base_autonomy/vehicle_simulator/mesh/unity/`. The `environment/Model.x86_64` executable must exist and be marked executable.

To switch environments:
```bash
cd vector_navigation_stack/src/base_autonomy/vehicle_simulator/mesh/unity
rm -rf environment map.ply map.jpg object_list.txt traversable_area.ply render.jpg
unzip unity_env_models/<env_name>.zip
mv <env_name>/* . && rmdir <env_name>
chmod +x environment/Model.x86_64
```
