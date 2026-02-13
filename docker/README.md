# Vector Autonomy Stack - Docker Environment

This directory contains Docker configuration for running the Vector Autonomy Stack in an isolated Ubuntu 24.04 + ROS 2 Jazzy environment.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Quick Start](#quick-start)
3. [Detailed Setup Guide](#detailed-setup-guide)
4. [Building the Autonomy Stack](#building-the-autonomy-stack)
5. [Running the Simulation](#running-the-simulation)
6. [Switching Environments](#switching-environments)

---

## Prerequisites

### 1. Install Docker

Docker allows you to run applications in isolated containers. Install it with:

```bash
# Update package list
sudo apt update

# Install Docker
sudo apt install -y docker.io docker-compose-v2

# Add yourself to the docker group (so you don't need sudo)
sudo usermod -aG docker $USER

# IMPORTANT: Log out and log back in for group changes to take effect
# Or run: newgrp docker
```

Verify Docker is working:

```bash
docker run hello-world
```

### 2. Install X11 utilities (for GUI support)

```bash
sudo apt install -y x11-xserver-utils
```

### 3. Download Unity Environments (for simulation)

The simulation requires Unity environment files (~5.6GB total). Download from:

**[Google Drive - Unity Environments](https://drive.google.com/drive/folders/1G1JYkccvoSlxyySuTlPfvmrWoJUO8oSs?usp=sharing)**

**Note:** Google Drive automatically splits large folder downloads into multiple zip files (e.g., `unity_env_models-001.zip`, `002.zip`, `003.zip`). This is normal - each zip contains different environments with no overlap. Extract all of them.

**Extract and set up:**

```bash
cd vector_navigation_stack/src/base_autonomy/vehicle_simulator/mesh/unity/

# Extract all downloaded zips (they contain nested environment zips)
unzip ~/Downloads/unity_env_models-*.zip

# Pick an environment to use (e.g., office_1)
unzip unity_env_models/office_1.zip
mv office_1/* .
rmdir office_1

# Make Unity executable
chmod +x environment/Model.x86_64
```

After setup, you should have:
```
mesh/unity/
├── environment/
│   └── Model.x86_64    # The Unity executable
├── map.ply
├── map.jpg
├── object_list.txt
├── traversable_area.ply
├── render.jpg
└── unity_env_models/   # Other environments (zip files)
```

---

## Quick Start

```bash
# 1. Navigate to the docker directory
cd <path-to-repo>/docker

# 2. Build the Docker image (one-time)
./build.sh

# 3. Start the container
./run.sh

# 4. Open a terminal inside the container
./shell.sh

# 5. Inside the container, build the autonomy stack for simulation
cb-sim

# 6. Run the simulation (after Unity environment is set up)
cd /workspace
./system_simulation.sh
```

---

## Detailed Setup Guide

### Step 1: Build the Docker Image

The Docker image contains Ubuntu 24.04 with ROS 2 Jazzy and all dependencies pre-installed.

```bash
cd <path-to-repo>/docker
./build.sh
```

**What happens:**
- Downloads Ubuntu 24.04 base image
- Installs ROS 2 Jazzy Desktop Full
- Installs all required dependencies (PCL, OpenCV, etc.)
- Creates a user inside the container that matches your host user

**Expected time:** 10-15 minutes on first build (downloads ~3GB)

### Step 2: Start the Container

```bash
./run.sh
```

**What happens:**
- Starts the Docker container in the background
- Mounts your `vector_navigation_stack` folder into the container at `/workspace`
- Sets up X11 forwarding so GUI apps (RViz, Unity) work
- Enables access to joystick/gamepad devices

### Step 3: Enter the Container

```bash
./shell.sh
```

This opens a bash terminal inside the container. You'll see a prompt like:
```
username@hostname:/workspace$
```

**Key points:**
- `/workspace` inside the container = `vector_navigation_stack` on your host
- Any files you edit are saved to your host (they're the same files!)
- ROS 2 Jazzy is already sourced and ready to use

### Step 4: Build the Autonomy Stack

Inside the container:

```bash
# For simulation (skips SLAM and lidar driver - faster build)
cb-sim

# Or for full build (if you need SLAM for real robot use)
cb
```

**What happens:**
- `colcon build` compiles all ROS 2 packages (22 packages for simulation)
- Creates `build/`, `install/`, and `log/` directories
- These directories appear on your host system too

**Expected time:** 5-10 minutes

**Note:** If the build runs out of memory (compiler killed), use:
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-skip arise_slam_mid360 arise_slam_mid360_msgs livox_ros_driver2 \
  --parallel-workers 2 --executor sequential
```

### Step 5: Run the Simulation

**First, ensure Unity environment is set up** (see Prerequisites).

Inside the container:

```bash
cd /workspace
./system_simulation.sh
```

**What happens:**
1. Launches the Unity simulator (3D environment window)
2. Starts ROS 2 nodes (navigation, terrain analysis, etc.)
3. Opens RViz (visualization window)

### Step 6: Stop the Simulation

```bash
# Press Ctrl+C in the terminal running the simulation
# Or from another terminal:
pkill -f ros2
```

---

## Building the Autonomy Stack

### Simulation-Only Build (Recommended for Testing)

```bash
cb-sim
```

This skips SLAM and lidar driver packages, which are only needed for real robots.

### Full Build

```bash
cb
```

Builds everything including SLAM. Requires additional dependencies to be built first (see main README.md for Sophus, Ceres, GTSAM installation).

### Single Package Build

```bash
cd /workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select <package_name>
```

### Clean Build

```bash
cd /workspace
rm -rf build install log
cb-sim  # or cb for full build
```

---

## Running the Simulation

### Base Autonomy

```bash
./system_simulation.sh
```

### With Route Planner (FAR Planner)

```bash
./system_simulation_with_route_planner.sh
```

### With Exploration Planner (TARE)

```bash
./system_simulation_with_exploration_planner.sh
```

### Stopping the Simulation

| Method | Command |
|--------|---------|
| In terminal | `Ctrl+C` |
| From another terminal | `pkill -f ros2` |
| Kill everything | `pkill -f 'ros2\|Model.x86_64\|rviz'` |

### Using a Joystick

If you have a gamepad connected:
1. Connect it to your host computer
2. The container has access to `/dev/input` devices
3. Joystick input should work automatically

---

## Switching Environments

18 environments are available in `mesh/unity/unity_env_models/`:

| Environment | Size | Description |
|-------------|------|-------------|
| office_1, office_2 | ~155MB | Small office rooms |
| office_building_1 | ~1GB | Large office building |
| office_building_2 | ~730MB | Large office building |
| livingroom_1-4 | ~170MB | Living room scenes |
| home_building_1-2 | ~400-570MB | Full home buildings |
| hotel_room_1-2 | ~165MB | Hotel room scenes |
| loft | ~226MB | Loft apartment |
| studio | ~145MB | Studio apartment |
| arabic_room | ~158MB | Themed room |
| chinese_room | ~155MB | Themed room |
| japanese_room | ~174MB | Themed room |

### To Switch Environments

```bash
cd vector_navigation_stack/src/base_autonomy/vehicle_simulator/mesh/unity

# Remove current environment files
rm -rf environment map.ply map.jpg object_list.txt traversable_area.ply render.jpg readme.txt

# Extract new environment
unzip unity_env_models/<environment_name>.zip
mv <environment_name>/* .
rmdir <environment_name>

# Make executable
chmod +x environment/Model.x86_64
```

Example - switch to `home_building_1`:
```bash
rm -rf environment map.ply map.jpg object_list.txt traversable_area.ply render.jpg readme.txt
unzip unity_env_models/home_building_1.zip
mv home_building_1/* . && rmdir home_building_1
chmod +x environment/Model.x86_64
```

---

## Helper Script Reference

| Script | Description |
|--------|-------------|
| `./build.sh` | Build the Docker image |
| `./run.sh` | Start the container (runs in background) |
| `./shell.sh` | Open a terminal in the container |
| `./stop.sh` | Stop the container |
| `./clean.sh` | Remove container, image, and volumes |

## Container Aliases

Inside the container, these aliases are available:

| Alias | Command | Description |
|-------|---------|-------------|
| `cb` | `colcon build ...` | Full build |
| `cb-sim` | `colcon build ... --packages-skip ...` | Simulation-only build |
| `ws` | `cd /workspace` | Go to workspace |

---