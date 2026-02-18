# Unitree Go1 High-Level Control Guide

Control your Unitree Go1 robot using ROS 2 commands through a Docker-based development environment. Works on Linux, macOS, and Windows.

## Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                         HOST MACHINE                              │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │                    DOCKER CONTAINER                         │  │
│  │                                                             │  │
│  │  ┌─────────────┐     ┌─────────────┐     ┌─────────────┐   │  │
│  │  │ Your Node   │────▶│  ros2_udp   │────▶│    UDP      │   │  │
│  │  │ /high_cmd   │     │   bridge    │     │  packets    │   │  │
│  │  └─────────────┘     └─────────────┘     └──────┬──────┘   │  │
│  │                            ▲                     │          │  │
│  │  ┌─────────────┐     ┌─────┴───────┐            │          │  │
│  │  │ /high_state │◀────│  Feedback   │            │          │  │
│  │  └─────────────┘     └─────────────┘            │          │  │
│  └─────────────────────────────────────────────────│──────────┘  │
│                                                     │             │
│  Ethernet (192.168.123.162) ◀───────────────────────┘             │
└───────────────────────────────────────────────────────────────────┘
                              │
                              │ UDP (ports 8082, 8090)
                              ▼
                    ┌─────────────────┐
                    │   Unitree Go1   │
                    │ 192.168.123.161 │
                    └─────────────────┘
```

## Quick Start

### Prerequisites
- Docker Desktop (Windows/macOS) or Docker Engine (Linux)
- Ethernet connection to Go1 robot
- Go1 robot powered on

### Step 1: Configure Network (Host Machine)

Connect Ethernet cable to Go1, then configure your network interface.

<details>
<summary><b>Linux</b></summary>

```bash
# From repository root
./unitree_network_setup.sh enp118s0  # Replace with your interface

# Or use the setup script
cd ros2_unitree_ws
./setup_go1.sh network enp118s0
```

Find your interface with: `ip link show`
</details>

<details>
<summary><b>macOS</b></summary>

```bash
# Find your Ethernet interface
ifconfig -l

# Configure (replace en0 with your interface)
sudo ifconfig en0 192.168.123.162 netmask 255.255.255.0 up
```
</details>

<details>
<summary><b>Windows</b></summary>

**Option A: GUI**
1. Open `ncpa.cpl` (Network Connections)
2. Right-click your Ethernet adapter → Properties
3. Select "Internet Protocol Version 4 (TCP/IPv4)" → Properties
4. Select "Use the following IP address":
   - IP address: `192.168.123.162`
   - Subnet mask: `255.255.255.0`
5. Click OK

**Option B: PowerShell (Admin)**
```powershell
# Find your Ethernet adapter name
Get-NetAdapter

# Set static IP (replace "Ethernet" with your adapter name)
New-NetIPAddress -InterfaceAlias "Ethernet" -IPAddress 192.168.123.162 -PrefixLength 24
```
</details>

Verify connection: `ping 192.168.123.161`

### Step 2: Start Docker Container

```bash
cd docker
./run.sh       # Linux/macOS/Git Bash
./shell.sh     # Enter container
```

**Windows (PowerShell):**
```powershell
cd docker
docker compose up -d
docker compose exec ros bash
```

### Step 3: Build Workspace (Inside Container)

```bash
cd /unitree_ws
./setup_go1.sh build
```

### Step 4: Run the Bridge (Inside Container)

```bash
./setup_go1.sh run
```

You should see:
```
UDP Initialized. socketfd: 3   Port: 8080
UDP Initialized. socketfd: 4   Port: 8090
high level runing!
```

### Step 5: Send Commands (New Terminal)

Open another terminal in the container:
```bash
cd docker && ./shell.sh  # Or: docker compose exec ros bash
```

Then send commands:
```bash
source /unitree_ws/install/setup.bash

# Stand up
python3 /unitree_ws/scripts/go1_cmd.py stand_up

# Walk forward
python3 /unitree_ws/scripts/go1_cmd.py walk 0.3 0 0

# Turn in place
python3 /unitree_ws/scripts/go1_cmd.py walk 0 0 0.5

# Stop
python3 /unitree_ws/scripts/go1_cmd.py stop

# Lie down
python3 /unitree_ws/scripts/go1_cmd.py stand_down

# Interactive keyboard control
python3 /unitree_ws/scripts/go1_control.py
```

---

## Command Reference

### go1_cmd.py - Single Commands

```bash
python3 /unitree_ws/scripts/go1_cmd.py <command> [args]
```

| Command | Arguments | Description |
|---------|-----------|-------------|
| `stand_up` | - | Stand up from lying |
| `stand_down` | - | Lie down |
| `idle` | - | Idle standing pose |
| `stop` | - | Stop all movement |
| `walk` | `vx vy yaw [duration]` | Walk with velocity |
| `damping` | - | **EMERGENCY** - Motors off |

**Walk examples:**
```bash
python3 go1_cmd.py walk 0.3 0 0      # Forward
python3 go1_cmd.py walk -0.2 0 0     # Backward
python3 go1_cmd.py walk 0 0.2 0      # Strafe left
python3 go1_cmd.py walk 0 0 0.5      # Turn left
python3 go1_cmd.py walk 0.2 0 0.3 5  # Forward+turn for 5 seconds
```

### go1_control.py - Interactive Keyboard Control

```bash
python3 /unitree_ws/scripts/go1_control.py
```

| Key | Action |
|-----|--------|
| `w/s` | Forward / Backward |
| `a/d` | Turn left / right |
| `q/e` | Strafe left / right |
| `u` | Stand up |
| `j` | Stand down (lie down) |
| `i` | Idle mode |
| `SPACE` | Stop |
| `x` | **EMERGENCY STOP** |
| `ESC` | Quit |

### Raw ROS 2 Commands

```bash
# Publish command directly
ros2 topic pub /high_cmd ros2_unitree_legged_msgs/msg/HighCmd "{
  head: [254, 239],
  level_flag: 238,
  mode: 2,
  gait_type: 1,
  velocity: [0.3, 0.0],
  yaw_speed: 0.0,
  foot_raise_height: 0.08
}" --rate 50

# Monitor robot state
ros2 topic echo /high_state
```

---

## High-Level Protocol Reference

### Modes

| Mode | Value | Description |
|------|-------|-------------|
| IDLE | 0 | Default standing pose |
| FORCE_STAND | 1 | Locked stand, body pose adjustable |
| WALK | 2 | Walking mode |
| STAND_DOWN | 5 | Lie down |
| STAND_UP | 6 | Stand up |
| DAMPING | 7 | Motors off (robot collapses!) |
| RECOVERY | 8 | Recovery stand |

### Gait Types (WALK mode)

| Gait | Value | Description |
|------|-------|-------------|
| TROT | 1 | Normal trot |
| TROT_RUNNING | 2 | Faster trot |
| CLIMB_STAIR | 3 | Stair climbing |
| TROT_OBSTACLE | 4 | Obstacle negotiation |

### Velocity Limits

| Parameter | Range | Description |
|-----------|-------|-------------|
| `velocity[0]` | -1.0 to 1.0 | Forward (+) / Backward (-) |
| `velocity[1]` | -1.0 to 1.0 | Left (+) / Right (-) |
| `yaw_speed` | -1.0 to 1.0 | Counter-clockwise (+) |

### HighState Feedback

Key fields from `/high_state`:
- `imu` - Orientation, angular velocity, acceleration
- `motor_state[20]` - Motor positions and velocities
- `foot_force[4]` - Foot contact forces
- `position[3]` - Estimated x, y, z position
- `velocity[3]` - Estimated velocity
- `mode` - Current mode
- `bms` - Battery state (SOC, voltage, current)

---

## Troubleshooting

### Cannot ping 192.168.123.161
- Check Ethernet cable connection
- Verify network configuration: `ip addr show` (Linux) or `ipconfig` (Windows)
- Ensure robot is powered on (LEDs should be on)
- Try different Ethernet port

### "ros2_udp not found" or "colcon not found"
- You must be inside the Docker container
- Source the workspace: `source /unitree_ws/install/setup.bash`
- Rebuild: `./setup_go1.sh build`

### Bridge runs but robot doesn't respond
- Check robot is not in locked mode (use remote to unlock)
- Verify commands are being sent: `ros2 topic echo /high_cmd`
- Ensure mode=2 (WALK) for velocity commands
- Commands must be sent continuously at ~50Hz

### Build fails with LCM error
```bash
sudo apt-get update && sudo apt-get install -y liblcm-dev
./setup_go1.sh build clean
```

### Robot moves erratically
- Only one node should publish to `/high_cmd`
- Check velocity values are in range (-1 to 1)
- Verify bridge is running: `ros2 node list`

---

## Safety

1. **Always supervise** the robot during operation
2. **Keep remote control** within reach (L2+B for emergency stop)
3. **Start with low velocities** (0.1-0.2) and increase gradually
4. **Test in open space** away from obstacles
5. **DAMPING mode** (mode=7) will make the robot collapse - emergency only
6. **Check battery** before each session

---

## Network Reference

| Device | IP Address |
|--------|------------|
| Go1 Robot | 192.168.123.161 |
| Your Computer | 192.168.123.162 |
| Subnet Mask | 255.255.255.0 |

| Port | Direction | Purpose |
|------|-----------|---------|
| 8082 | Host → Robot | Commands |
| 8090 | Robot → Host | State feedback |

---

## File Structure

```
ros2_unitree_ws/
├── setup_go1.sh              # Main setup script
├── scripts/
│   ├── go1_cmd.py            # Single command sender
│   ├── go1_control.py        # Interactive keyboard control
│   └── go1_highlevel_bridge.sh
└── src/
    ├── ros2_unitree_legged_msgs/  # Message definitions (symlink)
    ├── unitree_legged_sdk/        # Unitree SDK (headers + libs)
    └── unitree_ros2_to_real/      # ROS 2 bridge package
```
