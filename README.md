# Verified Autonomy

This repository contains a verified autonomy stack for robotic navigation, including support for simulation and real robot deployment (Unitree Go 1).

## Table of Contents

- [Prerequisites](#prerequisites)
- [Quick Start](#quick-start)
- [Simulation Setup](#simulation-setup)
- [Unitree Go 1 Real Robot Setup](#unitree-go-1-real-robot-setup)
- [Links](#links)

---

## Prerequisites

- **Linux host** (tested on Ubuntu 20.04, 22.04, 24.04)
- **~10GB disk space** (Docker image + Unity environments)
- **Ethernet port** (for Unitree Go 1 connection)

## Quick Start

### 1. Clone this repository

```bash
git clone https://github.com/gavin-hyl/verified_autonomy.git
cd verified_autonomy
```

### 2. Clone the Vector Navigation Stack

```bash
git clone https://github.com/VectorRobotics/vector_navigation_stack.git
```

### 3. Install Docker (if not installed)

```bash
sudo apt update
sudo apt install -y docker.io docker-compose-v2
sudo usermod -aG docker $USER

# IMPORTANT: Log out and log back in for group changes to take effect
# Or run: newgrp docker
```

Verify Docker installation:
```bash
docker run hello-world
```

### 4. Follow the README in `docker/`

For detailed simulation setup, see [docker/README.md](docker/README.md)

---

## Simulation Setup

See the comprehensive guide in [docker/README.md](docker/README.md) for:
- Building the Docker environment
- Downloading and setting up Unity environments
- Running simulations with the autonomy stack

**Quick simulation start:**
```bash
cd docker
./build.sh          # Build Docker image (one-time, ~10-15 min)
./run.sh            # Start container
./shell.sh          # Enter container
cb-sim              # Build autonomy stack
cd /workspace
./system_simulation.sh  # Run simulation
```

---

## Unitree Go 1 Real Robot Setup

This section provides instructions for connecting to and controlling a Unitree Go 1 robot using ROS 2.

### Prerequisites for Real Robot

- Ubuntu 20.04 or 22.04 (recommended for hardware compatibility)
- ROS 2 Humble or Eloquent
- Unitree Legged SDK v3.5.1
- Ethernet cable for direct connection to robot

### Network Configuration

#### 1. Physical Connection

Connect an Ethernet cable from your computer to the Unitree Go 1's Ethernet port.

#### 2. Identify Network Interface

Find the name of your Ethernet interface:
```bash
ifconfig
```

Look for an interface like `enx000ec6612921` or `eth0` that appears when the robot is connected.

#### 3. Configure Static IP

The Unitree Go 1 expects your computer to be on the `192.168.123.x` network. Configure your network interface:

**Option A: Temporary configuration (resets after reboot)**
```bash
# Replace enx000ec6612921 with your interface name
sudo ifconfig enx000ec6612921 192.168.123.162 netmask 255.255.255.0
```

**Option B: Permanent configuration**

Create a script `ipconfig.sh`:
```bash
#!/bin/bash
# Replace enx000ec6612921 with your interface name
sudo ifconfig enx000ec6612921 down
sudo ifconfig enx000ec6612921 192.168.123.162 netmask 255.255.255.0
sudo ifconfig enx000ec6612921 up
```

Make it executable:
```bash
chmod +x ipconfig.sh
sudo ./ipconfig.sh
```

**Option C: Automatic configuration on boot**

Add to `/etc/network/interfaces` (replace interface name):
```bash
auto enx000ec6612921
iface enx000ec6612921 inet static
    address 192.168.123.162
    netmask 255.255.255.0
```

#### 4. Verify Connection

Test connectivity to the robot:
```bash
# Ping the robot (high-level controller)
ping 192.168.123.161

# The robot should respond with packets
```

**Unitree Go 1 IP Addresses:**
- **Robot high-level controller**: `192.168.123.161`
- **Your computer**: `192.168.123.162`

### Installing Unitree ROS 2 Packages

#### 1. Create ROS 2 Workspace

```bash
mkdir -p ~/ros2_unitree_ws/src
cd ~/ros2_unitree_ws/src
```

#### 2. Clone Unitree ROS 2 Packages

```bash
# Clone the unitree_ros2_to_real package
git clone https://github.com/unitreerobotics/unitree_ros2_to_real.git

# Clone the Unitree Legged SDK v3.5.1
git clone -b v3.5.1 https://github.com/unitreerobotics/unitree_legged_sdk.git
```

Your workspace structure should look like:
```
~/ros2_unitree_ws/
├── src/
│   ├── unitree_ros2_to_real/
│   └── unitree_legged_sdk/
```

#### 3. Install Dependencies

```bash
cd ~/ros2_unitree_ws
sudo apt update
sudo apt install -y ros-humble-desktop-full  # or ros-eloquent-desktop-full
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

#### 4. Build the Workspace

```bash
cd ~/ros2_unitree_ws
colcon build --symlink-install
source install/setup.bash
```

Add to your `~/.bashrc` for automatic sourcing:
```bash
echo "source ~/ros2_unitree_ws/install/setup.bash" >> ~/.bashrc
```

### Controlling the Unitree Go 1

#### High-Level Control (Recommended for Navigation)

High-level control allows you to send velocity commands and basic locomotion commands.

**1. Start the UDP bridge node:**
```bash
ros2 run unitree_legged_real ros2_udp highlevel
```

This node creates a communication bridge between ROS 2 and the robot's high-level controller.

**2. In another terminal, run a control example:**
```bash
ros2 run unitree_legged_real ros2_walk_example
```

This will make the robot stand up and walk forward. Press `Ctrl+C` to stop.

**3. Send custom velocity commands:**

You can publish to the `/cmd_vel` topic (if available) or use the Unitree API to send custom commands.

Example Python script to control the robot:
```python
#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist

def main():
    rclpy.init()
    node = rclpy.create_node('unitree_controller')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    msg = Twist()
    msg.linear.x = 0.2   # Move forward at 0.2 m/s
    msg.angular.z = 0.0  # No rotation

    rate = node.create_rate(10)  # 10 Hz

    try:
        while rclpy.ok():
            pub.publish(msg)
            rate.sleep()
    except KeyboardInterrupt:
        msg.linear.x = 0.0
        pub.publish(msg)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Low-Level Control (Advanced - Joint Control)

Low-level control gives you direct access to motor positions and torques.

**WARNING:** Low-level control can damage the robot if used incorrectly. Always secure the robot before testing.

**1. Put robot in low-level mode:**
- Press **L2+A** on the remote to make the robot sit
- Press **L1+L2+Start** to enable joint control mode
- Secure the robot so it cannot fall

**2. Start the UDP bridge for low-level:**
```bash
ros2 run unitree_legged_real ros2_udp lowlevel
```

**3. Run low-level example:**
```bash
ros2 run unitree_legged_real ros2_position_example
```

### Available ROS 2 Topics

Once the bridge is running, you can inspect available topics:

```bash
# List all topics
ros2 topic list

# Example topics (high-level mode):
# /cmd_vel           - Velocity commands
# /imu               - IMU data
# /joint_states      - Joint positions and velocities
# /odom              - Odometry data
```

### Integrating with Verified Autonomy Stack

To integrate the Unitree Go 1 with this autonomy stack:

1. **Configure the robot platform** in your workspace:
   ```bash
   export ROBOT_CONFIG_PATH="unitree_go1"
   ```

2. **Create a launch file** that:
   - Starts the Unitree UDP bridge
   - Launches the autonomy stack with appropriate remappings
   - Maps `/cmd_vel` from the planner to the Unitree controller

3. **Sensor integration**:
   - Connect additional sensors (LiDAR, cameras) to the onboard computer
   - Configure sensor drivers in the autonomy stack
   - Ensure all sensors publish on correct ROS 2 topics

### Safety Notes

- **Always supervise** the robot during operation
- **Keep emergency stop** (remote control) within reach
- **Test in safe environment** before deploying in complex spaces
- **Start with low velocities** and increase gradually
- **Check battery level** before each session
- **Be aware of network latency** over WiFi (Ethernet is recommended)

### Troubleshooting

**Cannot ping robot (192.168.123.161):**
- Verify Ethernet cable is connected
- Check that your IP is set to 192.168.123.162
- Ensure robot is powered on
- Try different Ethernet port on your computer

**ROS 2 bridge not receiving data:**
- Verify network configuration with `ifconfig`
- Check firewall settings: `sudo ufw status`
- Ensure no other programs are using the same UDP ports
- Restart the robot's high-level controller

**Robot not responding to commands:**
- Check that bridge node is running: `ros2 node list`
- Verify topic names: `ros2 topic list`
- Monitor topic data: `ros2 topic echo /cmd_vel`
- Ensure robot is not in locked mode (check remote control)

**Build errors:**
- Ensure you have ROS 2 Humble or Eloquent installed
- Run `rosdep install` again to check dependencies
- Check SDK version is v3.5.1: `cd unitree_legged_sdk && git describe --tags`

---

## Links

- [Vector Navigation Stack](https://github.com/VectorRobotics/vector_navigation_stack)
- [Unitree ROS 2 Real Robot Interface](https://github.com/unitreerobotics/unitree_ros2_to_real)
- [Unitree Legged SDK](https://github.com/unitreerobotics/unitree_legged_sdk)
- [CMU Exploration Development Environment](https://www.cmu-exploration.com/development-environment)
- [Unity Environments (Google Drive)](https://drive.google.com/drive/folders/1G1JYkccvoSlxyySuTlPfvmrWoJUO8oSs?usp=sharing)
