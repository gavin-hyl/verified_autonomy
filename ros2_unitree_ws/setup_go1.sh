#!/bin/bash
# =============================================================================
# Unitree Go1 ROS 2 Setup Script
# =============================================================================
# Cross-platform setup script for Linux, macOS, and Windows (Git Bash/WSL)
#
# Usage:
#   ./setup_go1.sh              # Full setup (build + instructions)
#   ./setup_go1.sh build        # Build workspace only
#   ./setup_go1.sh network      # Configure network only (Linux/macOS)
#   ./setup_go1.sh test         # Test robot connection
#   ./setup_go1.sh run          # Run the UDP bridge
# =============================================================================

set -e

# Colors (works in most terminals including Windows Git Bash)
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
ROBOT_IP="192.168.123.161"
HOST_IP="192.168.123.162"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"

# Detect OS
detect_os() {
    case "$(uname -s)" in
        Linux*)     OS="linux";;
        Darwin*)    OS="macos";;
        CYGWIN*|MINGW*|MSYS*) OS="windows";;
        *)          OS="unknown";;
    esac
    echo "$OS"
}

# Detect if running inside Docker
is_docker() {
    [ -f /.dockerenv ] || grep -q docker /proc/1/cgroup 2>/dev/null
}

# Print colored message
print_msg() {
    local color=$1
    local msg=$2
    echo -e "${color}${msg}${NC}"
}

# Print section header
print_header() {
    echo ""
    print_msg "$BLUE" "=============================================="
    print_msg "$BLUE" "$1"
    print_msg "$BLUE" "=============================================="
}

# Check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Build the workspace
build_workspace() {
    print_header "Building Unitree ROS 2 Workspace"

    cd "$WORKSPACE_DIR"

    # Check if we're in Docker or have ROS 2 sourced
    if ! command_exists colcon; then
        if [ -f "/opt/ros/jazzy/setup.bash" ]; then
            source /opt/ros/jazzy/setup.bash
        elif [ -f "/opt/ros/humble/setup.bash" ]; then
            source /opt/ros/humble/setup.bash
        else
            print_msg "$RED" "Error: colcon not found. Are you in the Docker container?"
            print_msg "$YELLOW" "Run: cd docker && ./shell.sh"
            exit 1
        fi
    fi

    # Install LCM if needed
    if ! dpkg -l | grep -q liblcm-dev 2>/dev/null; then
        print_msg "$YELLOW" "Installing liblcm-dev..."
        sudo apt-get update && sudo apt-get install -y liblcm-dev || true
    fi

    # Clean build if requested or if there are issues
    if [ "$1" == "clean" ] || [ ! -d "install" ]; then
        print_msg "$YELLOW" "Cleaning previous build..."
        rm -rf build install log
    fi

    # Build
    print_msg "$GREEN" "Building workspace..."
    colcon build --symlink-install

    print_msg "$GREEN" "Build complete!"
    echo ""
    echo "To use the workspace, run:"
    echo "  source $WORKSPACE_DIR/install/setup.bash"
}

# Configure network (Linux/macOS only)
configure_network() {
    print_header "Network Configuration"

    local os=$(detect_os)

    if [ "$os" == "windows" ]; then
        print_msg "$YELLOW" "Windows detected. Please configure network manually:"
        echo ""
        echo "1. Open 'Network Connections' (ncpa.cpl)"
        echo "2. Right-click your Ethernet adapter -> Properties"
        echo "3. Select 'Internet Protocol Version 4 (TCP/IPv4)' -> Properties"
        echo "4. Select 'Use the following IP address':"
        echo "   IP address: $HOST_IP"
        echo "   Subnet mask: 255.255.255.0"
        echo "5. Click OK"
        echo ""
        echo "Or use PowerShell (Admin):"
        echo "  New-NetIPAddress -InterfaceAlias 'Ethernet' -IPAddress $HOST_IP -PrefixLength 24"
        return
    fi

    # Linux/macOS
    if [ -z "$1" ]; then
        echo "Available network interfaces:"
        echo ""
        if [ "$os" == "linux" ]; then
            ip link show | grep -E "^[0-9]+:" | grep -v "lo:" | while read line; do
                iface=$(echo "$line" | awk -F': ' '{print $2}' | cut -d'@' -f1)
                state=$(echo "$line" | grep -oP 'state \K\w+' || echo "UNKNOWN")
                echo "  $iface ($state)"
            done
        else
            ifconfig -l | tr ' ' '\n' | grep -v "lo" | while read iface; do
                echo "  $iface"
            done
        fi
        echo ""
        echo "Usage: $0 network <interface_name>"
        echo "Example: $0 network enp118s0"
        return 1
    fi

    local interface=$1

    print_msg "$YELLOW" "Configuring $interface with IP $HOST_IP..."

    if [ "$os" == "linux" ]; then
        sudo ip link set "$interface" down
        sudo ip addr flush dev "$interface"
        sudo ip addr add "$HOST_IP/24" dev "$interface"
        sudo ip link set "$interface" up
    else
        # macOS
        sudo ifconfig "$interface" "$HOST_IP" netmask 255.255.255.0 up
    fi

    print_msg "$GREEN" "Network configured!"
    test_connection
}

# Test robot connection
test_connection() {
    print_header "Testing Robot Connection"

    echo "Pinging robot at $ROBOT_IP..."
    if ping -c 1 -W 2 "$ROBOT_IP" >/dev/null 2>&1; then
        print_msg "$GREEN" "SUCCESS: Robot is reachable!"
    else
        print_msg "$RED" "FAILED: Cannot reach robot at $ROBOT_IP"
        echo ""
        echo "Troubleshooting:"
        echo "  1. Check Ethernet cable is connected"
        echo "  2. Verify robot is powered on"
        echo "  3. Run: $0 network <interface>"
    fi
}

# Run the UDP bridge
run_bridge() {
    print_header "Starting Go1 High-Level Bridge"

    cd "$WORKSPACE_DIR"

    if [ ! -f "install/setup.bash" ]; then
        print_msg "$RED" "Workspace not built. Run: $0 build"
        exit 1
    fi

    source install/setup.bash

    print_msg "$GREEN" "Starting UDP bridge..."
    echo "  Subscribes to: /high_cmd"
    echo "  Publishes to:  /high_state"
    echo ""
    echo "Press Ctrl+C to stop"
    echo ""

    ros2 run unitree_legged_real ros2_udp HIGHLEVEL
}

# Show help
show_help() {
    echo "Unitree Go1 ROS 2 Setup Script"
    echo ""
    echo "Usage: $0 [command] [options]"
    echo ""
    echo "Commands:"
    echo "  build [clean]     Build the ROS 2 workspace"
    echo "  network [iface]   Configure network for Go1 connection"
    echo "  test              Test connection to robot"
    echo "  run               Run the UDP bridge"
    echo "  help              Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                    # Show setup instructions"
    echo "  $0 build              # Build workspace"
    echo "  $0 build clean        # Clean and rebuild"
    echo "  $0 network enp118s0   # Configure network"
    echo "  $0 run                # Start bridge"
}

# Show full setup instructions
show_setup() {
    local os=$(detect_os)

    print_header "Go1 Setup Instructions"

    echo ""
    print_msg "$GREEN" "Step 1: Network Configuration"
    echo "--------------------------------------------"
    if [ "$os" == "windows" ]; then
        echo "On Windows, configure your Ethernet adapter:"
        echo "  IP: $HOST_IP"
        echo "  Subnet: 255.255.255.0"
        echo "  (See: $0 network for detailed instructions)"
    else
        echo "Run on HOST (not in Docker):"
        echo "  $0 network <ethernet_interface>"
    fi

    echo ""
    print_msg "$GREEN" "Step 2: Start Docker Container"
    echo "--------------------------------------------"
    echo "  cd docker"
    echo "  ./run.sh      # Windows: ./run.sh or docker compose up -d"
    echo "  ./shell.sh    # Windows: ./shell.sh or docker compose exec ros bash"

    echo ""
    print_msg "$GREEN" "Step 3: Build Workspace (in container)"
    echo "--------------------------------------------"
    echo "  cd /unitree_ws"
    echo "  ./setup_go1.sh build"

    echo ""
    print_msg "$GREEN" "Step 4: Run Bridge (in container)"
    echo "--------------------------------------------"
    echo "  ./setup_go1.sh run"

    echo ""
    print_msg "$GREEN" "Step 5: Send Commands (new container terminal)"
    echo "--------------------------------------------"
    echo "  source /unitree_ws/install/setup.bash"
    echo "  python3 /unitree_ws/scripts/go1_cmd.py stand_up"
    echo "  python3 /unitree_ws/scripts/go1_cmd.py walk 0.3 0 0"
    echo "  python3 /unitree_ws/scripts/go1_control.py  # Interactive"

    echo ""
    print_msg "$YELLOW" "Robot IP: $ROBOT_IP"
    print_msg "$YELLOW" "Host IP:  $HOST_IP"
}

# Main
main() {
    case "${1:-}" in
        build)
            build_workspace "${2:-}"
            ;;
        network)
            configure_network "${2:-}"
            ;;
        test)
            test_connection
            ;;
        run)
            run_bridge
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            show_setup
            ;;
    esac
}

main "$@"
