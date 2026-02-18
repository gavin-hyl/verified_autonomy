#!/bin/bash
# =============================================================================
# Go1 Real Robot - Tmux Session
# =============================================================================
#
# Layout:
#   ┌──────────────────────┬──────────────────────┐
#   │ Step 3a: Base        │ Step 3b: Exploration  │
#   │   Autonomy Stack     │   Planner             │
#   ├──────────────────────┼──────────────────────┤
#   │ Step 1: UDP Bridge   │ Step 4: cmd_vel       │
#   │                      │   Bridge              │
#   ├──────────────────────┴──────────────────────┤
#   │ Step 2: Command Terminal (stand_up first)    │
#   └─────────────────────────────────────────────┘
#
# Run order — just press Enter in each pane:
#   Step 1  - UDP Bridge pane
#   Step 2  - Command Terminal (stands robot up)
#   Step 3  - EITHER Base Autonomy (3a) OR Exploration (3b), not both
#   Step 4  - cmd_vel Bridge pane
#
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

SESSION="go1"
SHELL_CMD="$SCRIPT_DIR/shell.sh"

# Export user info for docker-compose
export USER_ID=$(id -u)
export GROUP_ID=$(id -g)
export USERNAME=$(whoami)
export DISPLAY=${DISPLAY:-:0}

# Allow X11
xhost +local:docker 2>/dev/null || true

# Ensure container is running
if ! docker compose ps --status running 2>/dev/null | grep -q "vector-autonomy-ros"; then
    echo "Starting container..."
    docker compose up -d
    sleep 2
fi

# Kill existing session
tmux kill-session -t "$SESSION" 2>/dev/null || true

echo "Creating tmux session '$SESSION'..."

# ===================================================================
# Create all panes first, then populate them
# ===================================================================
tmux new-session -d -s "$SESSION" -n main

# Split into 5 panes
tmux split-window -h -t "$SESSION:main"     # pane 1: right
tmux split-window -v -t "$SESSION:main.0"   # pane 2: below pane 0
tmux split-window -v -t "$SESSION:main.1"   # pane 3: below pane 1
tmux split-window -v -t "$SESSION:main.2"   # pane 4: below pane 2

# Apply tiled layout so pane positions are stable
tmux select-layout -t "$SESSION:main" tiled

# ===================================================================
# Enter container in all panes
# ===================================================================
echo "Opening shells in container (5 panes)..."
for i in 0 1 2 3 4; do
    tmux send-keys -t "$SESSION:main.$i" "$SHELL_CMD" C-m
done
sleep 4

# ===================================================================
# Pane 0 — Step 3a: Base Autonomy
# ===================================================================
tmux send-keys -t "$SESSION:main.0" "cd /workspace && source install/setup.bash && export ROBOT_CONFIG_PATH=unitree/unitree_g1" C-m
sleep 0.3
tmux send-keys -t "$SESSION:main.0" "echo ''" C-m
tmux send-keys -t "$SESSION:main.0" "echo '=== Step 3a: Route Planner (PCT) ==='" C-m
tmux send-keys -t "$SESSION:main.0" "echo 'Press Enter to launch. (Or use 3b for exploration.)'" C-m
tmux send-keys -t "$SESSION:main.0" "echo ''" C-m
tmux send-keys -t "$SESSION:main.0" "USE_PCT_PLANNER=true ./system_real_robot_with_route_planner.sh"

# ===================================================================
# Pane 1 — Step 3b: Exploration Planner
# ===================================================================
tmux send-keys -t "$SESSION:main.1" "cd /workspace && source install/setup.bash && export ROBOT_CONFIG_PATH=unitree/unitree_g1" C-m
sleep 0.3
tmux send-keys -t "$SESSION:main.1" "echo ''" C-m
tmux send-keys -t "$SESSION:main.1" "echo '=== Step 3b: Exploration Planner ==='" C-m
tmux send-keys -t "$SESSION:main.1" "echo 'Press Enter to launch. (Or use 3a for base autonomy.)'" C-m
tmux send-keys -t "$SESSION:main.1" "echo ''" C-m
tmux send-keys -t "$SESSION:main.1" "./system_real_robot_with_exploration_planner.sh"

# ===================================================================
# Pane 2 — Step 1: UDP Bridge
# ===================================================================
tmux send-keys -t "$SESSION:main.2" "cd /unitree_ws && source install/setup.bash" C-m
sleep 0.3
tmux send-keys -t "$SESSION:main.2" "echo ''" C-m
tmux send-keys -t "$SESSION:main.2" "echo '>>> Step 1: UDP Bridge — press Enter FIRST <<<'" C-m
tmux send-keys -t "$SESSION:main.2" "echo ''" C-m
tmux send-keys -t "$SESSION:main.2" "ros2 run unitree_legged_real ros2_udp HIGHLEVEL"

# ===================================================================
# Pane 3 — Step 4: cmd_vel -> high_cmd Bridge
# ===================================================================
tmux send-keys -t "$SESSION:main.3" "source /unitree_ws/install/setup.bash && source /workspace/install/setup.bash" C-m
sleep 0.3
tmux send-keys -t "$SESSION:main.3" "echo ''" C-m
tmux send-keys -t "$SESSION:main.3" "echo '=== Step 4: cmd_vel -> high_cmd Bridge ==='" C-m
tmux send-keys -t "$SESSION:main.3" "echo 'Press Enter AFTER launching autonomy stack (step 3).'" C-m
tmux send-keys -t "$SESSION:main.3" "echo ''" C-m
tmux send-keys -t "$SESSION:main.3" "python3 /unitree_ws/scripts/cmd_vel_to_high_cmd.py"

# ===================================================================
# Pane 4 — Step 2: Command Terminal
# ===================================================================
tmux send-keys -t "$SESSION:main.4" "source /unitree_ws/install/setup.bash && source /workspace/install/setup.bash" C-m
sleep 0.3
tmux send-keys -t "$SESSION:main.4" "echo ''" C-m
tmux send-keys -t "$SESSION:main.4" "echo '>>> Step 2: Stand up the robot — press Enter <<<'" C-m
tmux send-keys -t "$SESSION:main.4" "echo 'Then use this terminal for monitoring/commands.'" C-m
tmux send-keys -t "$SESSION:main.4" "echo ''" C-m
tmux send-keys -t "$SESSION:main.4" "python3 /unitree_ws/scripts/go1_cmd.py stand_up"

# ===================================================================
# Focus on Step 1 pane and attach
# ===================================================================
echo "Ready! Attaching to session..."
tmux select-pane -t "$SESSION:main.2"
tmux attach -t "$SESSION"
