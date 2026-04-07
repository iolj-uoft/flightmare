#!/bin/bash
# Launch N fully isolated Gazebo+ROS instances for port-shifted parallel RL training.
#
# Each instance runs its own roscore and gzserver on dedicated ports, so there is
# zero cross-env interference and each Gazebo runs at max CPU speed independently.
#
# Usage:
#   ./launch_isolated_training.sh [N_ENVS] [extra train_sb3_gazebo.py args...]
#
# Examples:
#   ./launch_isolated_training.sh 4
#   ./launch_isolated_training.sh 4 --algo PPO --timesteps 200000
#
# Port assignment for instance i:
#   ROS_MASTER_URI  = http://localhost:$((11311 + i))
#   GAZEBO_MASTER_URI = http://localhost:$((11345 + i))

set -e

N_ENVS=${1:-4}
shift || true   # remaining args forwarded to train_sb3_gazebo.py

BASE_ROS_PORT=11311
BASE_GAZ_PORT=11345

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=============================================="
echo " Isolated Gazebo RL Training"
echo " Instances  : $N_ENVS"
echo " ROS ports  : $BASE_ROS_PORT – $((BASE_ROS_PORT + N_ENVS - 1))"
echo " Gazebo ports: $BASE_GAZ_PORT – $((BASE_GAZ_PORT + N_ENVS - 1))"
echo "=============================================="

PIDS=()

cleanup() {
    echo ""
    echo "Shutting down all instances..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    # Kill any stray gzserver / rosmaster processes launched by roslaunch
    pkill -f "gzserver" 2>/dev/null || true
    pkill -f "rosmaster" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

for i in $(seq 0 $((N_ENVS - 1))); do
    ROS_PORT=$((BASE_ROS_PORT + i))
    GAZ_PORT=$((BASE_GAZ_PORT + i))
    echo "[env $i] Starting roscore on :$ROS_PORT, gzserver on :$GAZ_PORT"

    # Start dedicated roscore for this instance
    ROS_MASTER_URI=http://localhost:$ROS_PORT \
        roscore -p "$ROS_PORT" > /tmp/roscore_env${i}.log 2>&1 &
    PIDS+=($!)
    sleep 1.5   # let roscore finish binding its port

    # Launch Gazebo world + drone pair against this roscore
    ROS_MASTER_URI=http://localhost:$ROS_PORT \
    GAZEBO_MASTER_URI=http://localhost:$GAZ_PORT \
        roslaunch viri gazebo_isolated_env.launch gui:=false \
        > /tmp/gazebo_env${i}.log 2>&1 &
    PIDS+=($!)

    # Stagger launches so Gazebo instances don't race on shared resources
    sleep 7
done

echo ""
echo "All $N_ENVS instances launched. Waiting 5 s for drones to arm..."
sleep 5

echo "Starting training..."
python3 "$SCRIPT_DIR/train_sb3_gazebo.py" --train --isolated --n-envs "$N_ENVS" "$@"
