#!/bin/bash
# Launch parallel Gazebo RL training environments.
#
# Usage:
#   ./launch_parallel_training.sh [N_ENVS] [--gui] [EXTRA_TRAIN_ARGS...]
#
# Examples:
#   ./launch_parallel_training.sh 4
#   ./launch_parallel_training.sh 4 --gui
#   ./launch_parallel_training.sh 4 --algo SAC --timesteps 200000
#
# Each environment is separated by 20 m in Y to avoid physical collisions.
# All environments share one Gazebo instance (launched via gazebo_base.launch).

set -e

N_ENVS=${1:-4}
shift || true  # remaining args are passed to train_sb3_gazebo.py

# Parse --gui flag (consumed here, not forwarded to Python)
GUI=false
TRAIN_ARGS=()
for arg in "$@"; do
    if [[ "$arg" == "--gui" ]]; then
        GUI=true
    else
        TRAIN_ARGS+=("$arg")
    fi
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"

echo "=============================================="
echo " Parallel Gazebo RL Training"
echo " Environments : $N_ENVS"
echo " Y separation : 20 m per env"
echo "=============================================="

# ── 1. Launch Gazebo world (headless) ─────────────────────────────────────────
echo "[1/3] Launching Gazebo world (gui=$GUI)..."
roslaunch viri gazebo_base.launch gui:=$GUI &
GAZEBO_PID=$!
echo "      gzserver PID: $GAZEBO_PID"

echo "      Waiting 8 s for Gazebo to start..."
sleep 8

# ── 2. Spawn N drone pairs ─────────────────────────────────────────────────────
echo "[2/3] Spawning $N_ENVS drone pairs..."
ENV_PIDS=()
for i in $(seq 0 $((N_ENVS - 1))); do
    Y_OFFSET=$((i * 20))
    echo "      env_$i  (y_offset=${Y_OFFSET} m)"
    roslaunch viri gazebo_rl_env.launch env_id:=$i y_offset:=$Y_OFFSET &
    ENV_PIDS+=($!)
    sleep 3  # stagger spawns to avoid race conditions in Gazebo model loading
done

echo "      Waiting 5 s for all drones to initialise..."
sleep 5

# ── 3. Start training ──────────────────────────────────────────────────────────
echo "[3/3] Starting training with $N_ENVS envs..."
python3 "$SCRIPT_DIR/train_sb3_gazebo.py" --train --n-envs "$N_ENVS" "${TRAIN_ARGS[@]}"

# ── Cleanup on exit ────────────────────────────────────────────────────────────
echo "Training finished. Shutting down ROS nodes..."
kill "${ENV_PIDS[@]}" 2>/dev/null || true
kill "$GAZEBO_PID"    2>/dev/null || true
rosnode kill --all    2>/dev/null || true
