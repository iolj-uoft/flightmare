#!/bin/bash

# Gazebo RL Training Launcher Script
# This script sets up and launches the complete Gazebo RL training environment

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Print colored output
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Default values
ACTION="train"
ALGORITHM="PPO"
TIMESTEPS=100000
SAVE_DIR="./models"
LOG_DIR="./logs"
NOISY=false
HEADLESS=true

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --train)
            ACTION="train"
            shift
            ;;
        --test)
            ACTION="test"
            TEST_MODEL="$2"
            shift 2
            ;;
        --algo)
            ALGORITHM="$2"
            shift 2
            ;;
        --timesteps)
            TIMESTEPS="$2"
            shift 2
            ;;
        --save-dir)
            SAVE_DIR="$2"
            shift 2
            ;;
        --log-dir)
            LOG_DIR="$2"
            shift 2
            ;;
        --noisy)
            NOISY=true
            shift
            ;;
        --gui)
            HEADLESS=false
            shift
            ;;
        *)
            print_error "Unknown option: $1"
            exit 1
            ;;
    esac
done

print_info "Gazebo RL Training Launcher"
print_info "Action: $ACTION"
print_info "Algorithm: $ALGORITHM"

# Source ROS setup
if [ -f ~/catkin_ws/devel/setup.bash ]; then
    source ~/catkin_ws/devel/setup.bash
    print_info "ROS environment loaded"
else
    print_error "ROS setup not found at ~/catkin_ws/devel/setup.bash"
    exit 1
fi

# Create directories
mkdir -p "$SAVE_DIR"
mkdir -p "$LOG_DIR"
print_info "Created directories: $SAVE_DIR, $LOG_DIR"

# Start roscore if not running
if ! pgrep -x roscore > /dev/null; then
    print_info "Starting roscore..."
    roscore &
    ROSCORE_PID=$!
    sleep 2
else
    print_info "roscore already running"
fi

# Launch Gazebo environment
print_info "Launching Gazebo simulation..."
if [ "$HEADLESS" = true ]; then
    roslaunch viri gazebo_rl_training.launch gui:=false &
else
    roslaunch viri gazebo_rl_training.launch gui:=true &
fi
GAZEBO_PID=$!

# Wait for Gazebo to initialize
print_info "Waiting for Gazebo to initialize (30 seconds)..."
sleep 30

# Run training or testing
case $ACTION in
    train)
        print_info "Starting training..."
        rosrun viri train_sb3_gazebo.py \
            --train \
            --algo "$ALGORITHM" \
            --timesteps "$TIMESTEPS" \
            --save-dir "$SAVE_DIR" \
            --log-dir "$LOG_DIR" \
            $([ "$NOISY" = true ] && echo "--noisy")
        ;;
    test)
        if [ -z "$TEST_MODEL" ]; then
            print_error "Model path required for testing"
            exit 1
        fi
        print_info "Starting testing with model: $TEST_MODEL"
        rosrun viri train_sb3_gazebo.py --test "$TEST_MODEL"
        ;;
esac

# Cleanup
print_info "Cleaning up..."
kill $GAZEBO_PID 2>/dev/null || true
kill $ROSCORE_PID 2>/dev/null || true

print_info "Done!"
