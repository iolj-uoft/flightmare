# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**VIRI** (Visual Interception ROS Integration) is a ROS package for training a quadrotor drone agent to intercept a moving target using reinforcement learning in Gazebo simulation. It uses Stable Baselines 3 (SB3) with ground-truth observations to achieve 25-50x training speedup over vision-based approaches.

## Build Commands

```bash
# Build the package
cd ~/catkin_ws && catkin build viri -j4
source ~/catkin_ws/devel/setup.bash

# Install Python dependencies
pip3 install stable-baselines3 gymnasium torch tensorboard
```

## Running

**Terminal 1 — Start simulation:**
```bash
roslaunch viri gazebo_rl_training.launch gui:=false
```

**Terminal 2 — Run training:**
```bash
cd ~/catkin_ws/src/flightmare/viri
python3 scripts/train_sb3_gazebo.py --train --algo PPO --timesteps 100000
# Supported algorithms: PPO (default), SAC, TD3
```

**Test a trained model:**
```bash
python3 scripts/train_sb3_gazebo.py --test ./models/ppo_gazebo_rl_final.zip
```

**Monitor training:**
```bash
tensorboard --logdir=./logs/
```

**Verify system setup:**
```bash
python3 scripts/test_gazebo_rl.py
```

## Architecture

The system has three layers that communicate via ROS topics:

```
Gazebo (physics) → gazebo_rl_agent_node (C++) → train_sb3_gazebo.py (Python/SB3)
```

1. **Gazebo** spawns two hummingbird drones and publishes ground-truth odometry at 50 Hz.

2. **`gazebo_rl_agent_node`** (`src/training/gazebo_rl_agent_node.cpp`, 605 lines) is the core bridge:
   - Subscribes to chaser and target odometry
   - Computes 18D observation: relative position (3D) + relative velocity (3D) + 3-step action history (12D)
   - Receives 4D normalized actions from Python via `/chaser_drone/rl_action`
   - Maps actions → attitude commands (roll/pitch/yaw_rate/thrust) sent to `rpg_rotors_interface`
   - Detects crashes/flips and publishes rewards back to Python

3. **`train_sb3_gazebo.py`** (`scripts/train_sb3_gazebo.py`, 457 lines):
   - `GazeboRLEnv(gym.Env)` wraps ROS into a Gymnasium interface
   - Observation space: 18D Box, Action space: 4D Box [-1, 1]
   - Checkpoints every 10k steps to `scripts/models/`
   - `GazeboRLEnvNoisy` variant adds sensor noise injection

## Key ROS Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/chaser_drone/rl_observation` | `viri/RLObservation` | agent_node → Python |
| `/chaser_drone/rl_reward` | `std_msgs/Float32` | agent_node → Python |
| `/chaser_drone/rl_action` | `viri/RLAction` | Python → agent_node |
| `/chaser_drone/control_command` | `quadrotor_msgs/ControlCommand` | agent_node → rotors |

## Custom Message Types

- **`msg/RLAction.msg`**: `roll, pitch, yaw_rate, thrust` (float64 each)
- **`msg/RLObservation.msg`**: 6D relative state + 12D action history

## Action Mapping

Normalized action `[-1, 1]` maps to:
- Roll/Pitch: ±30° (±π/6 rad)
- Yaw rate: ±180°/s (±π rad/s)
- Thrust: 5–20 m/s² (quadratic mapping, 1.0 → hover)

## Reward Function

Located in `train_sb3_gazebo.py`:
```
reward = -0.1 * ||rel_pos|| - 0.01 * ||rel_vel||
       + 10 bonus if ||rel_pos|| < 1 m (intercept)
       - 50 penalty if ||rel_pos|| > 100 m (lost target)
       - 100 penalty on crash/flip
```

## Launch File Parameters

`launch/interceptor/gazebo_rl_training.launch`:
- `gui:=false` — headless mode (2-3x faster)
- `control_freq:=50.0` — control loop Hz
- `max_episode_time:=30.0` — episode length in seconds
- `max_relative_distance:=100.0` — failure distance threshold in meters

## Models and Logs

- Trained models: `scripts/models/ppo_gazebo_rl_*_steps.zip`
- TensorBoard logs: `scripts/logs/ppo_training_*/`
- Checkpoints saved every 10k timesteps automatically
