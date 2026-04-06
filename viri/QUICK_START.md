# Quick Start Guide - Gazebo RL Training

## 30-Second Quickstart

```bash
# Setup (one time)
chmod +x ~/catkin_ws/src/flightmare/viri/scripts/setup_gazebo_rl.sh
./scripts/setup_gazebo_rl.sh

# Build
cd ~/catkin_ws && catkin build viri

# Terminal 1: Launch simulation
roslaunch viri gazebo_rl_training.launch

# Terminal 2: Start training
cd ~/catkin_ws/src/flightmare/viri
python3 scripts/train_sb3_gazebo.py --train --algo PPO --timesteps 100000

# Terminal 3: Monitor (optional)
tensorboard --logdir=./logs/
```

## Training Commands

```bash
# Basic training (PPO)
python3 scripts/train_sb3_gazebo.py --train

# SAC (better for continuous control)
python3 scripts/train_sb3_gazebo.py --train --algo SAC --timesteps 200000

# TD3 (more stable)
python3 scripts/train_sb3_gazebo.py --train --algo TD3 --timesteps 500000

# With noisy observations (sim-to-real)
python3 scripts/train_sb3_gazebo.py --train --noisy

# Custom save directory
python3 scripts/train_sb3_gazebo.py --train --save-dir /path/to/models/
```

## Testing Commands

```bash
# Test trained model
python3 scripts/train_sb3_gazebo.py --test ./models/ppo_gazebo_rl_final.zip

# Test specific algorithm checkpoint
python3 scripts/train_sb3_gazebo.py --test ./models/ppo_gazebo_rl_100000_steps.zip
```

## Important Topics

```bash
# Monitor observations
rostopic echo /rl_agent/observation

# Monitor actions
rostopic echo /rl_agent/action

# Check control commands
rostopic echo /chaser_drone/control_command

# Chaser odometry
rostopic echo /chaser_drone/ground_truth/odometry

# Target odometry
rostopic echo /target_drone/ground_truth/odometry
```

## File Organization

```
models/              ← Save trained models here (auto-created)
logs/                ← TensorBoard logs (auto-created)
├── ppo_gazebo_rl_training/
│   └── events.out.tfevents.xxx
└── sac_gazebo_rl_training/
    └── events.out.tfevents.xxx
```

## Expected Training Results

**After ~10 minutes (1000 steps):**
- Episode reward: ~-100 to -50
- Episodes completing: ~5-10

**After ~30 minutes (5000 steps):**
- Episode reward: ~-50 to -20
- Episodes completing: ~20-30
- Agent starting to track target

**After ~1 hour (10000 steps):**
- Episode reward: ~-20 to 0
- Episodes completing: ~50-100
- Good interception performance

**After ~2 hours (30000 steps):**
- Episode reward: ~0 to +50
- Success rate: ~10-30%
- Convergent behavior

## Troubleshooting

| Problem | Solution |
|---------|----------|
| No observations | Check Gazebo is running, check topics with `rostopic list` |
| Slow training | Ensure `gui:=false` in launch file |
| Out of memory | Reduce `n_steps` in PPO or `batch_size` in SAC |
| Agent not learning | Check reward function, try `--noisy` option |
| ROS errors | Source setup: `source ~/catkin_ws/devel/setup.bash` |

## Installation

- [ ] Built with `catkin build viri`
- [ ] `gui:=false` in gazebo_rl_training.launch
- [ ] SB3 installed: `pip3 install stable-baselines3`
- [ ] Gymnasium installed: `pip3 install gymnasium`
- [ ] Can run: `python3 scripts/train_sb3_gazebo.py --help`
- [ ] Can see topics: `rostopic list`

## Performance Metrics to Watch

```
In TensorBoard (http://localhost:6006):

1. Episode Reward
   - Should increase over time
   - Negative initially, approach 0+

2. Episode Length  
   - May vary as agent explores
   - Should stabilize

3. Policy Loss
   - Should decrease and stabilize

4. Value Function Loss
   - Should decrease over time
```

## Algorithm Selection Guide

- **First time?** → Use PPO (most stable)
- **Want faster convergence?** → Try SAC
- **Need robustness?** → Use TD3
- **Unsure?** → Train 3 separate models, pick best

## Next Steps After Training

1. **Test on Flightmare vision system**:
   - Export policy as ROS node
   - Replace RL agent with trained policy

2. **Analyze behavior**:
   - Visualize in RViz
   - Plot trajectories
   - Compare with baselines

3. **Improve further**:
   - Adjust reward function
   - Add domain randomization
   - Fine-tune hyperparameters

## Common Customizations

### Change Episode Length
```xml
<!-- In gazebo_rl_training.launch -->
<param name="max_episode_time" value="60.0" />  <!-- 60 seconds instead of 30 -->
```

### Change Control Frequency
```xml
<param name="control_freq" value="100.0" />  <!-- 100 Hz instead of 50 Hz -->
```

### Change Reward Function
```python
# In train_sb3_gazebo.py - compute_reward()
def compute_reward(self, rel_pos, rel_vel):
    # Customize here
    return your_reward_value
```

### Change Network Architecture
```python
# In train_sb3_gazebo.py
model = PPO('MlpPolicy', env, policy_kwargs=dict(net_arch=[256, 256]))
#                                                           ↑ larger network
```

## Getting Help

1. **Documentation**: Read `README_GAZEBO_RL.md`
2. **Architecture**: See `GAZEBO_RL_IMPLEMENTATION.md`
3. **SB3 Docs**: https://stable-baselines3.readthedocs.io/
4. **ROS Debugging**: Use `rosgraph`, `rqt_graph`
5. **TensorBoard**: Monitor training in real-time

## Typical Workflow

```bash
# Day 1: Initial training
python3 scripts/train_sb3_gazebo.py --train --timesteps 100000
# Checkpoint saved every 10k steps

# Monitor progress
tensorboard --logdir=./logs/

# Day 2: Continue from best checkpoint
python3 scripts/train_sb3_gazebo.py --train --timesteps 100000
# SB3 will save new models alongside old ones

# Test best model
python3 scripts/train_sb3_gazebo.py --test ./models/ppo_gazebo_rl_final.zip

# Analyze results
# - Check TensorBoard metrics
# - Visually inspect agent behavior
# - Decide on next steps
```

---

**Happy training! 🚁**
