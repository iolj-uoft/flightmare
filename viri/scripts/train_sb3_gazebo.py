#!/usr/bin/env python3

"""
Gazebo RL Training Wrapper with Stable Baselines 3
Communicates with C++ ROS nodes to provide RL environment interface
"""

import rospy
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import subprocess
import time
import os
from pathlib import Path

from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32, Bool
from viri.msg import RLAction

from stable_baselines3 import PPO, SAC, TD3
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback


class GazeboRLEnv(gym.Env):
    """Gymnasium-compatible wrapper for Gazebo RL Environment"""
    
    metadata = {'render_modes': ['human']}
    
    def __init__(self, noisy=False):
        super().__init__()
        
        rospy.init_node('gazebo_rl_train', anonymous=True)
        
        # Observation: [rel_pos_x, rel_pos_y, rel_pos_z, rel_vel_x, rel_vel_y, rel_vel_z]
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(6,), dtype=np.float32
        )
        
        # Action: [roll, pitch, yaw_rate, thrust] all normalized in [-1, 1]
        # Now using custom RLAction message with 4D support
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(4,), dtype=np.float32
        )
        
        # Publishers and subscribers
        # Subscribe to chaser drone's relative observation/reward topics
        self.action_pub = rospy.Publisher('/chaser_drone/rl_action', RLAction, queue_size=1)
        self.obs_sub = rospy.Subscriber('/chaser_drone/rl_observation', Vector3, self.obs_callback)
        self.vel_sub = rospy.Subscriber('/chaser_drone/rl_relative_velocity', Vector3, self.vel_callback)
        self.reward_sub = rospy.Subscriber('/chaser_drone/rl_reward', Float32, self.reward_callback)
        self.done_sub = rospy.Subscriber('/chaser_drone/rl_episode_done', Bool, self.done_callback)
        self.crash_sub = rospy.Subscriber('/chaser_drone/rl_crashed', Bool, self.crash_callback)
        
        # State buffers
        self.rel_pos = np.zeros(3, dtype=np.float32)
        self.rel_vel = np.zeros(3, dtype=np.float32)
        self.last_reward = 0.0
        self.episode_done = False  # Episode termination signal
        self.crashed = False  # Crash/flip flag
        
        self.episode_step = 0
        self.max_steps = int(30.0 * 50.0)  # 30 seconds at 50 Hz
        self.noisy = noisy
        
        # Wait for ROS to be ready
        time.sleep(1)
        rospy.loginfo("GazeboRLEnv initialized with 4D action space [roll, pitch, yaw_rate, thrust]")
    
    def obs_callback(self, msg):
        """Callback for relative position observations"""
        self.rel_pos = np.array([msg.x, msg.y, msg.z], dtype=np.float32)
    
    def vel_callback(self, msg):
        """Callback for relative velocity observations"""
        self.rel_vel = np.array([msg.x, msg.y, msg.z], dtype=np.float32)
    
    def reward_callback(self, msg):
        """Callback for reward signal"""
        self.last_reward = msg.data
    
    def done_callback(self, msg):
        """Callback for episode done signal (crash or flip)"""
        self.episode_done = msg.data
    
    def crash_callback(self, msg):
        """Callback for crash/flip indicator"""
        self.crashed = msg.data
    
    def reset(self, seed=None, options=None):
        """Reset environment"""
        super().reset(seed=seed)
        
        self.episode_step = 0
        self.rel_pos = np.zeros(3, dtype=np.float32)
        self.rel_vel = np.zeros(3, dtype=np.float32)
        self.last_reward = 0.0
        self.episode_done = False  # Reset episode done flag
        self.crashed = False  # Reset crash flag
        
        # Return observation
        obs = self._get_observation()
        info = {}
        
        return obs, info
    
    def step(self, action):
        """Execute one step"""
        # Publish 4D action [roll, pitch, yaw_rate, thrust]
        action_msg = RLAction()
        action_msg.roll = float(action[0])      # Normalized [-1, 1]
        action_msg.pitch = float(action[1])     # Normalized [-1, 1]
        action_msg.yaw_rate = float(action[2])  # Normalized [-1, 1]
        action_msg.thrust = float(action[3])    # Normalized [0, 1] -> [9.81, 19.81] m/s^2
        
        self.action_pub.publish(action_msg)
        
        # Wait for response
        rospy.sleep(0.02)  # 50 Hz control loop
        
        # Get observation
        obs = self._get_observation()
        reward = float(self.last_reward)
        
        self.episode_step += 1
        
        # Episode ends if:
        # 1. Max steps reached
        # 2. Drone went too far away
        # 3. Crash or flip detected
        done = self.episode_done or self.episode_step >= self.max_steps or np.linalg.norm(self.rel_pos) > 100.0
        truncated = self.episode_step >= self.max_steps
        
        # Log crash/flip events
        if self.crashed:
            rospy.logwarn(f"Episode terminated: Crash/Flip detected! Reward penalty: {reward}")
        
        info = {
            'rel_pos_norm': np.linalg.norm(self.rel_pos),
            'rel_vel_norm': np.linalg.norm(self.rel_vel),
            'crashed': self.crashed,
            'episode_done': self.episode_done
        }
        
        return obs, reward, done, truncated, info
    
    def _get_observation(self):
        """Get current observation"""
        obs = np.concatenate([self.rel_pos, self.rel_vel], dtype=np.float32)
        
        if self.noisy:
            # Add Gaussian noise to simulate sensor noise
            obs += np.random.normal(0, 0.01, size=obs.shape).astype(np.float32)
        
        return obs
    
    def render(self, mode='human'):
        pass
    
    def close(self):
        pass


def train_sb3(algorithm='PPO', num_timesteps=100000, save_dir='./models/', 
              log_dir='./logs/', env_class=GazeboRLEnv, eval_freq=5000):
    """
    Train RL agent using Stable Baselines 3.
    
    Args:
        algorithm: 'PPO', 'SAC', or 'TD3'
        num_timesteps: Total training timesteps
        save_dir: Directory to save models
        log_dir: Directory for TensorBoard logs
        env_class: Environment class to use
        eval_freq: Evaluation frequency
    """
    # Create directories
    Path(save_dir).mkdir(parents=True, exist_ok=True)
    Path(log_dir).mkdir(parents=True, exist_ok=True)
    
    # Create environment
    rospy.loginfo(f"Creating {env_class.__name__}...")
    env = env_class(max_episode_steps=1500, control_freq=50.0)
    env = DummyVecEnv([lambda: env])
    
    # Select algorithm
    if algorithm == 'PPO':
        model = PPO(
            'MlpPolicy',
            env,
            learning_rate=3e-4,
            n_steps=2048,
            batch_size=64,
            n_epochs=10,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            verbose=1,
            tensorboard_log=log_dir
        )
    elif algorithm == 'SAC':
        model = SAC(
            'MlpPolicy',
            env,
            learning_rate=3e-4,
            buffer_size=1000000,
            batch_size=256,
            tau=0.005,
            gamma=0.99,
            verbose=1,
            tensorboard_log=log_dir
        )
    elif algorithm == 'TD3':
        model = TD3(
            'MlpPolicy',
            env,
            learning_rate=1e-3,
            buffer_size=1000000,
            batch_size=100,
            tau=0.005,
            gamma=0.99,
            verbose=1,
            tensorboard_log=log_dir
        )
    else:
        raise ValueError(f"Unknown algorithm: {algorithm}")
    
    # Callbacks
    checkpoint_callback = CheckpointCallback(
        save_freq=eval_freq,
        save_path=save_dir,
        name_prefix=f'{algorithm.lower()}_gazebo_rl'
    )
    
    # Train
    rospy.loginfo(f"Starting training with {algorithm}...")
    rospy.loginfo(f"Total timesteps: {num_timesteps}")
    
    model.learn(
        total_timesteps=num_timesteps,
        callback=checkpoint_callback,
        tb_log_name=f"{algorithm.lower()}_gazebo_rl_training"
    )
    
    # Save final model
    final_model_path = os.path.join(save_dir, f'{algorithm.lower()}_gazebo_rl_final')
    model.save(final_model_path)
    rospy.loginfo(f"Training complete! Final model saved to {final_model_path}")
    
    env.close()


def test_sb3(model_path, episodes=10):
    """
    Test trained model.
    
    Args:
        model_path: Path to saved model
        episodes: Number of episodes to test
    """
    # Determine algorithm from model path
    if 'ppo' in model_path.lower():
        model_class = PPO
    elif 'sac' in model_path.lower():
        model_class = SAC
    elif 'td3' in model_path.lower():
        model_class = TD3
    else:
        model_class = PPO
    
    # Load model
    rospy.loginfo(f"Loading model from {model_path}...")
    model = model_class.load(model_path)
    
    # Create environment
    env = GazeboRLEnv(max_episode_steps=1500, control_freq=50.0)
    
    # Test
    rospy.loginfo(f"Testing for {episodes} episodes...")
    for episode in range(episodes):
        obs = env.reset()
        done = False
        total_reward = 0
        steps = 0
        
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, info = env.step(action)
            total_reward += reward
            steps += 1
        
        rospy.loginfo(f"Episode {episode+1}: steps={steps}, reward={total_reward:.2f}")
    
    env.close()
    rospy.loginfo("Testing complete!")


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Train RL agent for quadrotor interception')
    parser.add_argument('--train', action='store_true', help='Start training')
    parser.add_argument('--test', type=str, help='Test trained model')
    parser.add_argument('--algo', type=str, default='PPO', choices=['PPO', 'SAC', 'TD3'])
    parser.add_argument('--timesteps', type=int, default=100000)
    parser.add_argument('--save-dir', type=str, default='./models/')
    parser.add_argument('--log-dir', type=str, default='./logs/')
    parser.add_argument('--noisy', action='store_true', help='Add observation noise')
    
    args = parser.parse_args()
    
    # Create directories
    Path(args.save_dir).mkdir(parents=True, exist_ok=True)
    Path(args.log_dir).mkdir(parents=True, exist_ok=True)
    
    if args.train:
        rospy.loginfo("Starting training...")
        
        # Create environment
        env = GazeboRLEnv(noisy=args.noisy)
        
        # Create model
        if args.algo == 'PPO':
            model = PPO(
                'MlpPolicy',
                env,
                learning_rate=3e-4,
                n_steps=2048,
                batch_size=64,
                n_epochs=10,
                gamma=0.99,
                gae_lambda=0.95,
                clip_range=0.2,
                verbose=1,
                tensorboard_log=args.log_dir
            )
        elif args.algo == 'SAC':
            model = SAC(
                'MlpPolicy',
                env,
                learning_rate=3e-4,
                buffer_size=1000000,
                batch_size=256,
                tau=0.005,
                gamma=0.99,
                verbose=1,
                tensorboard_log=args.log_dir
            )
        elif args.algo == 'TD3':
            model = TD3(
                'MlpPolicy',
                env,
                learning_rate=1e-3,
                buffer_size=1000000,
                batch_size=100,
                tau=0.005,
                gamma=0.99,
                verbose=1,
                tensorboard_log=args.log_dir
            )
        
        # Callbacks
        checkpoint_callback = CheckpointCallback(
            save_freq=10000,
            save_path=args.save_dir,
            name_prefix=f'{args.algo.lower()}_gazebo_rl'
        )
        
        # Train
        model.learn(
            total_timesteps=args.timesteps,
            callback=checkpoint_callback,
            tb_log_name=f"{args.algo.lower()}_training"
        )
        
        # Save final model
        model.save(os.path.join(args.save_dir, f'{args.algo.lower()}_gazebo_rl_final'))
        rospy.loginfo("Training complete!")
    
    elif args.test:
        rospy.loginfo(f"Testing model: {args.test}")
        
        env = GazeboRLEnv(noisy=False)
        
        # Load model
        if args.algo == 'PPO':
            model = PPO.load(args.test)
        elif args.algo == 'SAC':
            model = SAC.load(args.test)
        elif args.algo == 'TD3':
            model = TD3.load(args.test)
        
        # Test
        obs, info = env.reset()
        total_reward = 0.0
        steps = 0
        
        while steps < 1000:
            action, _states = model.predict(obs, deterministic=True)
            obs, reward, done, truncated, info = env.step(action)
            total_reward += reward
            steps += 1
            
            if done or truncated:
                rospy.loginfo(f"Episode finished! Total reward: {total_reward}, Steps: {steps}")
                break
        
        env.close()


if __name__ == '__main__':
    main()
