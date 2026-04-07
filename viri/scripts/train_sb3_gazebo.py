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
import threading
from pathlib import Path

from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32, Bool
from viri.msg import RLAction, RLObservation

from stable_baselines3 import PPO, SAC, TD3
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback


class GazeboRLEnv(gym.Env):
    """Gymnasium-compatible wrapper for Gazebo RL Environment"""
    
    metadata = {'render_modes': ['human']}
    
    def __init__(self, noisy=False):
        super().__init__()
        
        rospy.init_node('gazebo_rl_train', anonymous=True)
        
        # Observation: [rel_pos_x, rel_pos_y, rel_pos_z, rel_vel_x, rel_vel_y, rel_vel_z, 
        #               action_hist_1/2/3 x (roll, pitch, yaw_rate, thrust)]
        # Total: 6 state + 12 action history = 18 dimensions
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(18,), dtype=np.float32
        )
        
        # Action: [roll, pitch, yaw_rate, thrust] all normalized in [-1, 1]
        # Now using custom RLAction message with 4D support
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(4,), dtype=np.float32
        )
        
        # Publishers and subscribers
        # Subscribe to chaser drone's relative observation/reward topics
        self.action_pub = rospy.Publisher('/chaser_drone/rl_action', RLAction, queue_size=1)
        self.reset_pub = rospy.Publisher('/chaser_drone/rl_reset', Bool, queue_size=1)
        self.obs_sub = rospy.Subscriber('/chaser_drone/rl_observation', RLObservation, self.obs_callback)
        self.reward_sub = rospy.Subscriber('/chaser_drone/rl_reward', Float32, self.reward_callback)
        self.done_sub = rospy.Subscriber('/chaser_drone/rl_episode_done', Bool, self.done_callback)
        self.crash_sub = rospy.Subscriber('/chaser_drone/rl_crashed', Bool, self.crash_callback)
        
        # State buffers
        self.rel_pos = np.zeros(3, dtype=np.float32)
        self.rel_vel = np.zeros(3, dtype=np.float32)
        self.action_history = np.zeros(12, dtype=np.float32)  # 3 actions x 4 dimensions
        self.last_reward = 0.0
        self.reward_timestamp = 0  # Track when last reward was received
        self.episode_done = False  # Episode termination signal
        self.crashed = False  # Crash/flip flag
        self.reward_lock = threading.Lock()  # Protect reward access
        
        self.episode_step = 0
        self.max_steps = int(30.0 * 50.0)  # 30 seconds at 50 Hz
        self.noisy = noisy
        
        # Wait for ROS to be ready
        time.sleep(1)
        rospy.loginfo("GazeboRLEnv initialized with 4D action space [roll, pitch, yaw_rate, thrust]")
    
    def obs_callback(self, msg):
        """Callback for RLObservation with state and action history"""
        # Unpack state components
        self.rel_pos = np.array([msg.rel_pos_x, msg.rel_pos_y, msg.rel_pos_z], dtype=np.float32)
        self.rel_vel = np.array([msg.rel_vel_x, msg.rel_vel_y, msg.rel_vel_z], dtype=np.float32)
        
        # Unpack action history (3 actions x 4 dimensions each)
        self.action_history = np.array([
            msg.action_hist_1_roll, msg.action_hist_1_pitch, msg.action_hist_1_yaw_rate, msg.action_hist_1_thrust,
            msg.action_hist_2_roll, msg.action_hist_2_pitch, msg.action_hist_2_yaw_rate, msg.action_hist_2_thrust,
            msg.action_hist_3_roll, msg.action_hist_3_pitch, msg.action_hist_3_yaw_rate, msg.action_hist_3_thrust,
        ], dtype=np.float32)
    
    def reward_callback(self, msg):
        """Callback for reward signal - thread-safe"""
        with self.reward_lock:
            old_reward = self.last_reward
            self.last_reward = msg.data
            self.reward_timestamp = time.time()
            # Log all rewards for debugging
            rospy.logdebug(f"[REWARD RX] {old_reward:.2f} → {msg.data:.2f} | crashed={self.crashed} done={self.episode_done}")
    
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
        self.action_history = np.zeros(12, dtype=np.float32)
        self.last_reward = 0.0
        self.episode_done = False
        self.crashed = False

        # Tell C++ node to reset drone position and clear episode state.
        # The C++ arming sequence takes ~1.5s; wait 2s to ensure it completes.
        reset_msg = Bool()
        reset_msg.data = True
        self.reset_pub.publish(reset_msg)
        rospy.sleep(2.0)

        obs = self._get_observation()
        return obs, {}
    
    def step(self, action):
        """Execute one step"""
        # Publish 4D action [roll, pitch, yaw_rate, thrust]
        action_msg = RLAction()
        action_msg.roll = float(action[0])      # Normalized [-1, 1]
        action_msg.pitch = float(action[1])     # Normalized [-1, 1]
        action_msg.yaw_rate = float(action[2])  # Normalized [-1, 1]
        action_msg.thrust = float(action[3])    # Normalized [0, 1] -> [9.81, 19.81] m/s^2
        
        self.action_pub.publish(action_msg)
        
        # Wait for response from C++ node and allow ROS callbacks to process
        # Using longer wait to ensure crash/flip rewards are received
        # C++ publishes reward in actionCallback which may happen multiple times,
        # so we need to wait long enough for the latest one
        rospy.sleep(0.04)  # 40ms to allow callbacks to process
        
        # Get observation and reward (with thread-safe access)
        obs = self._get_observation()
        with self.reward_lock:
            reward = float(self.last_reward)
            timestamp_diff = (time.time() - self.reward_timestamp) * 1000 if self.reward_timestamp > 0 else 0
        
        # DEBUG: Log reward details for diagnostics
        if abs(reward) < 1.0:  # Only log normal step rewards (not terminal rewards)
            rospy.logdebug(f"Step {self.episode_step}: reward={reward:.4f}, distance={np.linalg.norm(self.rel_pos):.3f}m, last_rx={timestamp_diff:.1f}ms ago")
        
        self.episode_step += 1
        
        # Episode ends if:
        # 1. Max steps reached
        # 2. Drone went too far away
        # 3. Crash or flip detected
        done = self.episode_done or self.episode_step >= self.max_steps or np.linalg.norm(self.rel_pos) > 100.0
        truncated = self.episode_step >= self.max_steps
        
        # Log crash/flip events
        if self.crashed:
            with self.reward_lock:
                timestamp_diff = (time.time() - self.reward_timestamp) * 1000 if self.reward_timestamp > 0 else 0
            rospy.logwarn(f"CRASH: Expected -100.0, Got {reward:.4f} (received {timestamp_diff:.1f}ms ago), done={self.episode_done}")
        
        # Log when episode ends unexpectedly
        if done and not truncated and not self.crashed:
            rospy.loginfo(f"Episode terminated: done={self.episode_done}, rel_pos_norm={np.linalg.norm(self.rel_pos):.3f}, reward={reward}")
        
        info = {
            'rel_pos_norm': np.linalg.norm(self.rel_pos),
            'rel_vel_norm': np.linalg.norm(self.rel_vel),
            'crashed': self.crashed,
            'episode_done': self.episode_done
        }
        
        return obs, reward, done, truncated, info
    
    def _get_observation(self):
        """Get current observation: state + action history"""
        obs = np.concatenate([self.rel_pos, self.rel_vel, self.action_history], dtype=np.float32)
        
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
