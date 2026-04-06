#!/usr/bin/env python3
"""
Quick diagnostic script to verify Gazebo RL training system is working
Run this after launching gazebo_rl_training.launch
"""

import rospy
import sys
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
from viri.msg import RLAction

def main():
    rospy.init_node('gazebo_rl_diagnostic', anonymous=True)
    
    print("\n=== Gazebo RL Training System Diagnostic ===\n")
    
    # Topics that should exist from the simulation
    required_topics = {
        '/chaser_drone/ground_truth/odometry': 'Chaser odometry',
        '/target_drone/ground_truth/odometry': 'Target odometry',
        '/chaser_drone/rl_observation': 'RL observation (relative pos)',
        '/chaser_drone/rl_relative_velocity': 'RL relative velocity',
        '/chaser_drone/rl_reward': 'RL reward signal',
        '/chaser_drone/control_command': 'Motor control command',
    }
    
    # Get list of available topics
    print("Checking topics...")
    try:
        available_names = rospy.get_published_topics()
        available_names = [name for name, msg_type in available_names]
    except Exception as e:
        print(f"  ERROR: Could not get topic list ({e})")
        return 1
    
    # Check required topics
    print("\n📋 Required topics (from simulation):")
    all_found = True
    for topic, description in required_topics.items():
        if topic in available_names:
            print(f"  ✓ {topic}")
        else:
            print(f"  ✗ {topic} - NOT FOUND!")
            all_found = False
    
    print("\n" + "="*80)
    
    if not all_found:
        print("\n❌ ERROR: Some expected topics not found!")
        print("\nMake sure the following is running in another terminal:")
        print("  source ~/Desktop/catkin_ws/devel/setup.bash")
        print("  roslaunch viri gazebo_rl_training.launch")
        return 1
    
    # Try publishing a test action
    print("\n🚀 Testing action publishing...")
    action_pub = rospy.Publisher('/chaser_drone/rl_action', RLAction, queue_size=1)
    rospy.sleep(0.5)  # Wait for publisher to be ready
    
    test_action = RLAction()
    test_action.roll = 0.0
    test_action.pitch = 0.0
    test_action.yaw_rate = 0.0
    test_action.thrust = 0.5  # Hover thrust
    
    action_pub.publish(test_action)
    print("  ✓ Published hover command (roll=0, pitch=0, yaw_rate=0, thrust=0.5)")
    
    # Try subscribing to observations
    print("\n📡 Listening for observations (5 seconds)...\n")
    
    obs_received = {'pos': False, 'vel': False, 'reward': False}
    last_obs = {}
    
    def obs_callback(msg):
        obs_received['pos'] = True
        last_obs['pos'] = (msg.x, msg.y, msg.z)
        print(f"  ✓ Observation: rel_pos=[{msg.x:6.3f}, {msg.y:6.3f}, {msg.z:6.3f}]")
    
    def vel_callback(msg):
        obs_received['vel'] = True
        last_obs['vel'] = (msg.x, msg.y, msg.z)
        print(f"  ✓ Velocity:    rel_vel=[{msg.x:6.3f}, {msg.y:6.3f}, {msg.z:6.3f}]")
    
    def reward_callback(msg):
        obs_received['reward'] = True
        last_obs['reward'] = msg.data
        print(f"  ✓ Reward:      {msg.data:7.3f}")
    
    obs_sub = rospy.Subscriber('/chaser_drone/rl_observation', Vector3, obs_callback)
    vel_sub = rospy.Subscriber('/chaser_drone/rl_relative_velocity', Vector3, vel_callback)
    reward_sub = rospy.Subscriber('/chaser_drone/rl_reward', Float32, reward_callback)
    
    # Publish action periodically while listening
    start_time = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < 5:
        # Keep publishing the test action to trigger callbacks
        test_action.roll = 0.1 * (rospy.Time.now() - start_time).to_sec()  # Vary slightly
        action_pub.publish(test_action)
        rospy.sleep(0.05)
    
    print("\n" + "="*80)
    
    if obs_received['pos']:
        print("\n✅ SUCCESS! System is working correctly!")
        print("\nYou can now run RL training:")
        print("  python3 src/flightmare/viri/scripts/train_sb3_gazebo.py")
        return 0
    else:
        print("\n❌ No observations received!")
        print("\nDebugging steps:")
        print("  1. Check if gazebo_rl_agent_node is running:")
        print("     rosnode list | grep rl_agent")
        print("  2. Check C++ node output for errors:")
        print("     rostopic echo /chaser_drone/rl_observation")
        print("  3. Verify odometry is being published:")
        print("     rostopic echo /chaser_drone/ground_truth/odometry")
        return 1

if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\nDiagnostic interrupted by user")
        sys.exit(0)
