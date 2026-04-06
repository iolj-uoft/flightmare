#!/usr/bin/env python3
"""
Simple test action publisher that sends periodic RLAction commands to test drone control.
This allows testing the C++ agent node without running the full RL training pipeline.
"""

import rospy
from viri.msg import RLAction
import math

def main():
    rospy.init_node('test_action_publisher')
    
    # Publisher for RL actions
    action_pub = rospy.Publisher('rl_action', RLAction, queue_size=1)
    
    rospy.loginfo("Test Action Publisher started")
    rospy.loginfo("Publishing test actions to 'rl_action' at 50 Hz")
    
    rate = rospy.Rate(50)  # 50 Hz
    start_time = rospy.Time.now()
    test_phase = 0
    last_phase_switch = start_time
    
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        elapsed = (current_time - last_phase_switch).to_sec()
        
        action = RLAction()
        
        # Test phases: each 3 seconds
        if elapsed < 3:
            # Phase 1: Neutral (hover)
            action.roll = 0.0
            action.pitch = 0.0
            action.yaw_rate = 0.0
            action.thrust = 0.0  # Neutral - should hover
            test_phase = 1
        elif elapsed < 6:
            # Phase 2: Very slight climb
            action.roll = 0.0
            action.pitch = 0.0
            action.yaw_rate = 0.0
            action.thrust = 0.1  # Slight thrust above hover
            test_phase = 2
        elif elapsed < 9:
            # Phase 3: Slight pitch forward (aggressive climb reduces available thrust for forward motion)
            action.roll = 0.0
            action.pitch = 0.1  # Very slight pitch (10% of max)
            action.yaw_rate = 0.0
            action.thrust = 0.1
            test_phase = 3
        elif elapsed < 12:
            # Phase 4: Slight roll right
            action.roll = 0.1  # Very slight roll
            action.pitch = 0.0
            action.yaw_rate = 0.0
            action.thrust = 0.1
            test_phase = 4
        elif elapsed < 15:
            # Phase 5: Very slow spin
            action.roll = 0.0
            action.pitch = 0.0
            action.yaw_rate = 0.2  # Slow yaw (20% of max)
            action.thrust = 0.0  # Maintain altitude
            test_phase = 5
        else:
            # Phase 6: Return to neutral hover
            action.roll = 0.0
            action.pitch = 0.0
            action.yaw_rate = 0.0
            action.thrust = 0.0  # Neutral
            
            # After 18 seconds, restart
            if elapsed >= 18:
                last_phase_switch = current_time
        
        action_pub.publish(action)
        
        # Log every second
        if int(elapsed) != int((elapsed - 0.02)):  # Log roughly once per second
            rospy.logwarn(f"[TEST PUB Phase {test_phase}] roll={action.roll:.2f} pitch={action.pitch:.2f} "
                         f"yaw_rate={action.yaw_rate:.2f} thrust={action.thrust:.2f}")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Test Action Publisher shutting down")
