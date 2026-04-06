#!/usr/bin/env python3
"""
Simple diagnostic test: publish a SINGLE action with thrust slightly above hover.
This helps isolate whether the thrust calculation is correct.
"""

import rospy
from viri.msg import RLAction
import time

rospy.init_node('thrust_diagnostic')
pub = rospy.Publisher('/chaser_drone/rl_action', RLAction, queue_size=1)

rospy.loginfo("Waiting 3 seconds for system to initialize...")
time.sleep(3)

# Send a single simple command: neutral attitude, mild thrust
action = RLAction()
action.roll = 0.0
action.pitch = 0.0
action.yaw_rate = 0.0
action.thrust = 0.2  # 20% above hover = 7.16 + 0.2*5 = 8.16 m/s²

rospy.loginfo(f"Publishing test action: thrust={action.thrust} (should produce ~1 m/s² climb)")

# Publish continuously for 10 seconds
for i in range(500):  # 10 seconds at 50 Hz
    pub.publish(action)
    time.sleep(0.02)
    if i % 50 == 0:  # Log every second
        rospy.loginfo(f"[{i//50}s] Publishing: roll={action.roll} pitch={action.pitch} "
                     f"yaw_rate={action.yaw_rate} thrust={action.thrust}")

rospy.loginfo("Diagnostic test complete. Drone should have climbed ~10 meters.")
