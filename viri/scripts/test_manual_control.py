#!/usr/bin/env python3
"""
Manual test to verify chaser drone responds to direct control commands.
This script:
1. Publishes an ARM signal
2. Publishes a simple attitude + thrust control command
3. Monitors the drone's response
"""

import rospy
from quadrotor_msgs.msg import ControlCommand
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import time
from geometry_msgs.msg import Quaternion
import math

class ManualControlTester:
    def __init__(self):
        rospy.init_node('manual_control_tester')
        
        # Publishers
        self.arm_pub = rospy.Publisher('/chaser_drone/bridge/arm', Bool, queue_size=1)
        self.control_pub = rospy.Publisher('/chaser_drone/control_command', ControlCommand, queue_size=1)
        
        # Subscribers
        self.odom_sub = rospy.Subscriber('/chaser_drone/ground_truth/odometry', Odometry, self.odom_callback)
        
        self.initial_z = None
        self.current_z = None
        self.max_z = None
        
        rospy.loginfo("Manual Control Tester initialized")
        
    def odom_callback(self, msg):
        self.current_z = msg.pose.pose.position.z
        if self.initial_z is None:
            self.initial_z = self.current_z
        if self.max_z is None or self.current_z > self.max_z:
            self.max_z = self.current_z
        rospy.loginfo(f"Chaser Z: {self.current_z:.3f}, Initial: {self.initial_z:.3f}, Max: {self.max_z:.3f}")
    
    def send_arm(self):
        """Send ARM signal"""
        rospy.loginfo("Sending ARM signal...")
        arm_msg = Bool()
        arm_msg.data = True
        for i in range(20):
            self.arm_pub.publish(arm_msg)
            time.sleep(0.05)
        rospy.loginfo("ARM signal sent!")
        
    def send_control_command(self, roll=0, pitch=0, yaw_rate=0, thrust=1.5):
        """Send a single control command"""
        cmd = ControlCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "world"
        cmd.control_mode = 1  # ATTITUDE mode
        cmd.armed = True
        cmd.collective_thrust = thrust  # m/s^2
        
        # Convert roll/pitch/yaw to quaternion (ZYX convention)
        cy = math.cos(yaw_rate * 0.5)
        sy = math.sin(yaw_rate * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        cmd.orientation.w = cr * cp * cy + sr * sp * sy
        cmd.orientation.x = sr * cp * cy - cr * sp * sy
        cmd.orientation.y = cr * sp * cy + sr * cp * sy
        cmd.orientation.z = cr * cp * sy - sr * sp * cy
        
        # Optional: body rate feedforward
        cmd.bodyrates.x = 0.0
        cmd.bodyrates.y = 0.0
        cmd.bodyrates.z = 0.0  # No yaw rate feedforward for this test
        
        self.control_pub.publish(cmd)
        rospy.loginfo(f"Sent control command: thrust={thrust}")
        
    def run_test(self):
        """Run the test sequence"""
        rospy.loginfo("=== STARTING MANUAL CONTROL TEST ===")
        rospy.loginfo("Waiting 2 seconds for system to initialize...")
        time.sleep(2)
        
        # Step 1: Arm
        rospy.loginfo("\n--- Step 1: Arming drone ---")
        self.send_arm()
        time.sleep(1)
        
        # Step 2: Send mild thrust command
        rospy.loginfo("\n--- Step 2: Sending mild thrust (11 m/s^2) ---")
        for i in range(50):  # Send for 1 second at 50Hz
            self.send_control_command(roll=0, pitch=0, yaw_rate=0, thrust=11.0)
            time.sleep(0.02)
        
        # Step 3: Monitor for 2 seconds
        rospy.loginfo("\n--- Step 3: Monitoring for 2 seconds ---")
        time.sleep(2)
        
        # Step 4: Test with more thrust
        rospy.loginfo("\n--- Step 4: Sending higher thrust (15 m/s^2) ---")
        for i in range(100):  # Send for 2 seconds at 50Hz
            self.send_control_command(roll=0, pitch=0, yaw_rate=0, thrust=15.0)
            time.sleep(0.02)
        
        # Step 5: Final monitoring
        rospy.loginfo("\n--- Step 5: Final monitoring for 3 seconds ---")
        time.sleep(3)
        
        # Results
        rospy.loginfo("\n=== TEST RESULTS ===")
        if self.max_z is not None and self.initial_z is not None:
            height_gain = self.max_z - self.initial_z
            rospy.loginfo(f"Initial Z: {self.initial_z:.3f}")
            rospy.loginfo(f"Max Z reached: {self.max_z:.3f}")
            rospy.loginfo(f"Height gain: {height_gain:.3f} m")
            
            if height_gain > 0.1:
                rospy.loginfo("SUCCESS! Drone responded to control commands!")
            else:
                rospy.logwarn("FAILURE! Drone did not respond to control commands")
        else:
            rospy.logwarn("No odometry data received!")

if __name__ == '__main__':
    tester = ManualControlTester()
    try:
        tester.run_test()
    except rospy.ROSInterruptException:
        pass
