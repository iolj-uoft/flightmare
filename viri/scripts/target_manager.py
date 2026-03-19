#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Bool, Empty
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry

class TargetManager:
    def __init__(self):
        rospy.init_node('target_manager', anonymous=True)
        
        # Publishers for the target drone's autopilot
        self.arm_pub = rospy.Publisher('/target_drone/bridge/arm', Bool, queue_size=1)
        self.start_pub = rospy.Publisher('/target_drone/autopilot/start', Empty, queue_size=1)
        # self.pose_pub = rospy.Publisher('/target_drone/autopilot/pose_command', PoseStamped, queue_size=1)
        # Publish velocity commands for smooth control
        self.vel_pub = rospy.Publisher('/target_drone/autopilot/velocity_command', TwistStamped, queue_size=1)
        
        self.has_odometry = False
        rospy.Subscriber('/target_drone/ground_truth/odometry', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        self.has_odometry = True

    def run(self):
        rospy.loginfo("Target Manager waiting for Gazebo to unpause...")
        
        # 1. Wait until Gazebo unpauses (when you click Start in the GUI for the interceptor)
        while not rospy.is_shutdown() and not self.has_odometry:
            rospy.sleep(0.1)
            
        rospy.loginfo("Odometry received! Waking up the target drone...")
        rospy.sleep(0.5)

        # 2. Arm and Start the hover controller
        self.arm_pub.publish(Bool(True))
        rospy.sleep(0.1)
        self.start_pub.publish(Empty())
        
        # Give it a short moment to enter hover
        rospy.sleep(0.5) 

        rospy.loginfo("Commencing evasive figure-eight maneuver...")
        
        # 3. Publish a continuous evasive velocity at 20Hz (smooth)
        rate = rospy.Rate(20) 
        start_time = rospy.get_time()

        while not rospy.is_shutdown():
            t = rospy.get_time() - start_time
            
            # Figure-eight parameters
            amplitude = 3.0 
            speed = 2.0 
            
            # Compute desired velocity for figure-eight (smooth)
            vx = amplitude * speed * math.cos(speed * t)
            vy = amplitude * speed * (math.cos(2 * speed * t) - math.sin(2 * speed * t)) * 0.5
            vz = -0.5 * speed * math.sin(speed * t) * 0.5  # small vertical oscillation velocity

            cmd = TwistStamped()
            cmd.header.stamp = rospy.Time.now()
            cmd.twist.linear.x = vx
            cmd.twist.linear.y = vy
            cmd.twist.linear.z = vz
            cmd.twist.angular.z = 0.0

            self.vel_pub.publish(cmd)
            rate.sleep()

if __name__ == '__main__':
    try:
        manager = TargetManager()
        manager.run()
    except rospy.ROSInterruptException:
        pass