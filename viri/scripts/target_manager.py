#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Bool, Empty, String
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry

class TargetManager:
    def __init__(self):
        rospy.init_node('target_manager', anonymous=True)
        
        # Publishers for the target drone's autopilot
        self.arm_pub = rospy.Publisher('/target_drone/bridge/arm', Bool, queue_size=1)
        self.start_pub = rospy.Publisher('/target_drone/autopilot/start', Empty, queue_size=1)
        # Publish velocity commands for smooth control
        self.vel_pub = rospy.Publisher('/target_drone/autopilot/velocity_command', TwistStamped, queue_size=1)
        
        self.has_odometry = False
        rospy.Subscriber('/target_drone/ground_truth/odometry', Odometry, self.odom_callback)

        # Trajectory switching via ROS param (polled each loop). Default can be overridden at runtime:
        # rosparam set /target_manager/active_trajectory "figure_eight"
        self.active_traj = rospy.get_param("~default_trajectory", "hover")
        self.param_name = rospy.get_param("~param_name", "active_trajectory")

        # Trajectory params (tunable via rosparam)
        self.fe_amplitude = rospy.get_param("~figure_eight_amplitude", 10.0)
        self.fe_speed = rospy.get_param("~figure_eight_speed", 2.0)
        self.baf_amplitude = rospy.get_param("~back_and_forth_amplitude", 5.0)  # meters
        self.baf_speed = rospy.get_param("~back_and_forth_speed", 0.5)  # oscillation frequency (Hz)
        self.hover_vz = rospy.get_param("~hover_vz", 0.0)

        rospy.loginfo("TargetManager initialized. default_trajectory=%s", self.active_traj)

    def odom_callback(self, msg):
        self.has_odometry = True

    def trajectory_callback(self, msg):
        # kept for backward compatibility but not used in the new param-switching flow
        pass

    def compute_velocity(self, t):
        if self.active_traj == "figure_eight":
            A = self.fe_amplitude
            s = self.fe_speed
            vx = A * s * math.cos(s * t)
            vy = A * s * (math.cos(2 * s * t) - math.sin(2 * s * t)) * 0.5
            vz = -0.5 * s * math.sin(s * t) * 0.5
            return vx, vy, vz

        if self.active_traj == "back_and_forth":
            A = self.baf_amplitude
            f = self.baf_speed
            omega = 2.0 * math.pi * f
            # smooth sinusoidal position -> velocity is cosine
            vx = A * omega * math.cos(omega * t)
            vy = 0.0
            vz = 0.0
            return vx, vy, vz

        # hover (default)
        return 0.0, 0.0, self.hover_vz

    def run(self):
        rospy.loginfo("Target Manager waiting for Gazebo to unpause...")
        
        # 1. Wait until Gazebo unpauses / odom starts
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

        rospy.loginfo("Starting trajectory: %s", self.active_traj)
        
        # 3. Publish continuous velocity at configured rate
        rate_hz = rospy.get_param("~cmd_rate", 50)
        rate = rospy.Rate(rate_hz)
        start_time = rospy.get_time()

        while not rospy.is_shutdown():
            # -- New: poll the runtime param and switch via simple if/elif --
            requested = rospy.get_param("~" + self.param_name, self.active_traj).strip().lower()
            if requested in ("figure_eight", "back_and_forth", "hover"):
                if requested != self.active_traj:
                    rospy.loginfo("Switched trajectory to: %s (via param)", requested)
                self.active_traj = requested
            else:
                # ignore invalid values; keep current trajectory
                pass
            # -----------------------------------------------------------

            t = rospy.get_time() - start_time

            vx, vy, vz = self.compute_velocity(t)

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