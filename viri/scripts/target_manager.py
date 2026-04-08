#!/usr/bin/env python3
import rospy
import math
import random
from std_msgs.msg import Bool, Empty, String
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry

class TargetManager:
    def __init__(self):
        rospy.init_node('target_manager', anonymous=True)

        # target namespace (set in launch or via rosparam)
        self.target_name = rospy.get_param("~target_name", "target_drone").strip('/')
        ns = '/' + self.target_name
        # Publishers for the target drone's autopilot (namespaced)
        self.arm_pub = rospy.Publisher(ns + '/bridge/arm', Bool, queue_size=1)
        self.start_pub = rospy.Publisher(ns + '/autopilot/start', Empty, queue_size=1)
        # Publish velocity commands for smooth control
        self.vel_pub = rospy.Publisher(ns + '/autopilot/velocity_command', TwistStamped, queue_size=1)
         
        self.has_odometry = False
        rospy.Subscriber(ns + '/ground_truth/odometry', Odometry, self.odom_callback)

        rospy.loginfo("TargetManager will publish to namespace: %s", ns)
        rospy.loginfo("  arm -> %s/bridge/arm", ns)
        rospy.loginfo("  start -> %s/autopilot/start", ns)
        rospy.loginfo("  vel  -> %s/autopilot/velocity_command", ns)

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
        self.drift_y_speed = rospy.get_param("~drift_y_speed", 4.0)   # m/s base speed
        self.drift_y_noise = rospy.get_param("~drift_y_noise", 0.5)   # m/s noise amplitude

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
            vx = A * omega * math.cos(omega * t)
            vy = 0.0
            vz = 0.0
            return vx, vy, vz

        if self.active_traj == "back_and_forth_y":
            A = self.baf_amplitude
            f = self.baf_speed
            omega = 2.0 * math.pi * f
            vx = 0.0
            self.drift_y_speed + random.uniform(-self.drift_y_noise, self.drift_y_noise)
            vz = 0.0
            return vx, vy, vz

        if self.active_traj == "drift_y":
            # +y for 10s, then -y for 10s, repeat
            direction = 1.0 if int(t / 10.0) % 2 == 0 else -1.0
            vx = random.uniform(-self.drift_y_noise, self.drift_y_noise)
            vy = direction * self.drift_y_speed
            return 0.0, vy, 0.0

        # hover (default)
        return 0.0, 0.0, self.hover_vz

    def run(self):
        rospy.loginfo("Target Manager waiting for Gazebo to unpause...")
        
        # 1. Wait until Gazebo unpauses / odom starts
        while not rospy.is_shutdown() and not self.has_odometry:
            rospy.sleep(0.1)
            
        rospy.loginfo("Odometry received! Waking up the target drone...")
        rospy.sleep(0.5)

        # 2. Arm the bridge (publish repeatedly — single message can be dropped
        #    if rpg_rotors_interface isn't subscribed yet)
        rospy.sleep(1.0)  # wait for rpg_rotors_interface to be ready
        rospy.loginfo("Arming target drone...")
        for _ in range(20):
            self.arm_pub.publish(Bool(True))
            rospy.sleep(0.05)

        # 3. Send autopilot/start (repeat a few times to ensure delivery)
        rospy.sleep(0.5)
        rospy.loginfo("Sending autopilot/start to target...")
        for _ in range(5):
            self.start_pub.publish(Empty())
            rospy.sleep(0.1)

        # Give autopilot time to reach HOVER before velocity commands begin
        rospy.sleep(3.0)

        rospy.loginfo("Starting trajectory: %s", self.active_traj)
        
        # 3. Publish continuous velocity at configured rate
        rate_hz = rospy.get_param("~cmd_rate", 50)
        rate = rospy.Rate(rate_hz)
        start_time = rospy.get_time()

        while not rospy.is_shutdown():
            # -- New: poll the runtime param and switch via simple if/elif --
            requested = rospy.get_param("~" + self.param_name, self.active_traj).strip().lower()
            if requested in ("figure_eight", "back_and_forth", "back_and_forth_y", "drift_y", "hover"):
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