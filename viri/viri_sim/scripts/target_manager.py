#!/usr/bin/env python3
import rospy
import math
import random
from std_msgs.msg import Bool, Empty
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import AutopilotFeedback

TRAJECTORIES = ("hover", "back_and_forth", "figure_eight", "wander")


class WanderGenerator:
    """Smooth random cruise with bounded speed, acceleration and turn rate.

    Horizontal motion is a unicycle: speed tracks a randomly re-drawn reference under a
    tangential acceleration limit, and turn rate is an Ornstein-Uhlenbeck process clamped
    so the total (tangential + centripetal) acceleration stays below max_accel. Outside the
    geofence the turn rate steers back toward the centre. Altitude tracks a randomly
    re-drawn reference inside [z_min, z_max].

    The random process is stepped with a fixed dt, so a given seed gives the same sequence
    of commands regardless of loop timing (the path still depends on how the vehicle flies).
    """

    def __init__(self, p, centre_xy, rng):
        self.p = p
        self.cx, self.cy = centre_xy
        self.rng = rng
        self.speed = 0.0
        self.heading = rng.uniform(-math.pi, math.pi)
        self.turn_rate = 0.0
        self.returning = False
        self.speed_ref = self.z_ref = None
        self.time_to_redraw = 0.0

    def _redraw(self):
        p = self.p
        self.speed_ref = self.rng.uniform(p["v_min"], p["v_max"])
        self.z_ref = self.rng.uniform(p["z_min"], p["z_max"])
        self.time_to_redraw = self.rng.uniform(p["redraw_min"], p["redraw_max"])

    def step(self, dt, x, y, z):
        p = self.p
        self.time_to_redraw -= dt
        if self.time_to_redraw <= 0.0:
            self._redraw()

        # Speed: approach the reference under the tangential acceleration limit
        a_t = max(-p["a_tangential"], min(p["a_tangential"], (self.speed_ref - self.speed) / dt))
        self.speed += a_t * dt

        # Turn rate: OU process, or steer home near the geofence. Turning back takes about one
        # turn radius of outward travel, so heading outward the turn starts that far inside.
        dx, dy = self.cx - x, self.cy - y
        heading_err = math.atan2(math.sin(math.atan2(dy, dx) - self.heading),
                                 math.cos(math.atan2(dy, dx) - self.heading))
        turn_radius = self.speed / p["max_turn_rate"]
        dist = math.hypot(dx, dy)
        if dist > p["fence_radius"] or \
                (dist > p["fence_radius"] - turn_radius and abs(heading_err) > math.pi / 2):
            self.returning = True
        if self.returning:
            self.turn_rate = p["fence_gain"] * heading_err
            if abs(heading_err) < math.radians(20.0):
                self.returning = False
        else:
            tau = p["turn_tau"]
            sigma = p["turn_std"] * math.sqrt(2.0 / tau)  # stationary std = turn_std
            self.turn_rate += -self.turn_rate / tau * dt + sigma * math.sqrt(dt) * self.rng.gauss(0, 1)

        a_n_max = math.sqrt(max(p["max_accel"] ** 2 - a_t ** 2, 0.0))
        w_max = min(p["max_turn_rate"], a_n_max / max(self.speed, 0.1))
        self.turn_rate = max(-w_max, min(w_max, self.turn_rate))
        self.heading = math.atan2(math.sin(self.heading + self.turn_rate * dt),
                                  math.cos(self.heading + self.turn_rate * dt))

        vz = max(-p["vz_max"], min(p["vz_max"], p["z_gain"] * (self.z_ref - z)))
        return self.speed * math.cos(self.heading), self.speed * math.sin(self.heading), vz


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

        self.position = None
        rospy.Subscriber(ns + '/ground_truth/odometry', Odometry, self.odom_callback)
        self.autopilot_state = None
        rospy.Subscriber(ns + '/autopilot/feedback', AutopilotFeedback, self.feedback_callback)

        rospy.loginfo("TargetManager will publish to namespace: %s", ns)
        rospy.loginfo("  arm -> %s/bridge/arm", ns)
        rospy.loginfo("  start -> %s/autopilot/start", ns)
        rospy.loginfo("  vel  -> %s/autopilot/velocity_command", ns)

        # Trajectory switching via ROS param (polled each loop). Default can be overridden at runtime:
        # rosparam set /target_manager/active_trajectory "figure_eight"
        self.active_traj = rospy.get_param("~default_trajectory", "hover")
        self.param_name = rospy.get_param("~param_name", "active_trajectory")

        # Trajectory params (tunable via rosparam, see config/target.yaml)
        self.fe_amplitude = rospy.get_param("~figure_eight_amplitude", 6.0)   # [m]
        self.fe_speed = rospy.get_param("~figure_eight_speed", 0.5)           # [rad/s]
        self.baf_amplitude = rospy.get_param("~back_and_forth_amplitude", 4.0)  # [m]
        self.baf_speed = rospy.get_param("~back_and_forth_speed", 0.1)        # oscillation frequency [Hz]
        self.hover_vz = rospy.get_param("~hover_vz", 0.0)
        self.ramp_time = rospy.get_param("~ramp_time", 2.0)  # [s] smooth start for the periodic trajectories

        self.wander_params = {
            "v_min": rospy.get_param("~wander/v_min", 2.0),                   # [m/s]
            "v_max": rospy.get_param("~wander/v_max", 5.0),                   # [m/s]
            "a_tangential": rospy.get_param("~wander/a_tangential", 1.0),     # [m/s^2]
            "max_accel": rospy.get_param("~wander/max_accel", 4.9),           # total horizontal [m/s^2]
            "max_turn_rate": math.radians(rospy.get_param("~wander/max_turn_rate_deg", 30.0)),
            "turn_std": math.radians(rospy.get_param("~wander/turn_std_deg", 15.0)),
            "turn_tau": rospy.get_param("~wander/turn_tau", 2.0),             # [s]
            "redraw_min": rospy.get_param("~wander/redraw_min", 3.0),         # [s]
            "redraw_max": rospy.get_param("~wander/redraw_max", 8.0),         # [s]
            "z_min": rospy.get_param("~wander/z_min", 1.0),                   # [m]
            "z_max": rospy.get_param("~wander/z_max", 3.0),                   # [m]
            "z_gain": rospy.get_param("~wander/z_gain", 0.5),                 # [1/s]
            "vz_max": rospy.get_param("~wander/vz_max", 0.5),                 # [m/s]
            "fence_radius": rospy.get_param("~wander/fence_radius", 25.0),    # [m] around take-off point
            "fence_gain": rospy.get_param("~wander/fence_gain", 1.0),         # [1/s]
        }
        self.wander_seed = rospy.get_param("~wander/seed", 0)
        self.wander = None

        rospy.loginfo("TargetManager initialized. default_trajectory=%s", self.active_traj)

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        self.position = (p.x, p.y, p.z)

    def feedback_callback(self, msg):
        self.autopilot_state = msg.autopilot_state

    def compute_velocity(self, t, dt):
        if self.active_traj == "wander":
            if self.wander is None:
                self.wander = WanderGenerator(self.wander_params, self.position[:2],
                                              random.Random(self.wander_seed))
            return self.wander.step(dt, *self.position)

        # Periodic trajectories start at full speed at t=0; fade them in.
        x = min(t / self.ramp_time, 1.0) if self.ramp_time > 0.0 else 1.0
        ramp = x * x * (3.0 - 2.0 * x)

        if self.active_traj == "figure_eight":
            A = self.fe_amplitude
            s = self.fe_speed
            vx = A * s * math.cos(s * t)
            vy = A * s * (math.cos(2 * s * t) - math.sin(2 * s * t)) * 0.5
            vz = -0.5 * s * math.sin(s * t) * 0.5
            return ramp * vx, ramp * vy, ramp * vz

        if self.active_traj == "back_and_forth":
            A = self.baf_amplitude
            f = self.baf_speed
            omega = 2.0 * math.pi * f
            # smooth sinusoidal position -> velocity is cosine
            vx = A * omega * math.cos(omega * t)
            return ramp * vx, 0.0, 0.0

        # hover (default)
        return 0.0, 0.0, self.hover_vz

    def run(self):
        rospy.loginfo("Target Manager waiting for Gazebo to unpause...")

        # 1. Wait until Gazebo unpauses / odom starts
        while not rospy.is_shutdown() and self.position is None:
            rospy.sleep(0.1)

        rospy.loginfo("Odometry received! Waking up the target drone...")

        # 2. Wait for the bridge and autopilot to be connected: these topics aren't
        # latched, so a publish before the connection is up is silently dropped.
        while not rospy.is_shutdown() and (self.arm_pub.get_num_connections() == 0 or
                                           self.start_pub.get_num_connections() == 0):
            rospy.sleep(0.1)

        # 3. Arm and start, re-sending until the autopilot has finished take-off.
        # Velocity commands are ignored by the autopilot outside HOVER, so the
        # trajectory clock must not start before then.
        last_send = None
        while not rospy.is_shutdown() and self.autopilot_state != AutopilotFeedback.HOVER:
            now = rospy.get_time()
            if self.autopilot_state in (None, AutopilotFeedback.OFF) and \
                    (last_send is None or now - last_send > 1.0):
                self.arm_pub.publish(Bool(True))
                rospy.sleep(0.1)
                self.start_pub.publish(Empty())
                last_send = now
            rospy.sleep(0.1)

        rospy.loginfo("Starting trajectory: %s", self.active_traj)

        # 4. Publish continuous velocity at configured rate
        rate_hz = rospy.get_param("~cmd_rate", 50)
        dt = 1.0 / rate_hz
        rate = rospy.Rate(rate_hz)
        start_time = rospy.get_time()

        while not rospy.is_shutdown():
            # Poll the runtime param; a switch restarts the clock so the new trajectory ramps in
            requested = rospy.get_param("~" + self.param_name, self.active_traj).strip().lower()
            if requested in TRAJECTORIES and requested != self.active_traj:
                rospy.loginfo("Switched trajectory to: %s (via param)", requested)
                self.active_traj = requested
                self.wander = None
                start_time = rospy.get_time()

            t = rospy.get_time() - start_time

            vx, vy, vz = self.compute_velocity(t, dt)

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
