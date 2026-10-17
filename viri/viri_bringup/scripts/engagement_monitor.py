#!/usr/bin/env python3
"""Ground-truth engagement monitor.

* Declares an intercept when the ground-truth centre-to-centre distance drops below
  ``intercept_radius`` and publishes it (latched) on ``intercepted_topic`` so guidance stops.
* Logs one CSV row per ``1/log_rate`` s with the ground-truth relative position expressed in the
  chaser's gravity-aligned (GA) frame next to the LTVKF estimate and its 1-sigma bounds, plus
  world positions and the guidance command. ``plot_results.py`` turns the CSV into the paper's
  RMSE table and figures.
"""
import csv
import math
import os
import time

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from viri_msgs.msg import GuidanceStatus, RelativeState

COLUMNS = [
    "t", "gt_x", "gt_y", "gt_z", "est_x", "est_y", "est_z", "sig_x", "sig_y", "sig_z",
    "est_vx", "est_vy", "est_vz", "range_gt",
    "chaser_x", "chaser_y", "chaser_z", "target_x", "target_y", "target_z",
    "mode", "roll_cmd", "pitch_cmd", "yaw_rate_cmd", "thrust_cmd",
]


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class EngagementMonitor:
    def __init__(self):
        self.radius = rospy.get_param("~intercept_radius", 1.0)
        self.chaser = None
        self.target = None
        self.estimate = None
        self.status = None
        self.t0 = None
        self.engaged_at = None
        self.intercepted = False

        self.intercept_pub = rospy.Publisher(
            rospy.get_param("~intercepted_topic", "/engagement/intercepted"), Bool,
            queue_size=1, latch=True)
        self.intercept_pub.publish(Bool(False))

        rospy.Subscriber(rospy.get_param("~chaser_odometry_topic", "/chaser_drone/ground_truth/odometry"),
                         Odometry, self._chaser_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param("~target_odometry_topic", "/target_drone/ground_truth/odometry"),
                         Odometry, self._target_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param("~relative_state_topic", "/chaser_drone/relative_state"),
                         RelativeState, self._estimate_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param("~guidance_status_topic", "/chaser_drone/guidance/status"),
                         GuidanceStatus, self._status_cb, queue_size=1)

        self.writer = None
        self.log_file = None
        log_dir = rospy.get_param("~log_dir", "")
        if log_dir:
            os.makedirs(os.path.expanduser(log_dir), exist_ok=True)
            label = rospy.get_param("~run_label", "run")
            path = os.path.join(os.path.expanduser(log_dir),
                                "%s_%s.csv" % (label, time.strftime("%Y%m%d_%H%M%S")))
            self.log_file = open(path, "w", newline="")
            self.writer = csv.writer(self.log_file)
            self.writer.writerow(COLUMNS)
            rospy.loginfo("engagement_monitor: logging to %s", path)
        rospy.on_shutdown(self._close)

        rate = rospy.get_param("~log_rate", 50.0)
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate), self._tick)

    def _chaser_cb(self, msg):
        self.chaser = msg

    def _target_cb(self, msg):
        self.target = msg

    def _estimate_cb(self, msg):
        self.estimate = msg

    def _status_cb(self, msg):
        self.status = msg
        if self.engaged_at is None and msg.mode in (GuidanceStatus.PRONAV, GuidanceStatus.HEADING):
            self.engaged_at = rospy.Time.now()
            rospy.loginfo("engagement_monitor: guidance engaged (%s)", msg.law)

    def _tick(self, _event):
        if self.chaser is None or self.target is None:
            return
        now = rospy.Time.now()
        if self.t0 is None:
            self.t0 = now

        pc = self.chaser.pose.pose.position
        pt = self.target.pose.pose.position
        dx, dy, dz = pt.x - pc.x, pt.y - pc.y, pt.z - pc.z
        rng = math.sqrt(dx * dx + dy * dy + dz * dz)

        # Ground-truth relative position in the GA frame: rotate by -yaw
        yaw = yaw_from_quaternion(self.chaser.pose.pose.orientation)
        c, s = math.cos(yaw), math.sin(yaw)
        gt = (c * dx + s * dy, -s * dx + c * dy, dz)

        if not self.intercepted and self.engaged_at is not None and rng < self.radius:
            self.intercepted = True
            self.intercept_pub.publish(Bool(True))
            rospy.loginfo("engagement_monitor: INTERCEPT at t = %.2f s after engagement, |r| = %.2f m",
                          (now - self.engaged_at).to_sec(), rng)

        if self.writer is None:
            return
        nan = float("nan")
        est = (nan,) * 3
        sig = (nan,) * 3
        vel = (nan,) * 3
        if self.estimate is not None and self.estimate.initialized:
            e = self.estimate
            est = (e.p_rel.x, e.p_rel.y, e.p_rel.z)
            vel = (e.v_tgt.x, e.v_tgt.y, e.v_tgt.z)
            sig = tuple(math.sqrt(max(e.covariance[i * 7], 0.0)) for i in range(3))
        st = self.status
        cmd = (st.mode, st.roll_cmd, st.pitch_cmd, st.yaw_rate_cmd, st.thrust_cmd) if st else \
            (GuidanceStatus.IDLE, nan, nan, nan, nan)
        self.writer.writerow(
            ["%.3f" % (now - self.t0).to_sec()] + ["%.4f" % v for v in gt + est + sig + vel] +
            ["%.4f" % rng, "%.4f" % pc.x, "%.4f" % pc.y, "%.4f" % pc.z,
             "%.4f" % pt.x, "%.4f" % pt.y, "%.4f" % pt.z, cmd[0]] + ["%.4f" % v for v in cmd[1:]])

    def _close(self):
        if self.log_file:
            self.log_file.close()


def main():
    rospy.init_node("engagement_monitor")
    EngagementMonitor()
    rospy.spin()


if __name__ == "__main__":
    main()
