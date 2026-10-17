#!/usr/bin/env python3
"""Publish static TFs for the stereo rig from the same config the renderer uses.

    base_link -> camera_{left,right}          translation from cameras.yaml, pitched up by tilt_deg
    camera_*  -> camera_*_optical             REP-103 body -> optical (z forward, x right, y down)
"""
import math

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def make_tf(parent, child, xyz, quat):
    t = TransformStamped()
    t.header.stamp = rospy.Time(0)
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
    (t.transform.rotation.x, t.transform.rotation.y,
     t.transform.rotation.z, t.transform.rotation.w) = quat
    return t


def main():
    rospy.init_node("camera_tf_publisher")
    cams = rospy.get_param("~cameras")
    body = cams.get("body_frame", "chaser_drone/base_link")
    # Positive REP-103 pitch is nose-down, so an upward tilt is a negative pitch.
    q_mount = quat_from_rpy(0.0, -math.radians(cams.get("tilt_deg", 15.0)), 0.0)
    q_optical = quat_from_rpy(-math.pi / 2, 0.0, -math.pi / 2)

    transforms = []
    for side in ("left", "right"):
        c = cams[side]
        transforms.append(make_tf(body, c["frame"], c["position"], q_mount))
        transforms.append(make_tf(c["frame"], c["optical_frame"], (0.0, 0.0, 0.0), q_optical))

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(transforms)
    rospy.loginfo("camera_tf_publisher: published %d static transforms (tilt %.1f deg)",
                  len(transforms), cams.get("tilt_deg", 15.0))
    rospy.spin()


if __name__ == "__main__":
    main()
