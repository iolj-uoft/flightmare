#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace viri_estimation {

/// Skew-symmetric matrix such that skew(a) * b == a.cross(b).
Eigen::Matrix3d skew(const Eigen::Vector3d& v);

/// Roll, pitch, yaw (ZYX convention, R = Rz(yaw) Ry(pitch) Rx(roll)) of a rotation matrix.
Eigen::Vector3d rotationToRpy(const Eigen::Matrix3d& R);

/// Rotation about the world z-axis.
Eigen::Matrix3d rotZ(double angle);

/// Wrap an angle to (-pi, pi].
double wrapAngle(double angle);

/**
 * Body -> gravity-aligned (GA) rotation (paper Sec. III-E.2).
 *
 * The GA frame shares the chaser's yaw but has roll and pitch removed:
 *   R_GA = Rz(yaw)^T * R_WB
 *
 * @param q_world_body  chaser attitude (body -> world) from odometry
 */
Eigen::Matrix3d derotationMatrix(const Eigen::Quaterniond& q_world_body);

/**
 * Euler yaw rate from body rates (paper eq. 7):
 *   psi_dot = (w_y sin(phi) + w_z cos(phi)) / cos(theta)
 * Falls back to w_z when |cos(theta)| < 1e-3 (gimbal lock).
 */
double eulerYawRate(double roll, double pitch, const Eigen::Vector3d& w_body);

}  // namespace viri_estimation
