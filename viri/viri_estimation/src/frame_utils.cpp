#include "viri_estimation/frame_utils.hpp"

#include <algorithm>
#include <cmath>

namespace viri_estimation {

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d S;
  S <<   0.0, -v(2),  v(1),
        v(2),   0.0, -v(0),
       -v(1),  v(0),   0.0;
  return S;
}

Eigen::Vector3d rotationToRpy(const Eigen::Matrix3d& R) {
  const double roll = std::atan2(R(2, 1), R(2, 2));
  const double pitch = std::asin(std::clamp(-R(2, 0), -1.0, 1.0));
  const double yaw = std::atan2(R(1, 0), R(0, 0));
  return Eigen::Vector3d(roll, pitch, yaw);
}

Eigen::Matrix3d rotZ(double angle) {
  return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

double wrapAngle(double angle) {
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle <= 0.0) angle += 2.0 * M_PI;
  return angle - M_PI;
}

Eigen::Matrix3d derotationMatrix(const Eigen::Quaterniond& q_world_body) {
  const Eigen::Matrix3d R_WB = q_world_body.normalized().toRotationMatrix();
  const double yaw = rotationToRpy(R_WB)(2);
  return rotZ(yaw).transpose() * R_WB;
}

double eulerYawRate(double roll, double pitch, const Eigen::Vector3d& w_body) {
  const double cos_pitch = std::cos(pitch);
  if (std::abs(cos_pitch) < 1e-3) return w_body(2);
  return (w_body(1) * std::sin(roll) + w_body(2) * std::cos(roll)) / cos_pitch;
}

}  // namespace viri_estimation
