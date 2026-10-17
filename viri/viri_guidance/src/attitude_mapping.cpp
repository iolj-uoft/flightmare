#include "viri_guidance/attitude_mapping.hpp"

#include <algorithm>
#include <cmath>

namespace viri_guidance {

AttitudeCommand mapAccelerationToAttitude(const Eigen::Vector3d& a_total_ga,
                                          const Eigen::Matrix3d& R_GB,
                                          const AttitudeLimits& limits) {
  AttitudeCommand cmd;
  const Eigen::Vector3d& a = a_total_ga;

  cmd.pitch = std::atan2(a.x(), a.z());
  cmd.roll = std::atan2(-a.y(), std::hypot(a.x(), a.z()));
  cmd.roll = std::clamp(cmd.roll, -limits.max_tilt, limits.max_tilt);
  cmd.pitch = std::clamp(cmd.pitch, -limits.max_tilt, limits.max_tilt);

  const Eigen::Vector3d z_body_current = R_GB.col(2);
  cmd.thrust = std::clamp(a.dot(z_body_current), limits.thrust_min, limits.thrust_max);
  return cmd;
}

Eigen::Quaterniond attitudeToQuaternion(double roll, double pitch, double yaw) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
}

Eigen::Vector3d attitudeErrorToBodyRates(const Eigen::Quaterniond& q_current,
                                         const Eigen::Quaterniond& q_desired, double k_rp,
                                         double k_yaw, const Eigen::Vector3d& feedforward) {
  const Eigen::Quaterniond q_e = q_current.normalized().inverse() * q_desired.normalized();
  const double s = q_e.w() >= 0.0 ? 2.0 : -2.0;
  return Eigen::Vector3d(s * k_rp * q_e.x(), s * k_rp * q_e.y(), s * k_yaw * q_e.z()) +
         feedforward;
}

}  // namespace viri_guidance
