#include "viri_guidance/pro_nav.hpp"

#include <algorithm>
#include <cmath>

namespace viri_guidance {

GuidanceOutput ProNav::compute(const GuidanceInput& in) {
  GuidanceOutput out;
  out.mode = GuidanceMode::kProNav;
  out.range = in.r.norm();
  out.azimuth_error = std::atan2(in.r.y(), in.r.x());

  const Eigen::Vector3d gravity_comp(0.0, 0.0, params_.gravity);

  if (out.range < params_.min_range) {
    // Line of sight undefined: hold hover.
    out.accel = gravity_comp;
    out.command = mapAccelerationToAttitude(out.accel, in.R_GB, params_.limits);
    return out;
  }

  const Eigen::Vector3d r_hat = in.r / out.range;
  const Eigen::Vector3d v_rel = in.v_tgt - in.v_chaser;

  out.closing_speed = -r_hat.dot(v_rel);                        // eq. 25
  out.los_rate = in.r.cross(v_rel) / (out.range * out.range);   // eq. 26

  const Eigen::Vector3d a_pn =
      params_.navigation_gain * std::abs(out.closing_speed) * out.los_rate.cross(r_hat);  // eq. 27
  const Eigen::Vector3d a_fwd =
      params_.speed_gain * (params_.desired_closing_speed - out.closing_speed) * r_hat;   // eq. 28

  out.accel = a_pn + a_fwd + gravity_comp;                      // eq. 29
  out.command = mapAccelerationToAttitude(out.accel, in.R_GB, params_.limits);
  out.command.yaw_rate = 0.0;
  return out;
}

}  // namespace viri_guidance
