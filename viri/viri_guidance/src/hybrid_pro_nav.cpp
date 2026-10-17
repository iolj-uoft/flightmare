#include "viri_guidance/hybrid_pro_nav.hpp"

#include <algorithm>
#include <cmath>

namespace viri_guidance {

GuidanceOutput HybridProNav::compute(const GuidanceInput& in) {
  // ProNav terms are always evaluated so the status output stays informative.
  GuidanceOutput out = pro_nav_.compute(in);

  const double abs_eps = std::abs(out.azimuth_error);  // eq. 30, already in (-pi, pi]
  if (mode_ == GuidanceMode::kProNav && abs_eps >= params_.heading_threshold) {
    mode_ = GuidanceMode::kHeading;
  } else if (mode_ == GuidanceMode::kHeading &&
             abs_eps < params_.heading_threshold - params_.hysteresis) {
    mode_ = GuidanceMode::kProNav;
  }
  out.mode = mode_;

  if (mode_ == GuidanceMode::kHeading) {
    const double g = pro_nav_.params().gravity;
    const AttitudeLimits& lim = pro_nav_.params().limits;
    out.accel = Eigen::Vector3d(0.0, 0.0, g);
    out.command.roll = 0.0;
    out.command.pitch = 0.0;
    out.command.thrust = std::clamp(g, lim.thrust_min, lim.thrust_max);
    out.command.yaw_rate = std::clamp(params_.yaw_gain * out.azimuth_error,  // eq. 31
                                      -params_.max_yaw_rate, params_.max_yaw_rate);
  }
  return out;
}

}  // namespace viri_guidance
