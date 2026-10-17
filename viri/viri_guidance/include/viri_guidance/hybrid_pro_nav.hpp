#pragma once

#include "viri_guidance/pro_nav.hpp"

namespace viri_guidance {

struct HybridParams {
  double heading_threshold = 15.0 * M_PI / 180.0;  // switch to heading mode at |eps| >= this
  double hysteresis = 0.0;     // return to ProNav at |eps| < threshold - hysteresis (paper: 0)
  double yaw_gain = 4.0;       // K_psi [1/s]
  double max_yaw_rate = M_PI;  // [rad/s]
};

/**
 * Hybrid heading / ProNav law (paper Sec. III-G.2, eq. 30-31).
 *
 * The azimuth error is measured in the GA frame, whose yaw equals the drone heading, so
 *   eps_psi = atan2(r_y, r_x)       (== atan2 in world - psi_drone)
 * |eps_psi| >= threshold -> HEADING: roll = pitch = 0, yaw_rate = clamp(K_psi eps_psi, +-max),
 *                                    thrust = g (hover)
 * otherwise              -> pure ProNav.
 */
class HybridProNav : public GuidanceLaw {
 public:
  HybridProNav(const ProNavParams& pn_params = ProNavParams(),
               const HybridParams& params = HybridParams())
      : pro_nav_(pn_params), params_(params) {}

  GuidanceOutput compute(const GuidanceInput& in) override;
  void reset() override { mode_ = GuidanceMode::kProNav; }
  std::string name() const override { return "hybrid"; }

  GuidanceMode mode() const { return mode_; }

 private:
  ProNav pro_nav_;
  HybridParams params_;
  GuidanceMode mode_ = GuidanceMode::kProNav;
};

}  // namespace viri_guidance
