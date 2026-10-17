#pragma once

#include "viri_guidance/guidance_law.hpp"

namespace viri_guidance {

struct ProNavParams {
  double navigation_gain = 4.0;   // N
  double speed_gain = 1.5;        // Kp of the forward speed loop
  double desired_closing_speed = 1.5;  // Vd [m/s]
  double gravity = 9.81;
  double min_range = 0.05;        // below this the LOS is undefined -> hover [m]
  AttitudeLimits limits;
};

/**
 * Pure proportional navigation (paper Sec. III-G.1, eq. 25-29):
 *   v_rel = v_tgt - v_chaser,  Vc = -r_hat . v_rel,  Omega = r x v_rel / |r|^2
 *   a_PN  = N |Vc| (Omega x r_hat)
 *   a_fwd = Kp (Vd - Vc) r_hat
 *   a     = a_PN + a_fwd + [0, 0, g]
 * The yaw rate is zero: heading is not actively controlled.
 */
class ProNav : public GuidanceLaw {
 public:
  explicit ProNav(const ProNavParams& params = ProNavParams()) : params_(params) {}

  GuidanceOutput compute(const GuidanceInput& in) override;
  std::string name() const override { return "pronav"; }

  const ProNavParams& params() const { return params_; }

 private:
  ProNavParams params_;
};

}  // namespace viri_guidance
