#pragma once

#include <Eigen/Dense>
#include <string>

#include "viri_guidance/attitude_mapping.hpp"

namespace viri_guidance {

/// Relative engagement state, all vectors in the chaser's gravity-aligned (GA) frame.
struct GuidanceInput {
  Eigen::Vector3d r = Eigen::Vector3d::Zero();         // p_tgt - p_chaser
  Eigen::Vector3d v_tgt = Eigen::Vector3d::Zero();     // target inertial velocity
  Eigen::Vector3d v_chaser = Eigen::Vector3d::Zero();  // chaser inertial velocity
  Eigen::Matrix3d R_GB = Eigen::Matrix3d::Identity();  // current body -> GA rotation (tilt)
};

enum class GuidanceMode { kProNav, kHeading };

struct GuidanceOutput {
  GuidanceMode mode = GuidanceMode::kProNav;
  double range = 0.0;                                   // |r|
  double closing_speed = 0.0;                           // Vc
  double azimuth_error = 0.0;                           // eps_psi
  Eigen::Vector3d los_rate = Eigen::Vector3d::Zero();   // Omega
  Eigen::Vector3d accel = Eigen::Vector3d::Zero();      // a_total incl. gravity (GA)
  AttitudeCommand command;
};

/// Common interface so the node can swap laws via a parameter.
class GuidanceLaw {
 public:
  virtual ~GuidanceLaw() = default;
  virtual GuidanceOutput compute(const GuidanceInput& in) = 0;
  virtual void reset() {}
  virtual std::string name() const = 0;
};

}  // namespace viri_guidance
