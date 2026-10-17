#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "viri_guidance/attitude_mapping.hpp"
#include "viri_guidance/hybrid_pro_nav.hpp"
#include "viri_guidance/pro_nav.hpp"

using namespace viri_guidance;

namespace {

constexpr double kDeg = M_PI / 180.0;

Eigen::Matrix3d rotZ(double a) {
  return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

GuidanceInput makeInput(const Eigen::Vector3d& r, const Eigen::Vector3d& v_tgt,
                        const Eigen::Vector3d& v_chaser) {
  GuidanceInput in;
  in.r = r;
  in.v_tgt = v_tgt;
  in.v_chaser = v_chaser;
  return in;
}

}  // namespace

// ---------------------------------------------------------------- attitude mapping

TEST(AttitudeMapping, HoverIsLevelWithThrustG) {
  const AttitudeCommand c =
      mapAccelerationToAttitude(Eigen::Vector3d(0, 0, 9.81), Eigen::Matrix3d::Identity(), {});
  EXPECT_NEAR(c.roll, 0.0, 1e-12);
  EXPECT_NEAR(c.pitch, 0.0, 1e-12);
  EXPECT_NEAR(c.thrust, 9.81, 1e-12);
}

TEST(AttitudeMapping, ForwardAccelPitchesNoseDownLeftAccelRollsLeft) {
  // REP-103: +x forward, +y left, z up. Positive pitch tilts thrust toward +x,
  // positive roll tilts it toward -y.
  const AttitudeCommand fwd =
      mapAccelerationToAttitude(Eigen::Vector3d(2, 0, 9.81), Eigen::Matrix3d::Identity(), {});
  EXPECT_NEAR(fwd.pitch, std::atan2(2.0, 9.81), 1e-12);
  EXPECT_NEAR(fwd.roll, 0.0, 1e-12);

  const AttitudeCommand left =
      mapAccelerationToAttitude(Eigen::Vector3d(0, 2, 9.81), Eigen::Matrix3d::Identity(), {});
  EXPECT_LT(left.roll, 0.0);
  // The commanded body z-axis must point along the requested acceleration
  const Eigen::Vector3d z_b =
      (Eigen::AngleAxisd(left.pitch, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(left.roll, Eigen::Vector3d::UnitX())) * Eigen::Vector3d::UnitZ();
  EXPECT_LT((z_b - Eigen::Vector3d(0, 2, 9.81).normalized()).norm(), 1e-12);
}

TEST(AttitudeMapping, ClampsTiltAndThrust) {
  const AttitudeCommand c =
      mapAccelerationToAttitude(Eigen::Vector3d(50, -50, 30), Eigen::Matrix3d::Identity(), {});
  EXPECT_NEAR(c.pitch, 40.0 * kDeg, 1e-12);
  EXPECT_NEAR(c.roll, 40.0 * kDeg, 1e-12);
  EXPECT_NEAR(c.thrust, 14.0, 1e-12);

  const AttitudeCommand low =
      mapAccelerationToAttitude(Eigen::Vector3d(0, 0, 2.0), Eigen::Matrix3d::Identity(), {});
  EXPECT_NEAR(low.thrust, 9.0, 1e-12);
}

TEST(AttitudeMapping, ThrustProjectsOntoCurrentBodyZ) {
  const Eigen::Matrix3d tilted =
      Eigen::AngleAxisd(30 * kDeg, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const AttitudeCommand c = mapAccelerationToAttitude(Eigen::Vector3d(0, 0, 12.0), tilted, {});
  EXPECT_NEAR(c.thrust, 12.0 * std::cos(30 * kDeg), 1e-12);
}

TEST(AttitudeMapping, TiltClampHoldsVerticalAcceleration) {
  // 15 m/s^2 lateral wants ~57 deg of pitch; clamped to 40 deg. Plain projection would give
  // 14 * cos(40) = 10.7 m/s^2 vertical (> g) and climb.
  const Eigen::Vector3d a(15.0, 0.0, 9.81);
  const Eigen::Matrix3d at_limit =
      Eigen::AngleAxisd(40 * kDeg, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const AttitudeCommand c = mapAccelerationToAttitude(a, at_limit, {});
  EXPECT_NEAR(c.pitch, 40 * kDeg, 1e-12);
  EXPECT_NEAR(c.thrust * at_limit(2, 2), 9.81, 1e-9);

  // Still banking (10 deg) toward the clamped attitude: vertical is held there too.
  const Eigen::Matrix3d banking =
      Eigen::AngleAxisd(10 * kDeg, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const AttitudeCommand b = mapAccelerationToAttitude(a, banking, {});
  EXPECT_NEAR(b.thrust * banking(2, 2), 9.81, 1e-9);

  // Combined roll + pitch clamp beyond what thrust_max can hold: saturate, never exceed.
  const Eigen::Matrix3d steep = (Eigen::AngleAxisd(40 * kDeg, Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(40 * kDeg, Eigen::Vector3d::UnitX()))
                                    .toRotationMatrix();
  const AttitudeCommand s = mapAccelerationToAttitude(Eigen::Vector3d(20, -20, 9.81), steep, {});
  EXPECT_NEAR(s.thrust, 14.0, 1e-12);
  EXPECT_LT(s.thrust * steep(2, 2), 9.81);
}

TEST(AttitudeMapping, BodyRatesDriveTowardDesired) {
  const Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  const Eigen::Quaterniond q_des = attitudeToQuaternion(0.2, -0.1, 0.0);
  const Eigen::Vector3d w = attitudeErrorToBodyRates(q, q_des, 12.0, 5.0, Eigen::Vector3d::Zero());
  EXPECT_GT(w.x(), 0.0);
  EXPECT_LT(w.y(), 0.0);
  const Eigen::Vector3d w0 =
      attitudeErrorToBodyRates(q_des, q_des, 12.0, 5.0, Eigen::Vector3d(0, 0, 0.7));
  EXPECT_LT((w0 - Eigen::Vector3d(0, 0, 0.7)).norm(), 1e-12);
}

// ---------------------------------------------------------------- ProNav

TEST(ProNav, HeadOnHasNoLateralCommand) {
  ProNav pn;
  const GuidanceOutput o =
      pn.compute(makeInput(Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(-1, 0, 0), Eigen::Vector3d(1, 0, 0)));
  EXPECT_NEAR(o.closing_speed, 2.0, 1e-12);
  EXPECT_LT(o.los_rate.norm(), 1e-12);
  // Only the speed loop and gravity remain: Kp (Vd - Vc) r_hat = 1.5 * (1.5 - 2) along +x
  EXPECT_NEAR(o.accel.x(), 1.5 * (1.5 - 2.0), 1e-12);
  EXPECT_NEAR(o.accel.y(), 0.0, 1e-12);
  EXPECT_NEAR(o.accel.z(), 9.81, 1e-12);
  EXPECT_EQ(o.command.yaw_rate, 0.0);
}

TEST(ProNav, LateralCommandFollowsTargetMotion) {
  // Target ahead moving to the left (+y): ProNav must accelerate the chaser to the left.
  ProNav pn;
  const GuidanceOutput o =
      pn.compute(makeInput(Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(1, 0, 0)));
  EXPECT_GT(o.los_rate.z(), 0.0);
  EXPECT_GT(o.accel.y(), 0.0);
  // a_PN = N |Vc| (Omega x r_hat), Omega = r x v_rel / |r|^2 -> (0, 0, 0.1)
  EXPECT_NEAR(o.accel.y(), 4.0 * 1.0 * 0.1, 1e-12);
  EXPECT_LT(o.command.roll, 0.0);  // roll left
}

TEST(ProNav, DegenerateRangeHovers) {
  ProNav pn;
  const GuidanceOutput o = pn.compute(makeInput(Eigen::Vector3d(0.01, 0, 0), Eigen::Vector3d::Zero(),
                                                Eigen::Vector3d(1, 0, 0)));
  EXPECT_NEAR(o.command.roll, 0.0, 1e-12);
  EXPECT_NEAR(o.command.pitch, 0.0, 1e-12);
}

// ---------------------------------------------------------------- Hybrid

TEST(HybridProNav, SwitchesAtFifteenDegrees) {
  HybridProNav h;
  const auto at = [&](double deg) {
    const Eigen::Vector3d r = rotZ(deg * kDeg) * Eigen::Vector3d(8, 0, 0);
    return h.compute(makeInput(r, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
  };
  EXPECT_EQ(at(14.9).mode, GuidanceMode::kProNav);
  const GuidanceOutput heading = at(15.0);
  EXPECT_EQ(heading.mode, GuidanceMode::kHeading);
  EXPECT_EQ(heading.command.roll, 0.0);
  EXPECT_EQ(heading.command.pitch, 0.0);
  EXPECT_NEAR(heading.command.thrust, 9.81, 1e-12);
  EXPECT_NEAR(heading.command.yaw_rate, 4.0 * 15.0 * kDeg, 1e-9);
  // No hysteresis by default: drops back as soon as |eps| < 15 deg
  EXPECT_EQ(at(-14.9).mode, GuidanceMode::kProNav);
}

TEST(HybridProNav, YawRateClampedAndSigned) {
  HybridProNav h;
  const GuidanceOutput right =
      h.compute(makeInput(Eigen::Vector3d(0, -8, 0), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
  EXPECT_NEAR(right.command.yaw_rate, -M_PI, 1e-12);  // 4 * (-90 deg) clamped
  const GuidanceOutput behind =
      h.compute(makeInput(Eigen::Vector3d(-8, 0.1, 0), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
  EXPECT_NEAR(behind.command.yaw_rate, M_PI, 1e-12);
}

TEST(HybridProNav, Hysteresis) {
  ProNavParams pn;
  HybridParams hp;
  hp.hysteresis = 5.0 * kDeg;
  HybridProNav h(pn, hp);
  const auto at = [&](double deg) {
    return h.compute(makeInput(rotZ(deg * kDeg) * Eigen::Vector3d(8, 0, 0), Eigen::Vector3d::Zero(),
                               Eigen::Vector3d::Zero())).mode;
  };
  EXPECT_EQ(at(20), GuidanceMode::kHeading);
  EXPECT_EQ(at(12), GuidanceMode::kHeading);
  EXPECT_EQ(at(9), GuidanceMode::kProNav);
  EXPECT_EQ(at(12), GuidanceMode::kProNav);
}

// ---------------------------------------------------------------- closed loop

namespace {

/// Point-mass quadrotor: attitude follows the command with a first-order lag, thrust acts along
/// the body z-axis, yaw integrates the commanded yaw rate. Guidance runs at 50 Hz on GA-frame
/// quantities derived from perfect state.
double simulateEngagement(GuidanceLaw& law, const Eigen::Vector3d& p_tgt0,
                          const Eigen::Vector3d& v_tgt, double yaw0, double* t_intercept) {
  const double dt = 0.02, tau_att = 0.1, g = 9.81, t_max = 20.0;
  Eigen::Vector3d p_c = Eigen::Vector3d::Zero(), v_c = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_t = p_tgt0;
  double yaw = yaw0, roll = 0.0, pitch = 0.0;
  double min_range = 1e9;

  for (double t = 0.0; t < t_max; t += dt) {
    const Eigen::Matrix3d R_WG = rotZ(yaw);
    const Eigen::Matrix3d R_GB = (Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
    GuidanceInput in;
    in.r = R_WG.transpose() * (p_t - p_c);
    in.v_tgt = R_WG.transpose() * v_tgt;
    in.v_chaser = R_WG.transpose() * v_c;
    in.R_GB = R_GB;

    const double range = in.r.norm();
    min_range = std::min(min_range, range);
    if (range < 1.0) {
      *t_intercept = t;
      return range;
    }

    const GuidanceOutput o = law.compute(in);
    roll += (o.command.roll - roll) * dt / tau_att;
    pitch += (o.command.pitch - pitch) * dt / tau_att;
    yaw += o.command.yaw_rate * dt;

    const Eigen::Vector3d a = R_WG * R_GB * Eigen::Vector3d(0, 0, o.command.thrust) -
                              Eigen::Vector3d(0, 0, g);
    v_c += a * dt;
    p_c += v_c * dt;
    p_t += v_tgt * dt;
  }
  *t_intercept = -1.0;
  return min_range;
}

}  // namespace

TEST(ClosedLoop, ProNavInterceptsCrossingTarget) {
  ProNav pn;
  double t_hit = -1.0;
  const double miss = simulateEngagement(pn, Eigen::Vector3d(8, 0, 0.5), Eigen::Vector3d(0, 0.5, 0),
                                         0.0, &t_hit);
  EXPECT_LT(miss, 1.0);
  EXPECT_GT(t_hit, 0.0);
}

TEST(ClosedLoop, HybridInterceptsFromLargeAzimuth) {
  // Target starts 90 deg off the nose: the hybrid law must turn first, then pursue.
  HybridProNav h;
  double t_hit = -1.0;
  const double miss = simulateEngagement(h, Eigen::Vector3d(0, 8, 0.5), Eigen::Vector3d(0.3, 0, 0),
                                         0.0, &t_hit);
  EXPECT_LT(miss, 1.0);
  EXPECT_GT(t_hit, 0.0);
}

TEST(ClosedLoop, HybridInterceptsTargetBehind) {
  HybridProNav h;
  double t_hit = -1.0;
  const double miss = simulateEngagement(h, Eigen::Vector3d(-8, 2, 0.0), Eigen::Vector3d(0, -0.3, 0),
                                         0.0, &t_hit);
  EXPECT_LT(miss, 1.0);
  EXPECT_GT(t_hit, 0.0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
