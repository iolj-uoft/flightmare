#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "viri_estimation/frame_utils.hpp"
#include "viri_estimation/relative_ltvkf.hpp"

using viri_estimation::LtvkfParams;
using viri_estimation::RelativeLTVKF;
using viri_estimation::Vector6d;

namespace {

constexpr double kDt = 0.01;  // 100 Hz odometry

const Eigen::Matrix3d kI = Eigen::Matrix3d::Identity();
const Eigen::Vector3d kZero = Eigen::Vector3d::Zero();

}  // namespace

TEST(FrameUtils, DerotationRemovesRollPitchKeepsYaw) {
  const double roll = 0.3, pitch = -0.2, yaw = 1.1;
  const Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  const Eigen::Matrix3d R_GA = viri_estimation::derotationMatrix(q);
  const Eigen::Vector3d rpy = viri_estimation::rotationToRpy(R_GA);
  EXPECT_NEAR(rpy(0), roll, 1e-12);
  EXPECT_NEAR(rpy(1), pitch, 1e-12);
  EXPECT_NEAR(rpy(2), 0.0, 1e-12);
  // GA z-axis is the world z-axis: body vectors mapped through R_GA keep their world height
  const Eigen::Vector3d v_body(1.0, -2.0, 0.5);
  EXPECT_NEAR((R_GA * v_body).z(), (q * v_body).z(), 1e-12);
}

TEST(FrameUtils, YawRateLevelFlightEqualsBodyRate) {
  EXPECT_NEAR(viri_estimation::eulerYawRate(0.0, 0.0, Eigen::Vector3d(0.4, -0.2, 0.7)), 0.7, 1e-12);
  // Gimbal-lock fallback
  EXPECT_NEAR(viri_estimation::eulerYawRate(0.1, M_PI / 2.0, Eigen::Vector3d(0, 0.3, 0.7)), 0.7, 1e-12);
}

TEST(FrameUtils, WrapAngle) {
  EXPECT_NEAR(viri_estimation::wrapAngle(3.0 * M_PI / 2.0), -M_PI / 2.0, 1e-12);
  EXPECT_NEAR(viri_estimation::wrapAngle(-M_PI), M_PI, 1e-12);
  EXPECT_NEAR(viri_estimation::wrapAngle(0.25), 0.25, 1e-12);
}

TEST(RelativeLTVKF, RequiresAttitudeBeforeInit) {
  RelativeLTVKF f;
  EXPECT_EQ(f.update(0.0, Eigen::Vector3d(5, 0, 0)), RelativeLTVKF::UpdateResult::kNoAttitude);
  f.predict(0.0, kZero, kZero, kI);
  EXPECT_EQ(f.update(0.0, Eigen::Vector3d(5, 0, 0)), RelativeLTVKF::UpdateResult::kInitialized);
  EXPECT_TRUE(f.initialized());
  EXPECT_NEAR(f.state()(0), 5.0, 1e-12);
}

TEST(RelativeLTVKF, PaperNoiseModel) {
  RelativeLTVKF f;
  const Eigen::Matrix3d R = f.bodyMeasurementCovariance(Eigen::Vector3d(4.0, 0.0, 0.0));
  EXPECT_NEAR(R(0, 0), std::pow(0.001 * 16.0, 2) + 0.01, 1e-12);
  EXPECT_NEAR(R(1, 1), std::pow(0.0005 * 4.0, 2) + 0.01, 1e-12);
  EXPECT_NEAR(R(2, 2), R(1, 1), 1e-12);
}

TEST(RelativeLTVKF, ConstantVelocityTargetConverges) {
  // Chaser hovering, target moving at constant velocity; noisy position measurements.
  RelativeLTVKF f;
  std::mt19937 gen(42);
  std::normal_distribution<double> noise(0.0, 0.05);

  const Eigen::Vector3d p0(6.0, 1.0, 0.5), v_tgt(0.5, -0.3, 0.1);
  double t = 0.0;
  f.predict(t, kZero, kZero, kI);
  f.update(t, p0);
  // The paper's large velocity process noise (q_vel = 10) makes single velocity samples
  // noisy by design, so check the mean over the last 2 s.
  Eigen::Vector3d v_mean = Eigen::Vector3d::Zero();
  int n_mean = 0;
  for (int k = 1; k <= 1000; ++k) {
    t = k * kDt;
    f.predict(t, kZero, kZero, kI);
    if (k % 5 == 0) {  // 20 Hz camera
      const Eigen::Vector3d p = p0 + v_tgt * t;
      f.update(t, p + Eigen::Vector3d(noise(gen), noise(gen), noise(gen)));
    }
    if (k > 800) {
      v_mean += f.state().tail<3>();
      ++n_mean;
    }
  }
  v_mean /= n_mean;
  const Eigen::Vector3d p_true = p0 + v_tgt * t;
  EXPECT_LT((f.state().head<3>() - p_true).norm(), 0.15);
  EXPECT_LT((v_mean - v_tgt).norm(), 0.1);
}

TEST(RelativeLTVKF, YawingChaserTracksStationaryTarget) {
  // Chaser hovers at the origin yawing at 0.5 rad/s; the target is fixed in the world.
  // Prediction alone (no measurements after init) must rotate p_rel correctly in GA.
  RelativeLTVKF f;
  const double yaw_rate = 0.5;
  const Eigen::Vector3d p_world(5.0, 0.0, 1.0);
  f.predict(0.0, kZero, kZero, kI);
  f.update(0.0, p_world);
  double t = 0.0;
  for (int k = 1; k <= 200; ++k) {
    t = k * kDt;
    f.predict(t, kZero, Eigen::Vector3d(0, 0, yaw_rate), kI);
  }
  const Eigen::Vector3d expected = viri_estimation::rotZ(yaw_rate * t).transpose() * p_world;
  EXPECT_LT((f.state().head<3>() - expected).norm(), 1e-6);
}

TEST(RelativeLTVKF, DelayedUpdateMatchesInOrderFilter) {
  // Rewind-update-replay must be equivalent to having received the measurement on time.
  std::mt19937 gen(7);
  std::normal_distribution<double> n(0.0, 0.3);

  struct Step { Eigen::Vector3d v, w; };
  std::vector<Step> steps;
  for (int k = 0; k < 60; ++k) steps.push_back({Eigen::Vector3d(n(gen), n(gen), n(gen)),
                                                Eigen::Vector3d(0, 0, n(gen))});

  const Eigen::Vector3d z_init(6.0, 0.5, 0.2), z_meas(5.6, 0.7, 0.1);
  const int k_meas = 30;                      // measurement taken just after step 30
  const double t_meas = k_meas * kDt + 1e-4;

  RelativeLTVKF in_order, delayed;
  for (auto* f : {&in_order, &delayed}) {
    f->predict(0.0, kZero, kZero, kI);
    f->update(0.0, z_init);
  }
  for (int k = 1; k < 60; ++k) {
    const double t = k * kDt;
    in_order.predict(t, steps[k].v, steps[k].w, kI);
    delayed.predict(t, steps[k].v, steps[k].w, kI);
    if (k == k_meas) {
      EXPECT_EQ(in_order.update(t_meas, z_meas), RelativeLTVKF::UpdateResult::kApplied);
    }
  }
  EXPECT_EQ(delayed.update(t_meas, z_meas), RelativeLTVKF::UpdateResult::kApplied);

  EXPECT_LT((in_order.state() - delayed.state()).norm(), 1e-9);
  EXPECT_LT((in_order.covariance() - delayed.covariance()).norm(), 1e-9);
}

TEST(RelativeLTVKF, UsesHistoricalDerotation) {
  // The chaser pitched (rotated about y) at the image time, then levelled out before the
  // measurement arrived. The update must use R_GA(t_img), not the latest attitude.
  LtvkfParams p;
  p.q_pos = 0.0;  // isolate the rotation effect
  p.q_vel = 0.0;
  RelativeLTVKF f(p);

  const Eigen::Matrix3d R_pitched =
      Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Vector3d p_true_ga(5.0, 0.0, 0.0);

  f.predict(0.0, kZero, kZero, kI);
  f.update(0.0, p_true_ga);
  for (int k = 1; k <= 20; ++k) f.predict(k * kDt, kZero, kZero, k <= 10 ? R_pitched : kI);

  // Measurement at t = 0.05 (pitched): body-frame vector = R_pitched^T * p_ga
  const Eigen::Vector3d z_body = R_pitched.transpose() * p_true_ga;
  ASSERT_EQ(f.update(0.05, z_body), RelativeLTVKF::UpdateResult::kApplied);
  // Consistent measurement -> zero innovation -> state unchanged
  EXPECT_LT((f.state().head<3>() - p_true_ga).norm(), 1e-9);
}

TEST(RelativeLTVKF, RejectsMeasurementOlderThanBuffer) {
  LtvkfParams p;
  p.history_size = 10;
  RelativeLTVKF f(p);
  f.predict(0.0, kZero, kZero, kI);
  f.update(0.0, Eigen::Vector3d(5, 0, 0));
  for (int k = 1; k <= 30; ++k) f.predict(k * kDt, kZero, kZero, kI);
  EXPECT_EQ(f.historySize(), 10u);
  EXPECT_EQ(f.update(0.05, Eigen::Vector3d(5, 0, 0)), RelativeLTVKF::UpdateResult::kTooOld);
  // Newer than the newest entry is applied to the newest entry
  EXPECT_EQ(f.update(1.0, Eigen::Vector3d(5, 0, 0)), RelativeLTVKF::UpdateResult::kApplied);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
