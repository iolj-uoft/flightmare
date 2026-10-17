#include "viri_estimation/relative_ltvkf.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

#include "viri_estimation/frame_utils.hpp"

namespace viri_estimation {

RelativeLTVKF::RelativeLTVKF(const LtvkfParams& params) : params_(params) {
  Q_.setZero();
  Q_.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * params_.q_pos;
  Q_.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * params_.q_vel;
  reset();
}

void RelativeLTVKF::reset() {
  x_.setZero();
  P_.setZero();
  P_.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * params_.p0_pos;
  P_.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * params_.p0_vel;
  initialized_ = false;
  have_time_ = false;
  have_attitude_ = false;
  last_meas_time_ = 0.0;
  latest_R_GA_.setIdentity();
  history_.clear();
}

Vector6d RelativeLTVKF::dynamics(const Vector6d& x, const Eigen::Vector3d& v_ga,
                                 const Eigen::Vector3d& w_ga) {
  Vector6d x_dot;
  const Eigen::Vector3d p_rel = x.head<3>();
  const Eigen::Vector3d v_tgt = x.tail<3>();
  x_dot.head<3>() = v_tgt - v_ga - w_ga.cross(p_rel);  // eq. 9
  x_dot.tail<3>() = -w_ga.cross(v_tgt);                // eq. 10
  return x_dot;
}

void RelativeLTVKF::propagate(Vector6d& x, Matrix6d& P, const Eigen::Vector3d& v_ga,
                              const Eigen::Vector3d& w_ga, double dt) const {
  // Mean: RK4 with inputs held constant over the step (eq. 13)
  const Vector6d k1 = dynamics(x, v_ga, w_ga);
  const Vector6d k2 = dynamics(x + 0.5 * dt * k1, v_ga, w_ga);
  const Vector6d k3 = dynamics(x + 0.5 * dt * k2, v_ga, w_ga);
  const Vector6d k4 = dynamics(x + dt * k3, v_ga, w_ga);
  x += (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

  // Covariance: second-order transition matrix (eq. 12, 14, 15)
  Matrix6d A = Matrix6d::Zero();
  const Eigen::Matrix3d w_skew = skew(w_ga);
  A.topLeftCorner<3, 3>() = -w_skew;
  A.topRightCorner<3, 3>() = Eigen::Matrix3d::Identity();
  A.bottomRightCorner<3, 3>() = -w_skew;

  const Matrix6d Phi = Matrix6d::Identity() + A * dt + 0.5 * A * A * dt * dt;
  P = Phi * P * Phi.transpose() + Q_ * dt;
}

bool RelativeLTVKF::predict(double t, const Eigen::Vector3d& v_ga,
                            const Eigen::Vector3d& w_ga, const Eigen::Matrix3d& R_GA) {
  latest_R_GA_ = R_GA;
  have_attitude_ = true;

  if (!initialized_ || !have_time_) {
    last_time_ = t;
    have_time_ = true;
    return false;
  }

  const double dt = t - last_time_;
  if (dt <= 0.0 || dt > params_.max_dt) {
    last_time_ = t;
    return false;
  }

  propagate(x_, P_, v_ga, w_ga, dt);

  history_.push_back({t, x_, P_, v_ga, w_ga, R_GA, dt});
  while (history_.size() > params_.history_size) history_.pop_front();

  last_time_ = t;
  return true;
}

Eigen::Matrix3d RelativeLTVKF::bodyMeasurementCovariance(const Eigen::Vector3d& z_body) const {
  const double d = std::max(z_body.norm(), params_.min_range);
  const double sigma_depth = params_.depth_noise_coeff * d * d;  // eq. 17
  const double sigma_lat = params_.lateral_noise_coeff * d;       // eq. 18

  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  R(0, 0) = sigma_depth * sigma_depth;
  R(1, 1) = sigma_lat * sigma_lat;
  R(2, 2) = sigma_lat * sigma_lat;
  return R + Eigen::Matrix3d::Identity() * params_.noise_floor;  // eq. 19
}

void RelativeLTVKF::kalmanUpdate(Vector6d& x, Matrix6d& P, const Eigen::Vector3d& z_ga,
                                 const Eigen::Matrix3d& R_ga) const {
  Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
  H.leftCols<3>().setIdentity();

  const Eigen::Vector3d y = z_ga - H * x;
  const Eigen::Matrix3d S = H * P * H.transpose() + R_ga;                       // eq. 21
  const Eigen::Matrix<double, 6, 3> K = P * H.transpose() * S.inverse();         // eq. 22
  x += K * y;                                                                   // eq. 23
  P = (Matrix6d::Identity() - K * H) * P;                                       // eq. 24
}

RelativeLTVKF::UpdateResult RelativeLTVKF::update(double t_meas,
                                                  const Eigen::Vector3d& z_body) {
  const Eigen::Matrix3d R_body = bodyMeasurementCovariance(z_body);

  if (!initialized_) {
    if (!have_attitude_) return UpdateResult::kNoAttitude;
    // Seed p_rel from the first measurement; v_tgt = 0 with large P0 (Sec. III-F.1)
    x_.setZero();
    x_.head<3>() = latest_R_GA_ * z_body;
    initialized_ = true;
    last_meas_time_ = t_meas;
    history_.clear();
    return UpdateResult::kInitialized;
  }

  // No buffered steps yet: fuse into the current state with the latest attitude.
  if (history_.empty()) {
    kalmanUpdate(x_, P_, latest_R_GA_ * z_body,
                 latest_R_GA_ * R_body * latest_R_GA_.transpose());
    last_meas_time_ = t_meas;
    return UpdateResult::kApplied;
  }

  if (t_meas < history_.front().stamp) return UpdateResult::kTooOld;

  // 1. Rewind: last entry at or before t_meas. A measurement newer than the buffer is
  //    applied to the newest entry (nothing to replay).
  auto after = std::upper_bound(
      history_.begin(), history_.end(), t_meas,
      [](double t, const HistoryElement& e) { return t < e.stamp; });
  auto it = std::prev(after);

  // Historical derotation R_GA(t_img): attitude of the buffer entry nearest to t_meas.
  Eigen::Matrix3d R_GA = it->R_GA;
  if (after != history_.end() && (after->stamp - t_meas) < (t_meas - it->stamp)) {
    R_GA = after->R_GA;
  }

  // 2. Update the historical state (eq. 16, 20-24)
  kalmanUpdate(it->x, it->P, R_GA * z_body, R_GA * R_body * R_GA.transpose());

  // 3. Replay with the stored inputs, overwriting the buffer in place
  for (auto prev = it, next = after; next != history_.end(); ++prev, ++next) {
    next->x = prev->x;
    next->P = prev->P;
    propagate(next->x, next->P, next->v_ga, next->w_ga, next->dt);
  }

  // 4. Activate
  x_ = history_.back().x;
  P_ = history_.back().P;
  last_meas_time_ = std::max(last_meas_time_, t_meas);
  return UpdateResult::kApplied;
}

}  // namespace viri_estimation
