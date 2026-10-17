#pragma once

#include <Eigen/Dense>
#include <cstddef>
#include <deque>

namespace viri_estimation {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

/// Tuning of the relative LTVKF. Defaults are the values reported in the paper (Sec. III-F).
struct LtvkfParams {
  // Initial covariance P0 = diag(p0_pos * I3, p0_vel * I3)
  double p0_pos = 5.0;
  double p0_vel = 100.0;
  // Continuous process noise Q = diag(q_pos * I3, q_vel * I3)
  double q_pos = 0.05;
  double q_vel = 10.0;
  // Range-adaptive stereo noise (eq. 17-19), modelled in the body frame (x = depth axis)
  double depth_noise_coeff = 0.001;   // sigma_depth = c_d * d^2
  double lateral_noise_coeff = 0.0005;  // sigma_lat = c_l * d
  double noise_floor = 0.01;          // added to every diagonal entry
  double min_range = 0.5;             // range clamp used in the noise model [m]
  // Rewind-update-replay buffer
  std::size_t history_size = 100;     // ~1 s at 100 Hz
  double max_dt = 0.1;                // larger odometry gaps are skipped [s]
};

/**
 * Linear time-varying Kalman filter for the target state relative to the chaser,
 * expressed in the chaser's gravity-aligned (GA) frame (paper Sec. III-F).
 *
 *   x = [p_rel; v_tgt]
 *   p_rel_dot = v_tgt - v_chaser - w_GA x p_rel
 *   v_tgt_dot = -w_GA x v_tgt
 *
 * Delayed stereo measurements are fused with rewind-update-replay over a ring buffer of
 * prediction steps. The class is ROS-free; time is passed in seconds.
 */
class RelativeLTVKF {
 public:
  enum class UpdateResult { kApplied, kInitialized, kNoAttitude, kTooOld };

  explicit RelativeLTVKF(const LtvkfParams& params = LtvkfParams());

  /**
   * Prediction step driven by chaser odometry.
   * Before initialization this only records the latest time and attitude.
   *
   * @param t     odometry stamp [s]
   * @param v_ga  chaser linear velocity in GA
   * @param w_ga  GA angular velocity [0, 0, psi_dot]
   * @param R_GA  body -> GA derotation at time t
   * @return true when the state was propagated
   */
  bool predict(double t, const Eigen::Vector3d& v_ga, const Eigen::Vector3d& w_ga,
               const Eigen::Matrix3d& R_GA);

  /**
   * Fuse a (possibly delayed) stereo measurement of the target in the chaser body frame.
   * The derotation stored in the buffer at t_meas is used for both the measurement and its
   * covariance (eq. 16, 20). The first measurement initializes the filter.
   */
  UpdateResult update(double t_meas, const Eigen::Vector3d& z_body);

  void reset();

  bool initialized() const { return initialized_; }
  const Vector6d& state() const { return x_; }
  const Matrix6d& covariance() const { return P_; }
  double lastMeasurementTime() const { return last_meas_time_; }
  std::size_t historySize() const { return history_.size(); }
  const LtvkfParams& params() const { return params_; }

  /// Body-frame measurement covariance R_body(z_body) (eq. 17-19).
  Eigen::Matrix3d bodyMeasurementCovariance(const Eigen::Vector3d& z_body) const;

  /// Continuous dynamics f(x) = A x + b for fixed inputs.
  static Vector6d dynamics(const Vector6d& x, const Eigen::Vector3d& v_ga,
                           const Eigen::Vector3d& w_ga);

 private:
  struct HistoryElement {
    double stamp;
    Vector6d x;
    Matrix6d P;
    Eigen::Vector3d v_ga;
    Eigen::Vector3d w_ga;
    Eigen::Matrix3d R_GA;
    double dt;
  };

  void propagate(Vector6d& x, Matrix6d& P, const Eigen::Vector3d& v_ga,
                 const Eigen::Vector3d& w_ga, double dt) const;
  void kalmanUpdate(Vector6d& x, Matrix6d& P, const Eigen::Vector3d& z_ga,
                    const Eigen::Matrix3d& R_ga) const;

  LtvkfParams params_;
  Matrix6d Q_;

  Vector6d x_;
  Matrix6d P_;
  bool initialized_ = false;

  bool have_time_ = false;
  double last_time_ = 0.0;
  double last_meas_time_ = 0.0;
  bool have_attitude_ = false;
  Eigen::Matrix3d latest_R_GA_ = Eigen::Matrix3d::Identity();

  std::deque<HistoryElement> history_;
};

}  // namespace viri_estimation
