#include "flightros/viri/TrajectoryGenerator.hpp"

using namespace flightlib;

TrajectoryGenerator::TrajectoryGenerator() 
    : target_speed_(0.0), follow_distance_(4.0), chaser_osc_amp_(3.5), 
      chaser_osc_omega_(0.8), start_x_(50.0), start_y_(50.0), start_z_(50.0), 
      ring_radius_(1.0), target_yaw_(0.0), target_roll_(0.0), target_pitch_(0.0),
      ring_count_(8) {

    target_x_ = start_x_;
    target_y_ = start_y_;
    target_z_ = start_z_;

    // Initialize RNG
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    rng_ = std::default_random_engine(seed);
    yaw_rate_dist_ = std::uniform_real_distribution<double>(-1.0, 1.0);
    roll_pitch_dist_ = std::uniform_real_distribution<double>(-0.5, 0.5);
    ring_rate_dist_ = std::uniform_real_distribution<double>(-0.5, 0.5);

    // Initialize ring states
    ring_yaws_.assign(ring_count_, 0.0);
    ring_yaw_rates_.assign(ring_count_, 0.0);
    ring_rolls_.assign(ring_count_, 0.0);
    ring_roll_rates_.assign(ring_count_, 0.0);
    ring_pitchs_.assign(ring_count_, 0.0);
    ring_pitch_rates_.assign(ring_count_, 0.0);

    for (int i = 0; i < ring_count_; ++i) {
        ring_yaw_rates_[i] = ring_rate_dist_(rng_);
        ring_roll_rates_[i] = ring_rate_dist_(rng_);
        ring_pitch_rates_[i] = ring_rate_dist_(rng_);
    }
}

double TrajectoryGenerator::getTargetX() const {
    return target_x_;
}

void TrajectoryGenerator::updateTargetState(double current_time, double dt, std::shared_ptr<Quadrotor> target_quad) {
    QuadState state;
    state.setZero();

    // Position
    target_x_ = start_x_ + target_speed_ * current_time;
    state.x[QS::POSX] = target_x_;
    state.x[QS::POSY] = target_y_;
    state.x[QS::POSZ] = target_z_;

    // Orientation
    double target_yaw_rate = yaw_rate_dist_(rng_) * 0.5;
    double target_roll_rate = roll_pitch_dist_(rng_) * 0.2;
    double target_pitch_rate = roll_pitch_dist_(rng_) * 0.2;
    
    target_yaw_ += target_yaw_rate * dt;
    target_roll_ += target_roll_rate * dt;
    target_pitch_ += target_pitch_rate * dt;

    double cy = std::cos(target_yaw_ * 0.5);
    double sy = std::sin(target_yaw_ * 0.5);
    double cp = std::cos(target_pitch_ * 0.5);
    double sp = std::sin(target_pitch_ * 0.5);
    double cr = std::cos(target_roll_ * 0.5);
    double sr = std::sin(target_roll_ * 0.5);

    state.x[QS::ATTW] = cr * cp * cy + sr * sp * sy;
    state.x[QS::ATTX] = sr * cp * cy - cr * sp * sy;
    state.x[QS::ATTY] = cr * sp * cy + sr * cp * sy;
    state.x[QS::ATTZ] = cr * cp * sy - sr * sp * cy;

    target_quad->setState(state);
}

void TrajectoryGenerator::updateChaserState(double current_time, std::shared_ptr<Quadrotor> chaser_quad) {
    QuadState state;
    state.setZero();

    // Position (follows target with oscillation)
    double chaser_x = target_x_ - follow_distance_ + chaser_osc_amp_ * std::sin(chaser_osc_omega_ * current_time);
    state.x[QS::POSX] = chaser_x;
    state.x[QS::POSY] = target_y_;
    state.x[QS::POSZ] = target_z_;

    // Orientation (facing target)
    double chaser_yaw = -M_PI / 2;
    state.x[QS::ATTW] = std::cos(chaser_yaw / 2.0);
    state.x[QS::ATTZ] = std::sin(chaser_yaw / 2.0);

    chaser_quad->setState(state);
}

void TrajectoryGenerator::updateRingStates(double dt, const std::vector<std::shared_ptr<Quadrotor>>& ring_quads) {
    const size_t n = ring_quads.size();
    // make sure internal ring state vectors match the number of quads
    if (ring_yaws_.size() != n) {
        ring_yaws_.assign(n, 0.0);
        ring_yaw_rates_.assign(n, 0.0);
        ring_rolls_.assign(n, 0.0);
        ring_roll_rates_.assign(n, 0.0);
        ring_pitchs_.assign(n, 0.0);
        ring_pitch_rates_.assign(n, 0.0);
        for (size_t i = 0; i < n; ++i) {
            ring_yaw_rates_[i] = ring_rate_dist_(rng_);
            ring_roll_rates_[i] = ring_rate_dist_(rng_);
            ring_pitch_rates_[i] = ring_rate_dist_(rng_);
        }
    }

    for (size_t i = 0; i < n; ++i) {
        QuadState state;
        state.setZero();
        // update angular states (these now control in-place rotation only)
        ring_yaws_[i] += ring_yaw_rates_[i] * dt;
        ring_rolls_[i] += ring_roll_rates_[i] * dt;
        ring_pitchs_[i] += ring_pitch_rates_[i] * dt;

        // place drones at fixed points on the Y-Z plane (no orbital motion)
        double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(n);
        // fixed theta for position (DO NOT add ring_yaws_ here)
        double theta = angle;
        state.x[QS::POSX] = target_x_; // center on target X so ring lies in Y-Z plane
        state.x[QS::POSY] = target_y_ + ring_radius_ * std::cos(theta);
        state.x[QS::POSZ] = target_z_ + ring_radius_ * std::sin(theta);

        // orientation uses the time-integrated ring_yaws_/rolls/pitchs so each drone spins in-place
        double cy = std::cos(ring_yaws_[i] * 0.5);
        double sy = std::sin(ring_yaws_[i] * 0.5);
        double cp = std::cos(ring_pitchs_[i] * 0.5);
        double sp = std::sin(ring_pitchs_[i] * 0.5);
        double cr = std::cos(ring_rolls_[i] * 0.5);
        double sr = std::sin(ring_rolls_[i] * 0.5);

        state.x[QS::ATTW] = cr * cp * cy + sr * sp * sy;
        state.x[QS::ATTX] = sr * cp * cy - cr * sp * sy;
        state.x[QS::ATTY] = cr * sp * cy + sr * cp * sy;
        state.x[QS::ATTZ] = cr * cp * sy - sr * sp * cy;

        ring_quads[i]->setState(state);
    }
}