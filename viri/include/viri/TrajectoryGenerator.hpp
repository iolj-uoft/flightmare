#ifndef TRAJECTORY_GENERATOR_HPP
#define TRAJECTORY_GENERATOR_HPP

#include <memory>
#include <vector>
#include <random>
#include <chrono>

#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/common/quad_state.hpp"

class TrajectoryGenerator {
public:
    TrajectoryGenerator();
    ~TrajectoryGenerator() = default;

    // Update functions for each entity
    void updateTargetState(double current_time, double dt, std::shared_ptr<flightlib::Quadrotor> target_quad);
    void updateChaserState(double current_time, std::shared_ptr<flightlib::Quadrotor> chaser_quad);
    void updateRingStates(double dt, const std::vector<std::shared_ptr<flightlib::Quadrotor>>& ring_quads);

    // Helper to get the target's current X position for the chaser to follow
    double getTargetX() const;

private:
    // Trajectory constants
    double target_speed_;
    double follow_distance_;
    double chaser_osc_amp_;
    double chaser_osc_omega_;
    double start_x_, start_y_, start_z_;
    double ring_radius_;

    // Target dynamic state
    double target_x_, target_y_, target_z_;
    double target_yaw_, target_roll_, target_pitch_;

    // Ring drones dynamic state
    int ring_count_;
    std::vector<double> ring_yaws_, ring_yaw_rates_;
    std::vector<double> ring_rolls_, ring_roll_rates_;
    std::vector<double> ring_pitchs_, ring_pitch_rates_;

    // Random Number Generators
    std::default_random_engine rng_;
    std::uniform_real_distribution<double> yaw_rate_dist_;
    std::uniform_real_distribution<double> roll_pitch_dist_;
    std::uniform_real_distribution<double> ring_rate_dist_;
};

#endif // TRAJECTORY_GENERATOR_HPP