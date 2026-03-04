#ifndef ENVIRONMENT_MANAGER_HPP
#define ENVIRONMENT_MANAGER_HPP

#include <memory>
#include <vector>
#include <ros/ros.h>

#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

class EnvironmentManager {
public:
    EnvironmentManager();
    ~EnvironmentManager() = default;

    bool connectToUnity();
    void renderFrame(flightlib::FrameID frame_id);

    std::shared_ptr<flightlib::Quadrotor> getTargetQuad() const;
    std::shared_ptr<flightlib::Quadrotor> getChaserQuad() const;
    std::vector<std::shared_ptr<flightlib::Quadrotor>> getRingQuads() const;
    std::shared_ptr<flightlib::RGBCamera> getRGBCamera() const;

private:
    std::shared_ptr<flightlib::UnityBridge> unity_bridge_ptr_;
    std::shared_ptr<flightlib::Quadrotor> target_quad_ptr_;
    std::shared_ptr<flightlib::Quadrotor> chaser_quad_ptr_; 
    std::vector<std::shared_ptr<flightlib::Quadrotor>> ring_quads_;

    std::shared_ptr<flightlib::RGBCamera> rgb_camera_;

    void initializeRingDrones();
};

#endif