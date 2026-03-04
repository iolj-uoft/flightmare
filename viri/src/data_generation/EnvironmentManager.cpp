#include "flightros/viri/EnvironmentManager.hpp"

using namespace flightlib;

EnvironmentManager::EnvironmentManager() {
    // init Unity Bridge
    unity_bridge_ptr_ = UnityBridge::getInstance();
    Vector<3> quad_size(0.5, 0.5, 0.5);

    // create target quadrotor
    target_quad_ptr_ = std::make_shared<Quadrotor>();
    target_quad_ptr_->setSize(quad_size);
    QuadState target_quad_state;
    target_quad_state.setZero();
    target_quad_ptr_->reset(target_quad_state);

    // create chaser quadrotor
    chaser_quad_ptr_ = std::make_shared<Quadrotor>();
    chaser_quad_ptr_->setSize(quad_size);
    QuadState chaser_quad_state;
    chaser_quad_state.setZero();
    chaser_quad_ptr_->reset(chaser_quad_state);

    // create and attatch camera to chaser
    rgb_camera_ = std::make_shared<RGBCamera>();
    Vector<3> B_r_BC(0.0, 0.0, 0.3);
    Matrix<3, 3> R_BC = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();

    rgb_camera_->setFOV(60);
    rgb_camera_->setWidth(640);
    rgb_camera_->setHeight(360);
    rgb_camera_->setRelPose(B_r_BC, R_BC);
    rgb_camera_->setPostProcesscing(std::vector<bool>{true, false, false});
    chaser_quad_ptr_->addRGBCamera(rgb_camera_);

    initializeRingDrones();
};

void EnvironmentManager::initializeRingDrones() {
    const int ring_count = 8;
    ring_quads_.reserve(ring_count);
    Vector<3> quad_size(0.5, 0.5, 0.5);

    for(int i = 0; i < ring_count; ++i) {
        auto q = std::make_shared<Quadrotor>();
        q->setSize(quad_size);

        QuadState qs;
        qs.setZero();
        q->reset(qs);

        ring_quads_.push_back(q);
    }
}

bool EnvironmentManager::connectToUnity() {
    try {
        unity_bridge_ptr_->addQuadrotor(target_quad_ptr_);
        ROS_INFO("Added target_quad to UnityBridge");

        unity_bridge_ptr_->addQuadrotor(chaser_quad_ptr_);
        ROS_INFO("Added chaser_quad to UnityBridge");

        for (size_t i = 0; i < ring_quads_.size(); ++i) {
            unity_bridge_ptr_->addQuadrotor(ring_quads_[i]);
            ROS_INFO("Added ring_quad_%d to UnityBridge", (int)i);
        }

        SceneID scene_id{UnityScene::GARAGE};
        bool unity_ready = unity_bridge_ptr_->connectUnity(scene_id);
        ROS_INFO("UnityBridge connectivity: %d", (int)unity_ready);
        return unity_ready;
    }
    catch (const std::exception& e) {
        ROS_ERROR("UnityBridge failed to connect: %s", e.what());
        return false;
    }
}

void EnvironmentManager::renderFrame(FrameID frame_id) {
    unity_bridge_ptr_->getRender(frame_id);
    unity_bridge_ptr_->handleOutput();
}

std::shared_ptr<Quadrotor> EnvironmentManager::getTargetQuad() const { return target_quad_ptr_; }
std::shared_ptr<Quadrotor> EnvironmentManager::getChaserQuad() const { return chaser_quad_ptr_; }
std::vector<std::shared_ptr<Quadrotor>> EnvironmentManager::getRingQuads() const { return ring_quads_; }
std::shared_ptr<RGBCamera> EnvironmentManager::getRGBCamera() const { return rgb_camera_; }