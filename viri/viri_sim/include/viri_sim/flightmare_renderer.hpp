#pragma once

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <memory>
#include <string>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

namespace viri_sim {

/// One stereo camera, extrinsics in the Gazebo (REP-103) body frame.
struct CameraConfig {
  Eigen::Vector3d position_body = Eigen::Vector3d::Zero();
};

/**
 * Renders the chaser's stereo cameras in Flightmare/Unity from Gazebo ground-truth poses
 * (paper Sec. III-B.1).
 *
 * Every pose sent to Unity is right-multiplied by a yaw offset (default -135 deg = -90 deg
 * ROS/Unity forward axis + -45 deg for the RotorS "+" vs Flightmare "x" arm layout). Camera
 * extrinsics are configured in the Gazebo body frame and converted to the Flightmare body frame
 * with the same offset, so the TF tree (camera_tf_publisher.py) and the renderer share one
 * source of truth: config/cameras.yaml.
 */
class FlightmareRenderer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FlightmareRenderer(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

 private:
  void loadParams();
  void setupCameras();
  void chaserOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void targetOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void renderLoop(const ros::TimerEvent& event);
  bool connectUnity();

  /// Gazebo pose -> Flightmare quad state (render offset + Unity z offset).
  void toQuadState(const nav_msgs::Odometry& msg, flightlib::QuadState& state) const;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::unique_ptr<image_transport::ImageTransport> it_;
  image_transport::Publisher left_pub_;
  image_transport::Publisher right_pub_;
  ros::Subscriber chaser_sub_;
  ros::Subscriber target_sub_;
  ros::Timer render_timer_;

  std::shared_ptr<flightlib::Quadrotor> chaser_quad_;
  std::shared_ptr<flightlib::Quadrotor> target_quad_;
  flightlib::QuadState chaser_state_;
  flightlib::QuadState target_state_;
  std::shared_ptr<flightlib::RGBCamera> left_camera_;
  std::shared_ptr<flightlib::RGBCamera> right_camera_;

  std::shared_ptr<flightlib::UnityBridge> unity_bridge_;
  bool unity_ready_ = false;

  // Parameters
  bool unity_render_ = true;
  int scene_id_ = 1;  // UnityScene::WAREHOUSE
  double render_rate_ = 50.0;
  double unity_startup_delay_ = 5.0;
  double render_z_offset_ = 50.0;
  double render_yaw_offset_ = -0.75 * M_PI;  // -135 deg
  double quad_size_ = 0.5;
  std::string chaser_odom_topic_;
  std::string target_odom_topic_;
  std::string left_topic_, right_topic_;
  std::string left_frame_, right_frame_;

  int image_width_ = 640;
  int image_height_ = 360;
  double fov_deg_ = 60.0;
  double tilt_ = 15.0 * M_PI / 180.0;
  CameraConfig left_cfg_, right_cfg_;
};

}  // namespace viri_sim
