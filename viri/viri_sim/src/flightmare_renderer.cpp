#include "viri_sim/flightmare_renderer.hpp"

#include <sensor_msgs/Image.h>

#include <vector>

using namespace flightlib;

namespace viri_sim {

namespace {

constexpr double kDegToRad = M_PI / 180.0;

Eigen::Vector3d loadVector(const ros::NodeHandle& pnh, const std::string& name,
                           const Eigen::Vector3d& fallback) {
  std::vector<double> v;
  if (!pnh.getParam(name, v) || v.size() != 3) return fallback;
  return Eigen::Vector3d(v[0], v[1], v[2]);
}

}  // namespace

FlightmareRenderer::FlightmareRenderer(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh) {
  loadParams();

  // flightlib setters take non-const Ref<> parameters, so pass mutable lvalues.
  Vector<3> quad_size = Vector<3>::Constant(static_cast<Scalar>(quad_size_));

  // Target first: Unity locks its view to the last added quadrotor (the chaser).
  target_quad_ = std::make_shared<Quadrotor>();
  target_quad_->setSize(quad_size);
  target_state_.setZero();
  target_quad_->reset(target_state_);

  chaser_quad_ = std::make_shared<Quadrotor>();
  chaser_quad_->setSize(quad_size);
  chaser_state_.setZero();
  chaser_quad_->reset(chaser_state_);

  setupCameras();

  it_ = std::make_unique<image_transport::ImageTransport>(nh_);
  left_pub_ = it_->advertise(left_topic_, 1);
  right_pub_ = it_->advertise(right_topic_, 1);

  chaser_sub_ = nh_.subscribe(chaser_odom_topic_, 1, &FlightmareRenderer::chaserOdomCallback, this);
  target_sub_ = nh_.subscribe(target_odom_topic_, 1, &FlightmareRenderer::targetOdomCallback, this);

  // Wait for Gazebo and the Unity binary to come up before connecting.
  ros::Duration(unity_startup_delay_).sleep();
  connectUnity();

  render_timer_ = nh_.createTimer(ros::Rate(render_rate_), &FlightmareRenderer::renderLoop, this);
  ROS_INFO("[flightmare_renderer] Publishing stereo images on %s and %s at %.0f Hz.",
           left_pub_.getTopic().c_str(), right_pub_.getTopic().c_str(), render_rate_);
}

void FlightmareRenderer::loadParams() {
  pnh_.param("unity_render", unity_render_, true);
  pnh_.param("scene_id", scene_id_, 1);  // UnityScene::WAREHOUSE
  pnh_.param("main_loop_freq", render_rate_, 50.0);
  pnh_.param("unity_startup_delay", unity_startup_delay_, 5.0);
  pnh_.param("render_z_offset", render_z_offset_, 50.0);
  double yaw_offset_deg = -135.0;
  pnh_.param("render_yaw_offset_deg", yaw_offset_deg, yaw_offset_deg);
  render_yaw_offset_ = yaw_offset_deg * kDegToRad;
  pnh_.param("quad_size", quad_size_, 0.5);

  pnh_.param<std::string>("chaser_odometry_topic", chaser_odom_topic_, "/chaser_drone/ground_truth/odometry");
  pnh_.param<std::string>("target_odometry_topic", target_odom_topic_, "/target_drone/ground_truth/odometry");
  pnh_.param<std::string>("left_image_topic", left_topic_, "camera_left");
  pnh_.param<std::string>("right_image_topic", right_topic_, "camera_right");

  // Shared with camera_tf_publisher.py (config/cameras.yaml)
  pnh_.param("cameras/width", image_width_, 640);
  pnh_.param("cameras/height", image_height_, 360);
  pnh_.param("cameras/fov_deg", fov_deg_, 60.0);
  double tilt_deg = 15.0;
  pnh_.param("cameras/tilt_deg", tilt_deg, tilt_deg);
  tilt_ = tilt_deg * kDegToRad;
  left_cfg_.position_body = loadVector(pnh_, "cameras/left/position", Eigen::Vector3d(0.1, 0.1, 0.3));
  right_cfg_.position_body = loadVector(pnh_, "cameras/right/position", Eigen::Vector3d(0.1, -0.1, 0.3));
  pnh_.param<std::string>("cameras/left/optical_frame", left_frame_, "chaser_drone/camera_left_optical");
  pnh_.param<std::string>("cameras/right/optical_frame", right_frame_, "chaser_drone/camera_right_optical");
}

void FlightmareRenderer::setupCameras() {
  // Gazebo body B -> Flightmare body F: the rendered model is q_gazebo * Rz(offset),
  // hence p_F = Rz(offset)^T p_B.
  const Eigen::Matrix3d R_FB =
      Eigen::AngleAxisd(-render_yaw_offset_, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  // A Flightmare camera with identity R_BC looks along +y_F (ROS->Unity axis mapping).
  // We want it to look along +x_B = R_FB e_x, so yaw it by (-offset - 90 deg), then tilt it
  // up about its own x-axis (paper eq. 1: R_BC = Rz(45 deg) Rx(15 deg) for offset = -135 deg).
  const Eigen::Matrix3d R_FC =
      (Eigen::AngleAxisd(-render_yaw_offset_ - 0.5 * M_PI, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(tilt_, Eigen::Vector3d::UnitX())).toRotationMatrix();

  const auto make_camera = [&](const CameraConfig& cfg) {
    auto cam = std::make_shared<RGBCamera>();
    cam->setFOV(static_cast<Scalar>(fov_deg_));
    cam->setWidth(image_width_);
    cam->setHeight(image_height_);
    Vector<3> B_r_BC = (R_FB * cfg.position_body).cast<Scalar>();
    Matrix<3, 3> R_BC = R_FC.cast<Scalar>();
    cam->setRelPose(B_r_BC, R_BC);
    cam->setPostProcesscing(std::vector<bool>{false, false, false});
    chaser_quad_->addRGBCamera(cam);
    return cam;
  };
  left_camera_ = make_camera(left_cfg_);
  right_camera_ = make_camera(right_cfg_);
}

void FlightmareRenderer::toQuadState(const nav_msgs::Odometry& msg, QuadState& state) const {
  state.x[QS::POSX] = static_cast<Scalar>(msg.pose.pose.position.x);
  state.x[QS::POSY] = static_cast<Scalar>(msg.pose.pose.position.y);
  state.x[QS::POSZ] = static_cast<Scalar>(msg.pose.pose.position.z + render_z_offset_);

  const auto& o = msg.pose.pose.orientation;
  const Eigen::Quaterniond q_gazebo(o.w, o.x, o.y, o.z);
  // Right-multiplication: spin the model in place without changing the world frame.
  const Eigen::Quaterniond q_rendered =
      q_gazebo * Eigen::Quaterniond(Eigen::AngleAxisd(render_yaw_offset_, Eigen::Vector3d::UnitZ()));

  state.x[QS::ATTW] = static_cast<Scalar>(q_rendered.w());
  state.x[QS::ATTX] = static_cast<Scalar>(q_rendered.x());
  state.x[QS::ATTY] = static_cast<Scalar>(q_rendered.y());
  state.x[QS::ATTZ] = static_cast<Scalar>(q_rendered.z());
}

void FlightmareRenderer::chaserOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  toQuadState(*msg, chaser_state_);
  chaser_quad_->setState(chaser_state_);
}

void FlightmareRenderer::targetOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  toQuadState(*msg, target_state_);
  target_quad_->setState(target_state_);
}

void FlightmareRenderer::renderLoop(const ros::TimerEvent& event) {
  if (!unity_render_ || !unity_ready_) return;

  unity_bridge_->getRender(0);
  unity_bridge_->handleOutput();

  // Both images carry the same stamp so the stereo synchronizer pairs them exactly.
  std_msgs::Header header;
  header.stamp = event.current_real;

  cv::Mat img;
  if (left_camera_->getRGBImage(img)) {
    header.frame_id = left_frame_;
    left_pub_.publish(cv_bridge::CvImage(header, "bgr8", img).toImageMsg());
  }
  if (right_camera_->getRGBImage(img)) {
    header.frame_id = right_frame_;
    right_pub_.publish(cv_bridge::CvImage(header, "bgr8", img).toImageMsg());
  }
}

bool FlightmareRenderer::connectUnity() {
  if (!unity_render_) return false;
  unity_bridge_ = UnityBridge::getInstance();
  unity_bridge_->addQuadrotor(target_quad_);
  unity_bridge_->addQuadrotor(chaser_quad_);
  unity_ready_ = unity_bridge_->connectUnity(static_cast<SceneID>(scene_id_));
  if (!unity_ready_) ROS_ERROR("[flightmare_renderer] Could not connect to Unity.");
  return unity_ready_;
}

}  // namespace viri_sim
