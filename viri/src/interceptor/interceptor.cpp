#include "flightros/interceptor/interceptor.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

namespace flightros {

FlightPilot::FlightPilot(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
  : nh_(nh),
    pnh_(pnh),
    scene_id_(UnityScene::WAREHOUSE),
    unity_ready_(false),
    unity_render_(false),
    receive_id_(0),
    main_loop_freq_(50.0) {

  // load parameters
  if (!loadParams()) {
    ROS_WARN("[%s] Could not load all parameters.", pnh_.getNamespace().c_str());
  } else {
    ROS_INFO("[%s] Loaded all parameters.", pnh_.getNamespace().c_str());
  }

  Vector<3> quad_size(0.5, 0.5, 0.5);
  // --- 1. Target Quad Initialization ---
  target_quad_ptr_ = std::make_shared<Quadrotor>();
  target_quad_ptr_->setSize(quad_size);
  target_quad_state_.setZero();
  target_quad_ptr_->reset(target_quad_state_);

  // --- 2. Interceptor Quad Initialization ---
  quad_ptr_ = std::make_shared<Quadrotor>();
  quad_ptr_->setSize(quad_size);
  quad_state_.setZero();
  quad_ptr_->reset(quad_state_);

  // --- 3. Stereo Camera Setup ---
  rgb_camera_left_ = std::make_shared<RGBCamera>();
  rgb_camera_right_ = std::make_shared<RGBCamera>();

  Matrix<3, 3> R_BC = Eigen::AngleAxisf(M_PI / 4.0, Eigen::Vector3f::UnitZ()).toRotationMatrix();

  // Left Camera: offset by -0.1m in X (creating a 20cm stereo baseline) and +0.1m in Y 
  Vector<3> B_r_BC_left(-0.1414, 0.0, 0.3);
  rgb_camera_left_->setFOV(60);
  rgb_camera_left_->setWidth(640);
  rgb_camera_left_->setHeight(360);
  rgb_camera_left_->setRelPose(B_r_BC_left, R_BC);
  rgb_camera_left_->setPostProcesscing(std::vector<bool>{false, false, false});
  quad_ptr_->addRGBCamera(rgb_camera_left_);

  // Right Camera: offset by 0.1m in X (creating a 20cm stereo baseline) and +0.1m in Y 
  Vector<3> B_r_BC_right(0.0, 0.1414, 0.3);
  rgb_camera_right_->setFOV(60);
  rgb_camera_right_->setWidth(640);
  rgb_camera_right_->setHeight(360);
  rgb_camera_right_->setRelPose(B_r_BC_right, R_BC);
  rgb_camera_right_->setPostProcesscing(std::vector<bool>{false, false, false});
  quad_ptr_->addRGBCamera(rgb_camera_right_);
  
  // --- 4. ROS Publishers & Subscribers ---
  img_it_.reset(new image_transport::ImageTransport(nh_));
  rgb_pub_left_ = img_it_->advertise(rgb_topic_left_, 1);
  rgb_pub_right_ = img_it_->advertise(rgb_topic_right_, 1);
  ROS_INFO_STREAM("Publishing stereo images on: " << rgb_topic_left_ << " and " << rgb_topic_right_);

  sub_state_est_ = nh_.subscribe("/chaser_drone/ground_truth/odometry", 1, &FlightPilot::poseCallback, this);
  sub_target_state_est_ = nh_.subscribe("/target_drone/ground_truth/odometry", 1, &FlightPilot::targetPoseCallback, this);

  // Main loop timer
  timer_main_loop_ = nh_.createTimer(ros::Rate(main_loop_freq_), &FlightPilot::mainLoopCallback, this);

  // wait until the gazebo and unity are loaded
  ros::Duration(5.0).sleep();

  // connect unity
  setUnity(unity_render_);
  connectUnity();
}

FlightPilot::~FlightPilot() {}

void FlightPilot::poseCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  // 1. Pass the position directly from Gazebo WITHOUT any rotation
  quad_state_.x[QS::POSX] = (Scalar)msg->pose.pose.position.x;
  quad_state_.x[QS::POSY] = (Scalar)msg->pose.pose.position.y;
  quad_state_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z + 50.0; 

  // 2. Extract raw orientation
  Eigen::Quaterniond q_gazebo(msg->pose.pose.orientation.w, 
                              msg->pose.pose.orientation.x, 
                              msg->pose.pose.orientation.y, 
                              msg->pose.pose.orientation.z);

  // 3. Define the -90 degree (CW) visual offset
  Eigen::Quaterniond q_yaw_offset(Eigen::AngleAxisd(-M_PI / 2.0 - M_PI / 4.0, Eigen::Vector3d::UnitZ()));

  // 4. Multiply with the offset on the RIGHT (Local Rotation)
  // This spins the 3D model in place without changing the global coordinate frame
  Eigen::Quaterniond q_rendered = q_gazebo * q_yaw_offset;

  quad_state_.x[QS::ATTW] = (Scalar)q_rendered.w();
  quad_state_.x[QS::ATTX] = (Scalar)q_rendered.x();
  quad_state_.x[QS::ATTY] = (Scalar)q_rendered.y();
  quad_state_.x[QS::ATTZ] = (Scalar)q_rendered.z();

  quad_ptr_->setState(quad_state_);
}

void FlightPilot::targetPoseCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  // 1. Pass the position directly from Gazebo WITHOUT any rotation
  target_quad_state_.x[QS::POSX] = (Scalar)msg->pose.pose.position.x;
  target_quad_state_.x[QS::POSY] = (Scalar)msg->pose.pose.position.y;
  target_quad_state_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z + 50.0;

  // 2. Extract raw orientation
  Eigen::Quaterniond q_gazebo(msg->pose.pose.orientation.w, 
                              msg->pose.pose.orientation.x, 
                              msg->pose.pose.orientation.y, 
                              msg->pose.pose.orientation.z);

  // 3. Define the -90 degree (CW) visual offset
  Eigen::Quaterniond q_yaw_offset(Eigen::AngleAxisd(-M_PI / 2.0 - M_PI / 4.0, Eigen::Vector3d::UnitZ()));

  // 4. Multiply with the offset on the RIGHT (Local Rotation)
  Eigen::Quaterniond q_rendered = q_gazebo * q_yaw_offset;

  target_quad_state_.x[QS::ATTW] = (Scalar)q_rendered.w();
  target_quad_state_.x[QS::ATTX] = (Scalar)q_rendered.x();
  target_quad_state_.x[QS::ATTY] = (Scalar)q_rendered.y();
  target_quad_state_.x[QS::ATTZ] = (Scalar)q_rendered.z();
  
  target_quad_ptr_->setState(target_quad_state_);
}

void FlightPilot::mainLoopCallback(const ros::TimerEvent &event) {
  if (unity_render_ && unity_ready_) {
    // 1. Tell Unity to render the environment
    unity_bridge_ptr_->getRender(0);
    unity_bridge_ptr_->handleOutput();
    
    cv::Mat img_left, img_right;
    std_msgs::Header header;
    // Use the exact timer event time for perfect sync across both images
    header.stamp = event.current_real; 

    // 2. Publish Left Camera
    if (rgb_camera_left_ && rgb_pub_left_) {
      if (rgb_camera_left_->getRGBImage(img_left)) { 
        header.frame_id = "/camera_left"; 
        sensor_msgs::ImagePtr msg_left = cv_bridge::CvImage(header, "bgr8", img_left).toImageMsg();
        rgb_pub_left_.publish(msg_left);
      }
    }

    // 3. Publish Right Camera
    if (rgb_camera_right_ && rgb_pub_right_) {
      if (rgb_camera_right_->getRGBImage(img_right)) { 
        header.frame_id = "/camera_right"; 
        sensor_msgs::ImagePtr msg_right = cv_bridge::CvImage(header, "bgr8", img_right).toImageMsg();
        rgb_pub_right_.publish(msg_right);
      }
    }
  }
}

bool FlightPilot::setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr) {
    unity_bridge_ptr_ = UnityBridge::getInstance();
    
    // Add BOTH drones to the renderer
    unity_bridge_ptr_->addQuadrotor(target_quad_ptr_);
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);
    
    ROS_INFO("[%s] Unity Bridge is created.", pnh_.getNamespace().c_str());
  }
  return true;
}

bool FlightPilot::connectUnity() {
  if (!unity_render_ || unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}

bool FlightPilot::loadParams(void) {
  quadrotor_common::getParam("main_loop_freq", main_loop_freq_, pnh_);
  quadrotor_common::getParam("unity_render", unity_render_, pnh_);
  return true;
}

}  // namespace flightros