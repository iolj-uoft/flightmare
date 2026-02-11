// ros
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/bridges/unity_message_types.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

using namespace flightlib;

int main(int argc, char *argv[]) {
  // initialize ROS
  ros::init(argc, argv, "camera_example");
  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");
  ros::Rate(50.0);

  // publisher
  image_transport::Publisher rgb_pub;
  image_transport::Publisher depth_pub;
  image_transport::Publisher segmentation_pub;
  image_transport::Publisher opticalflow_pub;

  // Create target quadrotor in Unity
  std::shared_ptr<Quadrotor> target_quad_ptr = std::make_shared<Quadrotor>();
  // define quadsize scale (for unity visualization only)
  Vector<3> target_quad_size(0.5, 0.5, 0.5);
  target_quad_ptr->setSize(target_quad_size);
  QuadState target_quad_state;

  // Create chasing quadrotor in Unity
  std::shared_ptr<Quadrotor> chaser_quad_ptr = std::make_shared<Quadrotor>();
  // define quadsize scale (for unity visualization only)
  Vector<3> chaser_quad_size(0.5, 0.5, 0.5);
  chaser_quad_ptr->setSize(chaser_quad_size);
  QuadState chaser_quad_state;

  // create camera
  std::shared_ptr<RGBCamera> rgb_camera = std::make_shared<RGBCamera>();

  // Flightmare(Unity3D)
  std::shared_ptr<UnityBridge> unity_bridge_ptr = UnityBridge::getInstance();
  SceneID scene_id{UnityScene::GARAGE};
  bool unity_ready{false};

  // initialize publishers
  image_transport::ImageTransport it(pnh);
  rgb_pub = it.advertise("/rgb", 1);
  depth_pub = it.advertise("/depth", 1);

  // Flightmare
  Vector<3> B_r_BC(0.0, 0.0, 0.3);
  Matrix<3, 3> R_BC = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  std::cout << R_BC << std::endl;
  rgb_camera->setFOV(60);
  rgb_camera->setWidth(1920);
  rgb_camera->setHeight(1080);
  rgb_camera->setRelPose(B_r_BC, R_BC);
  rgb_camera->setPostProcesscing(
    std::vector<bool>{false, false, false});  // depth, segmentation, optical flow
  chaser_quad_ptr->addRGBCamera(rgb_camera);

  // initialization
  target_quad_state.setZero();
  target_quad_ptr->reset(target_quad_state);
  chaser_quad_state.setZero();
  chaser_quad_ptr->reset(chaser_quad_state);
  
  // trajectory parameters
  const double target_speed = 1.0;       // m/s along +X
  const double follow_distance = 2.0;    // chaser stays this far behind target along X
  const double start_x = 50.0;           // initial X for target
  const double start_y = 50.0;           // initial Y for both
  const double start_z = 30.0;           // altitude
  // initialize start positions
  target_quad_state.x[QS::POSX] = start_x;
  target_quad_state.x[QS::POSY] = start_y;
  target_quad_state.x[QS::POSZ] = start_z;
  chaser_quad_state.x[QS::POSX] = start_x - follow_distance;
  chaser_quad_state.x[QS::POSY] = start_y;
  chaser_quad_state.x[QS::POSZ] = start_z;
  target_quad_ptr->setState(target_quad_state);
  chaser_quad_ptr->setState(chaser_quad_state);

  // connect unity
  try {
    unity_bridge_ptr->addQuadrotor(target_quad_ptr);
    ROS_INFO("Added target_quad_ptr (%p) to UnityBridge", target_quad_ptr.get());
    unity_bridge_ptr->addQuadrotor(chaser_quad_ptr);
    ROS_INFO("Added chaser_quad_ptr (%p) to UnityBridge", chaser_quad_ptr.get());
    unity_ready = unity_bridge_ptr->connectUnity(scene_id);
    ROS_INFO("UnityBridge connectUnity returned: %d", (int)unity_ready);
  } catch (const std::exception &e) {
    ROS_ERROR("UnityBridge/ZMQ failed to connect: %s", e.what());
    ros::shutdown();
    return -1;
  }
  
  FrameID frame_id = 0;
  while (ros::ok() && unity_ready) {
    // time (seconds)
    double current_time = frame_id * 0.02;

    // Target: straight-line motion along +X at constant speed
    double target_x = start_x + target_speed * current_time;
    double target_y = start_y;
    double target_z = start_z;
    target_quad_state.x[QS::POSX] = target_x;
    target_quad_state.x[QS::POSY] = target_y;
    target_quad_state.x[QS::POSZ] = target_z;
    // target facing along +X
    double target_yaw = -M_PI / 2;
    target_quad_state.x[QS::ATTW] = std::cos(target_yaw / 2.0);
    target_quad_state.x[QS::ATTZ] = std::sin(target_yaw / 2.0);

    // Chaser: follow behind target at fixed distance along X
    double chaser_x = target_x - follow_distance;
    double chaser_y = target_y;
    double chaser_z = target_z;
    chaser_quad_state.x[QS::POSX] = chaser_x;
    chaser_quad_state.x[QS::POSY] = chaser_y;
    chaser_quad_state.x[QS::POSZ] = chaser_z;
    // chaser yaw: face toward the target (compute yaw from chaser->target vector)
    double dx = target_x - chaser_x;
    double dy = target_y - chaser_y;
    double chaser_yaw = -M_PI / 2;
    chaser_quad_state.x[QS::ATTW] = std::cos(chaser_yaw / 2.0);
    chaser_quad_state.x[QS::ATTZ] = std::sin(chaser_yaw / 2.0);

    target_quad_ptr->setState(target_quad_state);
    chaser_quad_ptr->setState(chaser_quad_state);

    if (frame_id % 1 == 0) { // Only render every 5th step
      unity_bridge_ptr->getRender(frame_id);
      unity_bridge_ptr->handleOutput();
    }

    cv::Mat img;
    
    ros::Time timestamp = ros::Time::now();

    rgb_camera->getRGBImage(img);
    sensor_msgs::ImagePtr rgb_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    rgb_msg->header.stamp = timestamp;
    rgb_pub.publish(rgb_msg);

    rgb_camera->getDepthMap(img);
    sensor_msgs::ImagePtr depth_msg =
      cv_bridge::CvImage(std_msgs::Header(), "32FC1", img).toImageMsg();
    depth_msg->header.stamp = timestamp;
    depth_pub.publish(depth_msg);

    frame_id += 1;
  }

  return 0;
}
