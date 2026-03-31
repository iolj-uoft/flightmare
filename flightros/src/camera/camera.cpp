
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

  // unity quadrotor
  std::shared_ptr<Quadrotor> quad_ptr = std::make_shared<Quadrotor>();
  // define quadsize scale (for unity visualization only)
  Vector<3> quad_size(0.5, 0.5, 0.5);
  quad_ptr->setSize(quad_size);
  QuadState quad_state;

  //
  std::shared_ptr<RGBCamera> rgb_camera = std::make_shared<RGBCamera>();

  // Flightmare(Unity3D)
  std::shared_ptr<UnityBridge> unity_bridge_ptr = UnityBridge::getInstance();
  SceneID scene_id{UnityScene::GARAGE};
  bool unity_ready{false};

  // initialize publishers
  image_transport::ImageTransport it(pnh);
  rgb_pub = it.advertise("/rgb", 1);
  depth_pub = it.advertise("/depth", 1);
  // segmentation_pub = it.advertise("/segmentation", 1);
  // opticalflow_pub = it.advertise("/opticalflow", 1);

  rgb_pub = it.advertise("/hummingbird/camera/rgb", 1);
  depth_pub = it.advertise("/hummingbird/camera/depth", 1);
  segmentation_pub = it.advertise("/hummingbird/camera/segmentation", 1);
  opticalflow_pub = it.advertise("/hummingbird/camera/opticalflow", 1);

  // Flightmare
  Vector<3> B_r_BC(0.0, 0.0, 0.3);
  Matrix<3, 3> R_BC = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  std::cout << R_BC << std::endl;
  rgb_camera->setFOV(90);
  rgb_camera->setWidth(640);
  rgb_camera->setHeight(360);
  rgb_camera->setRelPose(B_r_BC, R_BC);
  rgb_camera->setPostProcesscing(
    std::vector<bool>{false, false, false});  // depth, segmentation, optical flow
  quad_ptr->addRGBCamera(rgb_camera);

  // initialization
  quad_state.setZero();
  quad_ptr->reset(quad_state);

  // connect unity
  unity_bridge_ptr->addQuadrotor(quad_ptr);
  unity_ready = unity_bridge_ptr->connectUnity(scene_id);

  FrameID frame_id = 0;
  while (ros::ok() && unity_ready) {
    // 1. Define Circle Parameters
    double radius = 5.0;      // Radius in meters
    double omega = 1.0;       // Angular velocity (rad/s)
    double altitude = 2.0;    // Maintain a constant height of 2 meters

    // 2. Calculate current angle based on frame_id and loop rate (50Hz)
    // time = frame_id * (1.0 / 50.0)
    double current_time = frame_id * 0.02; 
    double angle = omega * current_time;

    // 3. Update States
    quad_state.x[QS::POSX] = -radius * std::cos(angle) + 60.0;
    quad_state.x[QS::POSY] = -radius * std::sin(angle) + 50.0;
    quad_state.x[QS::POSZ] = altitude + 30.0;

    // 4. Optional: Make the drone face the center of the circle
    // This keeps the camera pointed at the origin (useful for target tracking)
    double yaw = angle + M_PI; 
    quad_state.x[QS::ATTW] = std::cos(yaw / 2.0);
    quad_state.x[QS::ATTZ] = std::sin(yaw / 2.0);

    quad_ptr->setState(quad_state);

    if (frame_id % 5 == 0) { // Only render every 5th step
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

    rgb_camera->getSegmentation(img);
    sensor_msgs::ImagePtr segmentation_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    segmentation_msg->header.stamp = timestamp;
    segmentation_pub.publish(segmentation_msg);

    rgb_camera->getOpticalFlow(img);
    sensor_msgs::ImagePtr opticflow_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    opticflow_msg->header.stamp = timestamp;
    opticalflow_pub.publish(opticflow_msg);

    frame_id += 1;
  }

  return 0;
}
