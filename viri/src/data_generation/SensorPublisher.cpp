#include "flightros/viri/SensorPublisher.hpp"
#include <sensor_msgs/Image.h>

SensorPublisher::SensorPublisher(ros::NodeHandle& nh, std::shared_ptr<flightlib::RGBCamera> camera)
    : nh_(nh), it_(nh), rgb_camera_(camera) {
        rgb_pub_ = it_.advertise("/rgb", 1);
        depth_pub_ = it_.advertise("/depth", 1);
    }

void SensorPublisher::publishImages() {
    if (!rgb_camera_) return;

    ros::Time timestamp = ros::Time::now();

    // 1. Fetch and Publish RGB
    cv::Mat rgb_img;
    rgb_camera_->getRGBImage(rgb_img); 
    if (!rgb_img.empty()) {
        sensor_msgs::ImagePtr rgb_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_img).toImageMsg();
        rgb_msg->header.stamp = timestamp;
        rgb_pub_.publish(rgb_msg);
    }

    // 2. Fetch and Publish Depth
    // cv::Mat depth_img;
    // rgb_camera_->getDepthMap(depth_img);
    // if (!depth_img.empty()) {
    //     // Ensure the encoding matches what flightlib provides (usually 32FC1)
    //     sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", depth_img).toImageMsg();
    //     depth_msg->header.stamp = timestamp;
    //     depth_pub_.publish(depth_msg);
    // }
    
}