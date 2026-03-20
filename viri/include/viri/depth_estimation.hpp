#pragma once

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PointStamped.h>
#include <string>

namespace viri {

class DepthEstimator {
public:
    DepthEstimator(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
    ~DepthEstimator() = default;

private:
    // Callback for the synchronized YOLO bounding boxes
    void bboxCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber bbox_sub_;
    ros::Publisher pose_pub_;

    // Camera parameters
    double f_;
    double B_;
    double cx_;
    double cy_;
    double max_y_jitter_;
};

}  // namespace viri