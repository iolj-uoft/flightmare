#pragma once

#include <ros/ros.h>
#include <string>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PointStamped.h>
#include <vision_msgs/Detection2DArray.h>

namespace viri {

class DepthEstimator {
public:
    DepthEstimator(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
    ~DepthEstimator() = default;

private:
    // Callback for the synchronized YOLO bounding boxes
    void bboxCallback(const vision_msgs::Detection2DArray::ConstPtr& msg);

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