#include "viri/depth_estimation.hpp"
#include <cmath>

namespace viri {

DepthEstimator::DepthEstimator(const ros::NodeHandle& nh, const ros::NodeHandle& pnh) 
  : nh_(nh), pnh_(pnh) {
    
    // Load parameters (Flightmare 640x360 default values)
    pnh_.param("focal_length", f_, 554.25);
    pnh_.param("baseline", B_, 0.2);
    pnh_.param("c_x", cx_, 320.0);
    pnh_.param("c_y", cy_, 180.0);
    pnh_.param("max_y_jitter", max_y_jitter_, 15.0);

    std::string bbox_topic, target_pose_topic;
    pnh_.param("bbox_topic", bbox_topic, std::string("/ultralytics/detection/target_centers"));
    pnh_.param("target_position_topic", target_pose_topic, std::string("/chaser_drone/target_position_relative"));

    bbox_sub_ = nh_.subscribe(bbox_topic, 1, &DepthEstimator::bboxCallback, this);
    pose_pub_ = nh_.advertise<geometry_msgs::PointStamped>(target_pose_topic, 1);

    ROS_INFO("[%s] Depth Estimation initialized.", pnh_.getNamespace().c_str());
    ROS_INFO("Baseline: %.2fm | Focal Length: %.2fpx", B_, f_);
}

void DepthEstimator::bboxCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    if (msg->data.size() < 4) {
        ROS_WARN_THROTTLE(1.0, "Received malformed bounding box array.");
        return;
    }

    double x_l = msg->data[0];
    double y_l = msg->data[1];
    double x_r = msg->data[2];
    double y_r = msg->data[3];

    // Edge Rejection Filter
    // If the center is within 40 pixels of the left or right edge, the box is 
    // likely clipping. Reject the measurement so the EKF can coast.
    double edge_margin = 40.0; 
    double image_width = 640.0; // Your camera width
    
    if (x_l < edge_margin || x_l > (image_width - edge_margin) || 
        x_r < edge_margin || x_r > (image_width - edge_margin)) {
        
        ROS_WARN_THROTTLE(0.5, "Target at edge of FOV. Box likely clipping. Dropping frame.");
        return; // Exit without publishing
    }

    // 1. Jitter Filter
    if (std::abs(y_l - y_r) > max_y_jitter_) {
        ROS_WARN_THROTTLE(0.5, "YOLO Jitter detected! Left Y: %.1f, Right Y: %.1f. Rejecting frame.", y_l, y_r);
        return;
    }

    // 2. Disparity Calculation
    double d = x_l - x_r;
    if (d <= 0.5) {
        // Target is too far, or boxes are swapped/invalid
        return; 
    }

    // 3. Triangulation Math
    double Z = (f_ * B_) / d;                  
    double X = ((x_l - cx_) * Z) / f_;         
    double y_avg = (y_l + y_r) / 2.0;
    double Y = ((y_avg - cy_) * Z) / f_;       

    // 4. Publish 3D Point
    geometry_msgs::PointStamped target_point;
    target_point.header.stamp = ros::Time::now();
    target_point.header.frame_id = "chaser_drone/camera_left_optical"; 
    
    target_point.point.x = X;
    target_point.point.y = Y;
    target_point.point.z = Z;

    pose_pub_.publish(target_point);
}

}  // namespace viri