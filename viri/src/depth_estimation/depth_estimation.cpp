#include "viri/depth_estimation.hpp"
 // <-- Add this header
#include <cmath>

namespace viri {

DepthEstimator::DepthEstimator(const ros::NodeHandle& nh, const ros::NodeHandle& pnh) 
  : nh_(nh), pnh_(pnh) {
    
    pnh_.param("focal_length", f_, 554.25);
    pnh_.param("baseline", B_, 0.2);
    pnh_.param("c_x", cx_, 320.0);
    pnh_.param("c_y", cy_, 180.0);
    pnh_.param("max_y_jitter", max_y_jitter_, 15.0);

    std::string bbox_topic, target_pose_topic;
    pnh_.param("bbox_topic", bbox_topic, std::string("/ultralytics/detection/target_centers"));
    pnh_.param("target_position_topic", target_pose_topic, std::string("/chaser_drone/target_position_relative"));

    // Update the callback signature
    bbox_sub_ = nh_.subscribe(bbox_topic, 1, &DepthEstimator::bboxCallback, this);
    pose_pub_ = nh_.advertise<geometry_msgs::PointStamped>(target_pose_topic, 1);

    ROS_INFO("[%s] Depth Estimation initialized using vision_msgs.", pnh_.getNamespace().c_str());
}

// Update the callback argument to take the new message type
void DepthEstimator::bboxCallback(const vision_msgs::Detection2DArray::ConstPtr& msg) {
    if (msg->detections.empty()) {
        return; // No target detected, let EKF coast
    }

    // Extract the first detection's bounding box
    auto bbox = msg->detections[0].bbox;

    // Convert center/size format back to your Left/Right format
    double x_l = bbox.center.x - (bbox.size_x / 2.0);
    double x_r = bbox.center.x + (bbox.size_x / 2.0);
    double y_l = bbox.center.y - (bbox.size_y / 2.0);
    double y_r = bbox.center.y + (bbox.size_y / 2.0);

    // Edge Rejection Filter
    double edge_margin = 50.0; 
    double image_width = 640.0; 
    
    if (x_l < edge_margin || x_l > (image_width - edge_margin) || 
        x_r < edge_margin || x_r > (image_width - edge_margin)) {
        ROS_WARN_THROTTLE(0.5, "Target at edge of FOV. Dropping frame.");
        return; 
    }

    // 1. Jitter Filter
    if (std::abs(y_l - y_r) > max_y_jitter_) {
        ROS_WARN_THROTTLE(0.5, "YOLO Jitter detected! Rejecting frame.");
        return;
    }

    // 2. Disparity Calculation
    double d = x_l - x_r;
    if (d <= 0.5) return; 

    // 3. Triangulation Math
    double Z = (f_ * B_) / d;                  
    double X = ((x_l - cx_) * Z) / f_;         
    double y_avg = (y_l + y_r) / 2.0;
    double Y = ((y_avg - cy_) * Z) / f_;       

    // 4. Publish 3D Point
    geometry_msgs::PointStamped target_point;
    
    // ==========================================
    // CRITICAL FIX: Use the original image stamp!
    // ==========================================
    target_point.header.stamp = msg->header.stamp; 
    
    target_point.header.frame_id = "chaser_drone/camera_left_optical"; 
    
    target_point.point.x = X;
    target_point.point.y = Y;
    target_point.point.z = Z;

    pose_pub_.publish(target_point);
}

}  // namespace viri