#include <ros/ros.h>
#include "viri/depth_estimation.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "depth_estimation_node");
    
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // Instantiate the class
    viri::DepthEstimator estimator(nh, pnh);
    
    ros::spin();

    return 0;
}