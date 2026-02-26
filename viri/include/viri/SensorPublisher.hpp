#ifndef SENSOR_PUBLISHER_HPP
#define SENSOR_PUBLISHER_HPP

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <flightlib/sensors/rgb_camera.hpp>
#include <memory>

class SensorPublisher {
public: 
    SensorPublisher(ros::NodeHandle& nh, std::shared_ptr<flightlib::RGBCamera> camera);

    void publishImages();

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher rgb_pub_;
    image_transport::Publisher depth_pub_;

    std::shared_ptr<flightlib::RGBCamera> rgb_camera_;
};

#endif 