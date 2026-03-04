#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <filesystem>

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_recorder");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    bool save_image = false;
    if (save_image) {
        std::string out_dir;
        pnh.param<std::string>("out_dir", out_dir, 
            std::string("/home/austin/Desktop/drone_images"));
        std::filesystem::create_directories(out_dir);

        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub = it.subscribe(
            "/rgb", 1,
            [out_dir](const sensor_msgs::ImageConstPtr& msg) {
                try {
                    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
                    auto t = std::chrono::system_clock::now();
                    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch()).count();
                    std::ostringstream ss;
                    ss << out_dir << "/img_" << ms << ".png";
                    cv::imwrite(ss.str(), img);
                }
                catch (const cv_bridge::Exception& e) {
                    ROS_WARN("cv_bridge exception: %s", e.what());
                }
            }
        );
        ROS_INFO("camera_recorder subscribing to /rgb, saving to %s", out_dir.c_str());
    }
    ros::spin();
    return 0;
}