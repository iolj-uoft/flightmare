// Corrupts chaser odometry before it reaches the LTVKF prediction step (paper Sec. III-B.2):
// zero-mean Gaussian velocity noise and a random-walk position bias modelling IMU drift.
// Ground truth stays untouched for guidance evaluation and intercept detection.

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <cmath>
#include <random>
#include <string>

namespace viri_sim {

class OdomNoiseInjector {
 public:
  OdomNoiseInjector(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    pnh.param("vel_noise_std", vel_noise_std_, 0.15);
    pnh.param("ang_vel_noise_scale", ang_vel_noise_scale_, 0.5);
    pnh.param("pos_drift_std", pos_drift_std_, 0.005);
    int seed = -1;
    pnh.param("seed", seed, seed);
    gen_.seed(seed < 0 ? std::random_device{}() : static_cast<unsigned>(seed));

    std::string input_topic, output_topic;
    pnh.param<std::string>("input_topic", input_topic, "/chaser_drone/ground_truth/odometry");
    pnh.param<std::string>("output_topic", output_topic, "noisy_odometry");

    sub_ = nh.subscribe(input_topic, 10, &OdomNoiseInjector::callback, this);
    pub_ = nh.advertise<nav_msgs::Odometry>(output_topic, 10);
  }

 private:
  void callback(const nav_msgs::Odometry::ConstPtr& msg) {
    nav_msgs::Odometry out = *msg;
    std::normal_distribution<double> vel(0.0, vel_noise_std_);
    std::normal_distribution<double> drift(0.0, pos_drift_std_);

    auto& tw = out.twist.twist;
    tw.linear.x += vel(gen_);
    tw.linear.y += vel(gen_);
    tw.linear.z += vel(gen_);
    tw.angular.x += ang_vel_noise_scale_ * vel(gen_);
    tw.angular.y += ang_vel_noise_scale_ * vel(gen_);
    tw.angular.z += ang_vel_noise_scale_ * vel(gen_);

    bias_x_ += drift(gen_);
    bias_y_ += drift(gen_);
    bias_z_ += drift(gen_);
    out.pose.pose.position.x += bias_x_;
    out.pose.pose.position.y += bias_y_;
    out.pose.pose.position.z += bias_z_;

    out.pose.covariance.fill(0.0);
    out.twist.covariance.fill(0.0);
    const double pos_var = std::pow(pos_drift_std_, 2) * 1000.0;  // inflated: drift accumulates
    const double vel_var = std::pow(vel_noise_std_, 2);
    for (int i = 0; i < 3; ++i) {
      out.pose.covariance[i * 7] = pos_var;
      out.twist.covariance[i * 7] = vel_var;
      out.twist.covariance[21 + i * 7] = std::pow(ang_vel_noise_scale_ * vel_noise_std_, 2);
    }
    pub_.publish(out);
  }

  ros::Subscriber sub_;
  ros::Publisher pub_;
  double vel_noise_std_ = 0.15;
  double ang_vel_noise_scale_ = 0.5;
  double pos_drift_std_ = 0.005;
  double bias_x_ = 0.0, bias_y_ = 0.0, bias_z_ = 0.0;
  std::mt19937 gen_;
};

}  // namespace viri_sim

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_noise_injector");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  viri_sim::OdomNoiseInjector node(nh, pnh);
  ros::spin();
  return 0;
}
