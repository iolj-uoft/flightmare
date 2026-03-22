#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <random>

class OdomNoiseInjector {
private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher noisy_odom_pub_;

    // Noise parameters
    double vel_noise_std_;
    double pos_drift_std_;

    // Running bias for random walk
    double pos_bias_x_ = 0.0;
    double pos_bias_y_ = 0.0;
    double pos_bias_z_ = 0.0;

    // Random number generation
    std::mt19937 gen_;

public:
    OdomNoiseInjector(ros::NodeHandle& nh) : nh_(nh) {
        // Load parameters
        ros::NodeHandle pnh("~");
        pnh.param("vel_noise_std", vel_noise_std_, 0.15);
        pnh.param("pos_drift_std", pos_drift_std_, 0.005);

        // Initialize random seed
        std::random_device rd;
        gen_ = std::mt19937(rd());

        odom_sub_ = nh_.subscribe("/chaser_drone/ground_truth/odometry", 1, &OdomNoiseInjector::odomCallback, this);
        noisy_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("noisy_odometry", 10);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        nav_msgs::Odometry noisy_msg = *msg; // Copy original
        
        std::normal_distribution<double> vel_dist(0.0, vel_noise_std_);
        std::normal_distribution<double> pos_dist(0.0, pos_drift_std_);

        // 1. Inject Jitter into Velocities
        noisy_msg.twist.twist.linear.x += vel_dist(gen_);
        noisy_msg.twist.twist.linear.y += vel_dist(gen_);
        noisy_msg.twist.twist.linear.z += vel_dist(gen_);
        
        noisy_msg.twist.twist.angular.x += vel_dist(gen_) * 0.5;
        noisy_msg.twist.twist.angular.y += vel_dist(gen_) * 0.5;
        noisy_msg.twist.twist.angular.z += vel_dist(gen_) * 0.5;

        // 2. Inject Random Walk into Position
        pos_bias_x_ += pos_dist(gen_);
        pos_bias_y_ += pos_dist(gen_);
        pos_bias_z_ += pos_dist(gen_);

        noisy_msg.pose.pose.position.x += pos_bias_x_;
        noisy_msg.pose.pose.position.y += pos_bias_y_;
        noisy_msg.pose.pose.position.z += pos_bias_z_;

        // 3. Populate Covariance
        for (int i = 0; i < 36; ++i) {
            noisy_msg.pose.covariance[i] = 0.0;
            noisy_msg.twist.covariance[i] = 0.0;
        }
        
        // Inflated position variance
        double pos_var = std::pow(pos_drift_std_, 2) * 1000.0;
        noisy_msg.pose.covariance[0]  = pos_var;
        noisy_msg.pose.covariance[7]  = pos_var;
        noisy_msg.pose.covariance[14] = pos_var;

        // Velocity variance
        double vel_var = std::pow(vel_noise_std_, 2);
        noisy_msg.twist.covariance[21] = vel_var;
        noisy_msg.twist.covariance[28] = vel_var;
        noisy_msg.twist.covariance[35] = vel_var;

        noisy_odom_pub_.publish(noisy_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_noise_injector");
    ros::NodeHandle nh;
    OdomNoiseInjector injector(nh);
    ros::spin();
    return 0;
}