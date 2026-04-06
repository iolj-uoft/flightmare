#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>

/**
 * Monitors and publishes ground truth states for debugging
 */
class StateMonitorNode {
public:
    StateMonitorNode(const ros::NodeHandle& nh) : nh_(nh) {
        nh_.param<double>("publish_freq", publish_freq_, 10.0);
        
        chaser_odom_sub_ = nh_.subscribe(
            "/chaser_drone/ground_truth/odometry", 1,
            &StateMonitorNode::chaserOdomCallback, this);
        
        target_odom_sub_ = nh_.subscribe(
            "/target_drone/ground_truth/odometry", 1,
            &StateMonitorNode::targetOdomCallback, this);
        
        // Publish relative state
        rel_state_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(
            "/rl_agent/relative_state_gt", 1);
        
        // Timer for publishing
        timer_ = nh_.createTimer(
            ros::Duration(1.0 / publish_freq_),
            &StateMonitorNode::timerCallback, this);
    }
    
private:
    ros::NodeHandle nh_;
    ros::Subscriber chaser_odom_sub_;
    ros::Subscriber target_odom_sub_;
    ros::Publisher rel_state_pub_;
    ros::Timer timer_;
    
    nav_msgs::Odometry chaser_odom_;
    nav_msgs::Odometry target_odom_;
    bool chaser_odom_received_ = false;
    bool target_odom_received_ = false;
    
    double publish_freq_;
    
    void chaserOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        chaser_odom_ = *msg;
        chaser_odom_received_ = true;
    }
    
    void targetOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        target_odom_ = *msg;
        target_odom_received_ = true;
    }
    
    void timerCallback(const ros::TimerEvent&) {
        if (!chaser_odom_received_ || !target_odom_received_) {
            return;
        }
        
        // Relative position
        Eigen::Vector3d chaser_pos(
            chaser_odom_.pose.pose.position.x,
            chaser_odom_.pose.pose.position.y,
            chaser_odom_.pose.pose.position.z
        );
        
        Eigen::Vector3d target_pos(
            target_odom_.pose.pose.position.x,
            target_odom_.pose.pose.position.y,
            target_odom_.pose.pose.position.z
        );
        
        Eigen::Vector3d rel_pos = target_pos - chaser_pos;
        
        // Relative velocity
        Eigen::Vector3d chaser_vel(
            chaser_odom_.twist.twist.linear.x,
            chaser_odom_.twist.twist.linear.y,
            chaser_odom_.twist.twist.linear.z
        );
        
        Eigen::Vector3d target_vel(
            target_odom_.twist.twist.linear.x,
            target_odom_.twist.twist.linear.y,
            target_odom_.twist.twist.linear.z
        );
        
        Eigen::Vector3d rel_vel = target_vel - chaser_vel;
        
        // Publish
        std_msgs::Float64MultiArray msg;
        msg.data.resize(6);
        msg.data[0] = rel_pos(0);
        msg.data[1] = rel_pos(1);
        msg.data[2] = rel_pos(2);
        msg.data[3] = rel_vel(0);
        msg.data[4] = rel_vel(1);
        msg.data[5] = rel_vel(2);
        
        rel_state_pub_.publish(msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_monitor_node");
    ros::NodeHandle nh("~");
    
    StateMonitorNode monitor(nh);
    
    ros::spin();
    
    return 0;
}
