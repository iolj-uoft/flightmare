#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <Eigen/Dense>

class RelativeESKF {
private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber yolo_sub_;
    ros::Publisher filtered_state_pub_;

    // --- The Core Math Variables ---
    Eigen::VectorXd x_; // Nominal State: [px, py, pz, vx, vy, vz]^T
    Eigen::MatrixXd P_; // Covariance Matrix (6x6)
    Eigen::MatrixXd Q_; // Process Noise (6x6)
    Eigen::MatrixXd R_; // Measurement Noise (3x3)

    // Time tracking for dt calculations
    ros::Time last_odom_time_;
    bool is_initialized_;
    
public:
    RelativeESKF(ros::NodeHandle& nh) : nh_(nh), is_initialized_(false) {
        // 1. Initialize Matrices
        x_ = Eigen::VectorXd::Zero(6);
        
        // P starts large because we don't know where the target is yet
        P_ = Eigen::MatrixXd::Identity(6, 6) * 10.0; 
        
        // Q represents the target's random-walk acceleration (from Eckenhoff 2019)
        Q_ = Eigen::MatrixXd::Identity(6, 6) * 0.1; 
        
        // R represents the YOLO stereo depth uncertainty
        R_ = Eigen::MatrixXd::Identity(3, 3) * 0.5;

        // 2. Setup ROS Plumbing
        // Notice we subscribe to the degraded ego-odometry!
        odom_sub_ = nh_.subscribe("noisy_odometry", 1, &RelativeESKF::predictionCallback, this);
        
        // Subscribe to your YOLO depth estimator output
        yolo_sub_ = nh_.subscribe("/target_position_relative", 1, &RelativeESKF::updateCallback, this);
        
        // The clean output for the RL agent
        filtered_state_pub_ = nh_.advertise<nav_msgs::Odometry>("filtered_relative_state", 10);

        ROS_INFO("Relative ESKF Node Initialized. Waiting for YOLO initialization...");
    }

    // Helper function: Converts a 3D vector into a 3x3 skew-symmetric cross-product matrix
    Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
        Eigen::Matrix3d S;
        S <<  0.0,  -v(2),   v(1),
              v(2),  0.0,  -v(0),
             -v(1),  v(0),   0.0;
        return S;
    }

    // --- The High-Speed Prediction Step (200Hz) ---
    void predictionCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        if (!is_initialized_) return; // Wait for the first YOLO point to anchor the filter

        ros::Time current_time = msg->header.stamp;
        double dt = (current_time - last_odom_time_).toSec();
        
        // Safety check: Prevent divide-by-zero or negative time jumps in simulation
        if (dt <= 0.0 || dt > 0.1) {
            last_odom_time_ = current_time;
            return; 
        }
`
        // 1. Extract drone inputs (The "Trusted Ego-State")
        Eigen::Vector3d v_drone(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
        Eigen::Vector3d w_drone(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);

        // 2. Extract current state variables
        Eigen::Vector3d p_rel = x_.head(3);
        Eigen::Vector3d v_tgt = x_.tail(3); // Target's velocity in the camera/body frame

        // 3. Kinematic Equations (Non-linear)
        // Rate of change of relative position
        Eigen::Vector3d p_dot = v_tgt - v_drone - w_drone.cross(p_rel);
        
        // Rate of change of target velocity in our rotating frame 
        // (Assuming target travels at constant velocity in the WORLD, it rotates inversely in our BODY frame)
        Eigen::Vector3d v_dot = -w_drone.cross(v_tgt);

        // 4. Integrate State (Euler Integration)
        x_.head(3) += p_dot * dt;
        x_.tail(3) += v_dot * dt;

        // 5. Calculate Continuous-Time Jacobian (F)
        // F = [ -[w_drone]_x ,      I       ]
        //     [      0       , -[w_drone]_x ]
        Eigen::MatrixXd F = Eigen::MatrixXd::Zero(6, 6);
        Eigen::Matrix3d w_skew = skew(w_drone);
        
        F.block<3, 3>(0, 0) = -w_skew;
        F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
        F.block<3, 3>(3, 3) = -w_skew;

        // 6. Discretize the Jacobian (Phi = I + F * dt)
        Eigen::MatrixXd Phi = Eigen::MatrixXd::Identity(6, 6) + F * dt;

        // 7. Update Covariance Matrix
        // P = Phi * P * Phi^T + Q * dt
        P_ = Phi * P_ * Phi.transpose() + Q_ * dt;

        last_odom_time_ = current_time;
        
        publishState(current_time);
    }

    // --- The Low-Speed Update Step (30Hz) ---
    void updateCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        if (!is_initialized_) {
            // First time seeing the target: Initialize position, assume zero relative velocity
            x_(0) = msg->point.x;
            x_(1) = msg->point.y;
            x_(2) = msg->point.z;
            last_odom_time_ = msg->header.stamp;
            is_initialized_ = true;
            ROS_INFO("ESKF Initialized with first YOLO measurement.");
            return;
        }

        // 1. Extract the YOLO measurement vector (z)
        Eigen::Vector3d z(msg->point.x, msg->point.y, msg->point.z);

        // 2. Define the Observation Matrix (H)
        // H maps our 6D state [p, v] to our 3D measurement [p]
        // H = [ I_3x3 , 0_3x3 ]
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

        // 3. Calculate the Innovation / Residual (y)
        // y = z - H * x (Difference between actual measurement and predicted measurement)
        Eigen::Vector3d y = z - H * x_;

        // 4. Calculate the Innovation Covariance (S)
        // S = H * P * H^T + R
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_;

        // 5. Calculate the Kalman Gain (K)
        // K = P * H^T * S^-1
        // (Using .inverse() is computationally safe here because S is only a 3x3 matrix)
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

        // 6. Update the State (x)
        // x = x + K * y
        x_ = x_ + K * y;

        // 7. Update the Covariance (P)
        // P = (I - K * H) * P
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
        P_ = (I - K * H) * P_;
        
        // Optional: Print to terminal to verify velocity tracking is working
        ROS_INFO_STREAM_THROTTLE(1.0, "Est. Relative Velocity: [" 
                                      << x_(3) << ", " << x_(4) << ", " << x_(5) << "]");
    }

    void publishState(ros::Time stamp) {
        nav_msgs::Odometry out_msg;
        out_msg.header.stamp = stamp;
        out_msg.header.frame_id = "chaser_drone/base_link"; // Relative to your drone

        // Pack the filtered state into the message
        out_msg.pose.pose.position.x = x_(0);
        out_msg.pose.pose.position.y = x_(1);
        out_msg.pose.pose.position.z = x_(2);
        out_msg.twist.twist.linear.x = x_(3);
        out_msg.twist.twist.linear.y = x_(4);
        out_msg.twist.twist.linear.z = x_(5);

        filtered_state_pub_.publish(out_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "relative_eskf_node");
    ros::NodeHandle nh;
    RelativeESKF eskf(nh);
    ros::spin();
    return 0;
}