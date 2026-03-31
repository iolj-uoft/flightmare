#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <Eigen/Dense>

class RelativeEKF {
private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber yolo_sub_;
    ros::Publisher filtered_state_pub_;

    // --- The Core Math Variables ---
    Eigen::VectorXd x_; // State: [px_rel, py_rel, pz_rel, vx_tgt, vy_tgt, vz_tgt]^T
    Eigen::MatrixXd P_; // Covariance Matrix (6x6)
    Eigen::MatrixXd Q_; // Process Noise (6x6)
    Eigen::MatrixXd R_; // Measurement Noise (3x3)

    // Time tracking for dt calculations
    ros::Time last_odom_time_;
    bool is_initialized_;
    
public:
    RelativeEKF(ros::NodeHandle& nh) : nh_(nh), is_initialized_(false) {
        // 1. Initialize Matrices
        x_ = Eigen::VectorXd::Zero(6);
        
        // P starts large because we don't know where the target is initially
        P_ = Eigen::MatrixXd::Identity(6, 6) * 10.0; 
        
        // Q represents the target's random-walk acceleration uncertainty
        Q_ = Eigen::MatrixXd::Identity(6, 6) * 0.1; 
        
        // R represents the YOLO stereo depth uncertainty
        R_ = Eigen::MatrixXd::Identity(3, 3) * 0.5;

        // 2. Setup ROS Plumbing
        // Subscribing to noisy_odometry
    odom_sub_ = nh_.subscribe("noisy_odometry", 1, &RelativeEKF::predictionCallback, this);
        
        // Subscribe to YOLO depth estimator output
    yolo_sub_ = nh_.subscribe("/target_position_relative", 1, &RelativeEKF::updateCallback, this);
        
        // The clean, drift-compensated output for the RL agent
        filtered_state_pub_ = nh_.advertise<nav_msgs::Odometry>("filtered_relative_state", 10);

    ROS_INFO("Relative EKF Node (RK4 + Odometry) Initialized. Waiting for YOLO...");
    }

    // Helper function: Converts a 3D vector into a 3x3 skew-symmetric matrix
    Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
        Eigen::Matrix3d S;
        S <<  0.0,  -v(2),   v(1),
              v(2),  0.0,  -v(0),
             -v(1),  v(0),   0.0;
        return S;
    }

    // Compute the nonlinear kinematic derivatives f(x, u) using drone velocity
    Eigen::VectorXd computeDynamics(const Eigen::VectorXd& state, 
                                    const Eigen::Vector3d& v_drone, 
                                    const Eigen::Vector3d& w_drone) {
        Eigen::VectorXd state_dot = Eigen::VectorXd::Zero(6);
        Eigen::Vector3d p_rel = state.head(3);
        Eigen::Vector3d v_tgt = state.tail(3); // Target velocity in body frame

        // Kinematics in the rotating body frame
        // Assuming target acceleration (a_tgt) is zero-mean random noise handled by Q_ matrix
        state_dot.head(3) = v_tgt - v_drone - w_drone.cross(p_rel);
        state_dot.tail(3) = -w_drone.cross(v_tgt);
        
        return state_dot;
    }

    // --- The High-Speed Prediction Step (e.g., 200Hz from Odometry) ---
    void predictionCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        if (!is_initialized_) return; // Wait for the first YOLO point to anchor the filter

        ros::Time current_time = msg->header.stamp;
        double dt = (current_time - last_odom_time_).toSec();
        
        // Safety guard for simulation time jumps
        if (dt <= 0.0 || dt > 0.1) {
            last_odom_time_ = current_time;
            return; 
        }

        // 1. Extract drone body rates and linear velocities from odometry
        Eigen::Vector3d v_drone(msg->twist.twist.linear.x, 
                                msg->twist.twist.linear.y, 
                                msg->twist.twist.linear.z);
                                
        Eigen::Vector3d w_drone(msg->twist.twist.angular.x, 
                                msg->twist.twist.angular.y, 
                                msg->twist.twist.angular.z);

        // 2. State Update: 4th-Order Runge-Kutta (RK4) Integration
        Eigen::VectorXd k1 = computeDynamics(x_, v_drone, w_drone);
        Eigen::VectorXd k2 = computeDynamics(x_ + 0.5 * dt * k1, v_drone, w_drone);
        Eigen::VectorXd k3 = computeDynamics(x_ + 0.5 * dt * k2, v_drone, w_drone);
        Eigen::VectorXd k4 = computeDynamics(x_ + dt * k3, v_drone, w_drone);

        x_ += (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

        // 3. Jacobian and Covariance Update
        Eigen::MatrixXd F = Eigen::MatrixXd::Zero(6, 6);
        Eigen::Matrix3d w_skew = skew(w_drone);
        
        F.block<3, 3>(0, 0) = -w_skew;
        F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
        F.block<3, 3>(3, 3) = -w_skew;

        // Aerospace-grade discretization: 2nd-Order Taylor Expansion
        // Phi = I + F*dt + (1/2)*F^2*dt^2
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
        Eigen::MatrixXd Phi = I + F * dt + 0.5 * F * F * dt * dt;

        // Update Covariance Matrix
        P_ = Phi * P_ * Phi.transpose() + Q_ * dt;

        last_odom_time_ = current_time;
        
        publishState(current_time);
    }

    // --- The Low-Speed Update Step (e.g., 30Hz from YOLO) ---
    void updateCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        if (!is_initialized_) {
            // First time seeing the target: Initialize position, assume zero relative velocity
            x_(0) = msg->point.x;
            x_(1) = msg->point.y;
            x_(2) = msg->point.z;
            last_odom_time_ = msg->header.stamp;
            is_initialized_ = true;
            ROS_INFO("EKF Initialized with first YOLO measurement.");
            return;
        }

        // 1. Extract the YOLO measurement vector (z)
        Eigen::Vector3d z(msg->point.x, msg->point.y, msg->point.z);

        // 2. Define the Observation Matrix (H)
        // H maps our 6D state [p_rel, v_tgt] to our 3D measurement [p_rel]
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

        // 3. Calculate the Innovation / Residual (y)
        Eigen::Vector3d y = z - H * x_;

        // 4. Calculate the Innovation Covariance (S)
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_;

        // 5. Calculate the Kalman Gain (K)
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

        // 6. Update the State (x)
        x_ = x_ + K * y;

        // 7. Update the Covariance (P)
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
        P_ = (I - K * H) * P_;
        
        // Optional: Print to terminal to verify velocity tracking is working
        ROS_INFO_STREAM_THROTTLE(1.0, "Est. Target Velocity: [" 
                                      << x_(3) << ", " << x_(4) << ", " << x_(5) << "]");
    }

    void publishState(ros::Time stamp) {
        nav_msgs::Odometry out_msg;
        out_msg.header.stamp = stamp;
        out_msg.header.frame_id = "chaser_drone/base_link"; // Relative to your drone's body frame

        // Pack the filtered state into the message
        // These are the values your RL agent should observe!
        out_msg.pose.pose.position.x = x_(0);
        out_msg.pose.pose.position.y = x_(1);
        out_msg.pose.pose.position.z = x_(2);
        
        // Note: The velocity output here is the estimated TARGET velocity (v_tgt)
        // in the drone's body frame. 
        out_msg.twist.twist.linear.x = x_(3);
        out_msg.twist.twist.linear.y = x_(4);
        out_msg.twist.twist.linear.z = x_(5);

        filtered_state_pub_.publish(out_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "relative_ekf_node");
    ros::NodeHandle nh;
    RelativeEKF ekf(nh);
    ros::spin();
    return 0;
}