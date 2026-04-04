#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <Eigen/Dense>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/exceptions.h>
#include <memory>

class RelativeEKF {
private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber yolo_sub_;
    ros::Publisher filtered_state_pub_;
    ros::Publisher filtered_point_pub_;

    // --- The Core Math Variables ---
    Eigen::VectorXd x_; // State: [px_rel, py_rel, pz_rel, vx_tgt, vy_tgt, vz_tgt]^T
    Eigen::MatrixXd P_; // Covariance Matrix (6x6)
    Eigen::MatrixXd Q_; // Process Noise (6x6)
    Eigen::MatrixXd R_; // Measurement Noise (3x3)

    // Time tracking for dt calculations
    ros::Time last_odom_time_;
    bool is_initialized_;
    // TF2 buffer & listener for frame transforms
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
public:
    RelativeEKF(ros::NodeHandle& nh) : nh_(nh), is_initialized_(false), tf_buffer_(ros::Duration(10.0)) {
        // 1. Initialize Matrices
        x_ = Eigen::VectorXd::Zero(6);
        
        // --- INITIAL COVARIANCE (P) ---
        // We know roughly where the target is, but have ZERO idea how fast it's moving initially.
        // A massive velocity covariance forces the filter to aggressively learn the velocity in the first few frames.
        P_ = Eigen::MatrixXd::Zero(6, 6);
        P_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 5.0;   // Position uncertainty
        P_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 100.0; // Velocity uncertainty (Huge)
        
        // --- PROCESS NOISE (Q) - The "Flexibility" Knob ---
        // Position kinematics are strictly tied to velocity, but velocity is subject to unknown accelerations.
        Q_ = Eigen::MatrixXd::Zero(6, 6);
        Q_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 0.05;  // Low trust in random position teleports
        Q_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 10.0;  // High trust in rapid velocity changes
        
        // --- MEASUREMENT NOISE (R) - The "YOLO Confidence" Knob ---
        // Lowering this slightly from 0.5 forces the EKF to trust the incoming YOLO depth data more,
        // which sharpens the velocity response.
        R_ = Eigen::MatrixXd::Identity(3, 3) * 0.2;

        // 2. Setup ROS Plumbing
        // Subscribing to noisy_odometry
        odom_sub_ = nh_.subscribe("noisy_odometry", 1, &RelativeEKF::predictionCallback, this);

        // Sub to gt_odometry
        // odom_sub_ = nh_.subscribe("/chaser_drone/odometry_sensor1/odometry", 1, &RelativeEKF::predictionCallback, this); //gt

    // Subscribe to YOLO depth estimator output
    yolo_sub_ = nh_.subscribe("/target_position_relative", 1, &RelativeEKF::updateCallback, this);

    // TF2 listener (long-lived). This will pick up the static transform
    // published by your launch file (chaser_drone/base_link -> chaser_drone/camera_left
    // and camera_left -> camera_left_optical).
    tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));
        
        // The clean, drift-compensated output for the RL agent
        filtered_state_pub_ = nh_.advertise<nav_msgs::Odometry>("filtered_relative_state", 10);
        filtered_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("filtered_target_position", 10);

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
        
        publishState(current_time, msg);
    }

    // --- The Low-Speed Update Step (e.g., 30Hz from YOLO) ---
    void updateCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        // Transform the incoming detection in camera_left_optical frame
        // into the body frame (chaser_drone/base_link) using TF2. The launch
        // file publishes the static chain: base_link -> camera_left -> camera_left_optical.
        geometry_msgs::PointStamped pt_body;
        try {
            // Transform into the chaser body frame (use the message timestamp)
            tf_buffer_.transform(*msg, pt_body, "chaser_drone/base_link", ros::Duration(0.1));
        } catch (const tf2::TransformException &ex) {
            ROS_WARN_THROTTLE(1.0, "TF transform to body frame failed: %s", ex.what());
            return;
        }

        // Now pt_body.point is expressed in the body frame (which we treat as FLU: X-front, Y-left, Z-up)
        double flu_x = pt_body.point.x;
        double flu_y = pt_body.point.y;
        double flu_z = pt_body.point.z;

        if (!is_initialized_) {
            // Initialize using the transformed body-frame coordinates
            x_(0) = flu_x;
            x_(1) = flu_y;
            x_(2) = flu_z;
            last_odom_time_ = pt_body.header.stamp;
            is_initialized_ = true;
            ROS_INFO("EKF Initialized with detection transformed to body frame.");
            return;
        }

        // 1. Extract the mapped YOLO measurement vector
        Eigen::Vector3d z(flu_x, flu_y, flu_z);

        // 2. Define Observation Matrix (H)
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

        // 3. EKF Update Math (Unchanged)
        Eigen::Vector3d y = z - H * x_;
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
        x_ = x_ + K * y;
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
        P_ = (I - K * H) * P_;
    }

    void publishState(ros::Time stamp, const nav_msgs::Odometry::ConstPtr& drone_odom) {
        // frame transformation for : body frame -> 

        // 1. Extract the drone's current orientation quaternion
        Eigen::Quaterniond q_body(
            drone_odom->pose.pose.orientation.w,
            drone_odom->pose.pose.orientation.x,
            drone_odom->pose.pose.orientation.y,
            drone_odom->pose.pose.orientation.z
        );

        // 2. Find where the drone's "Forward" (X-axis) is pointing in the real world
        Eigen::Vector3d forward_body(1.0, 0.0, 0.0);
        Eigen::Vector3d forward_world = q_body * forward_body;

        // 3. Flatten this forward vector against the gravity plane (Z = 0)
        forward_world(2) = 0.0; 

        // Edge Case Guard: If the drone is pointing EXACTLY straight up or down, 
        // the flattened vector length is 0. We default to the previous valid heading or X-axis.
        if (forward_world.norm() < 1e-6) {
            forward_world = Eigen::Vector3d(1.0, 0.0, 0.0); 
        } else {
            forward_world.normalized();
        }

        // 4. Build a robust Gravity-Aligned Rotation Matrix using cross products
        // Z is straight up (Gravity), X is our flattened forward vector, Y is Left
        Eigen::Vector3d z_world(0.0, 0.0, 1.0);
        Eigen::Vector3d y_world = z_world.cross(forward_world).normalized();

        Eigen::Matrix3d R_world_to_gravity_aligned;
        R_world_to_gravity_aligned.row(0) = forward_world;
        R_world_to_gravity_aligned.row(1) = y_world;
        R_world_to_gravity_aligned.row(2) = z_world;

        // 5. The full transformation: Body -> World -> Gravity-Aligned
        // We rotate the Body estimates into the World frame, then align them to our new Yaw frame
        Eigen::Vector3d p_rel_body(x_(0), x_(1), x_(2));
        Eigen::Vector3d v_tgt_body(x_(3), x_(4), x_(5));

        Eigen::Vector3d p_rel_gravity = R_world_to_gravity_aligned * (q_body * p_rel_body);
        Eigen::Vector3d v_tgt_gravity = R_world_to_gravity_aligned * (q_body * v_tgt_body);

        // 6. Publish to the RL Agent in the OPTICAL Gravity-Aligned Frame
        nav_msgs::Odometry out_msg;
        out_msg.header.stamp = stamp;
        out_msg.header.frame_id = "gravity_aligned_optical_frame"; 
        
        // Map FLU (0=Front, 1=Left, 2=Up) back to Optical (X=Right, Y=Down, Z=Front)
        out_msg.pose.pose.position.x = -p_rel_gravity(1); // Right is Negative Left
        out_msg.pose.pose.position.y = -p_rel_gravity(2); // Down is Negative Up
        out_msg.pose.pose.position.z = p_rel_gravity(0);  // Front is Forward
        
        // Apply the exact same mapping to the velocity vector
        out_msg.twist.twist.linear.x = -v_tgt_gravity(1); 
        out_msg.twist.twist.linear.y = -v_tgt_gravity(2); 
        out_msg.twist.twist.linear.z = v_tgt_gravity(0);  

        filtered_state_pub_.publish(out_msg);

        // Publish filtered relative target position
        geometry_msgs::PointStamped point_msg;
        point_msg.header.stamp = stamp;
        
        point_msg.header.frame_id = "chaser_drone/base_link"; 
        
        point_msg.point.x = x_(0);
        point_msg.point.y = x_(1);
        point_msg.point.z = x_(2);
        
        filtered_point_pub_.publish(point_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "relative_ekf_node");
    ros::NodeHandle nh;
    RelativeEKF ekf(nh);
    ros::spin();
    return 0;
}