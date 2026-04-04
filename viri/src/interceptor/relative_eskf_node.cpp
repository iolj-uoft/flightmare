#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
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

    ros::Time last_odom_time_;
    bool is_initialized_;

    // TF2 buffer, listener, and broadcaster
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
public:
    RelativeEKF(ros::NodeHandle& nh) : nh_(nh), is_initialized_(false), tf_buffer_(ros::Duration(10.0)) {
        // 1. Initialize Matrices
        x_ = Eigen::VectorXd::Zero(6);
        
        P_ = Eigen::MatrixXd::Zero(6, 6);
        P_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 5.0;   
        P_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 100.0; 
        
        Q_ = Eigen::MatrixXd::Zero(6, 6);
        Q_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 0.05;  
        Q_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 10.0;  
        
        R_ = Eigen::MatrixXd::Identity(3, 3) * 0.2;

        // 2. Setup ROS Plumbing
        odom_sub_ = nh_.subscribe("noisy_odometry", 1, &RelativeEKF::predictionCallback, this);
        yolo_sub_ = nh_.subscribe("/target_position_relative", 1, &RelativeEKF::updateCallback, this);

        tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));
        
        filtered_state_pub_ = nh_.advertise<nav_msgs::Odometry>("filtered_relative_state", 10);
        filtered_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("filtered_target_position", 10);

        ROS_INFO("Relative EKF Node (RK4 + GA Frame TF) Initialized. Waiting for YOLO...");
    }

    Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
        Eigen::Matrix3d S;
        S <<  0.0,  -v(2),   v(1),
              v(2),  0.0,  -v(0),
             -v(1),  v(0),   0.0;
        return S;
    }

    Eigen::VectorXd computeDynamics(const Eigen::VectorXd& state, 
                                    const Eigen::Vector3d& v_drone, 
                                    const Eigen::Vector3d& w_drone) {
        Eigen::VectorXd state_dot = Eigen::VectorXd::Zero(6);
        Eigen::Vector3d p_rel = state.head(3);
        Eigen::Vector3d v_tgt = state.tail(3);

        state_dot.head(3) = v_tgt - v_drone - w_drone.cross(p_rel);
        state_dot.tail(3) = -w_drone.cross(v_tgt);
        
        return state_dot;
    }

    void predictionCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        if (!is_initialized_) return; 

        ros::Time current_time = msg->header.stamp;
        double dt = (current_time - last_odom_time_).toSec();
        
        if (dt <= 0.0 || dt > 0.1) {
            last_odom_time_ = current_time;
            return; 
        }

        Eigen::Vector3d v_drone(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
        Eigen::Vector3d w_drone(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);

        Eigen::VectorXd k1 = computeDynamics(x_, v_drone, w_drone);
        Eigen::VectorXd k2 = computeDynamics(x_ + 0.5 * dt * k1, v_drone, w_drone);
        Eigen::VectorXd k3 = computeDynamics(x_ + 0.5 * dt * k2, v_drone, w_drone);
        Eigen::VectorXd k4 = computeDynamics(x_ + dt * k3, v_drone, w_drone);

        x_ += (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

        Eigen::MatrixXd F = Eigen::MatrixXd::Zero(6, 6);
        Eigen::Matrix3d w_skew = skew(w_drone);
        F.block<3, 3>(0, 0) = -w_skew;
        F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
        F.block<3, 3>(3, 3) = -w_skew;

        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
        Eigen::MatrixXd Phi = I + F * dt + 0.5 * F * F * dt * dt;
        P_ = Phi * P_ * Phi.transpose() + Q_ * dt;

        last_odom_time_ = current_time;
        publishState(current_time, msg);
    }

    void updateCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        geometry_msgs::PointStamped pt_body;
        try {
            tf_buffer_.transform(*msg, pt_body, "chaser_drone/base_link", ros::Duration(0.1));
        } catch (const tf2::TransformException &ex) {
            ROS_WARN_THROTTLE(1.0, "TF transform to body frame failed: %s", ex.what());
            return;
        }

        double flu_x = pt_body.point.x;
        double flu_y = pt_body.point.y;
        double flu_z = pt_body.point.z;

        if (!is_initialized_) {
            x_(0) = flu_x; x_(1) = flu_y; x_(2) = flu_z;
            last_odom_time_ = pt_body.header.stamp;
            is_initialized_ = true;
            ROS_INFO("EKF Initialized with detection transformed to body frame.");
            return;
        }

        Eigen::Vector3d z(flu_x, flu_y, flu_z);
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

        Eigen::Vector3d y = z - H * x_;
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
        x_ = x_ + K * y;
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
        P_ = (I - K * H) * P_;
    }

    void publishState(ros::Time stamp, const nav_msgs::Odometry::ConstPtr& drone_odom) {
        // --- 1. Extract RPY from the drone's current orientation ---
        tf2::Quaternion q_world_to_base;
        tf2::fromMsg(drone_odom->pose.pose.orientation, q_world_to_base);
        
        double roll, pitch, yaw;
        tf2::Matrix3x3(q_world_to_base).getRPY(roll, pitch, yaw);

        // --- 2. Construct Gravity-Aligned (GA) Frame Quaternion ---
        // The GA frame has the exact same Yaw as the drone, but 0 Roll and 0 Pitch.
        tf2::Quaternion q_world_to_ga;
        q_world_to_ga.setRPY(0.0, 0.0, yaw);

        // --- 3. Broadcast TF: base_link -> ga_body_frame ---
        // To find the rotation describing GA frame relative to Base Link:
        // q_base_to_ga = q_world_to_base_inverse * q_world_to_ga
        tf2::Quaternion q_base_to_ga = q_world_to_base.inverse() * q_world_to_ga;
        q_base_to_ga.normalize();

        geometry_msgs::TransformStamped t_msg;
        t_msg.header.stamp = stamp;
        t_msg.header.frame_id = "chaser_drone/base_link"; // Parent
        t_msg.child_frame_id = "ga_body_frame";            // Child
        t_msg.transform.translation.x = 0.0;
        t_msg.transform.translation.y = 0.0;
        t_msg.transform.translation.z = 0.0;
        t_msg.transform.rotation = tf2::toMsg(q_base_to_ga);
        
        tf_broadcaster_.sendTransform(t_msg);

        // --- 4. Transform Target State for RL Agent ---
        // We must map our internal EKF state (pure body frame) into the GA frame.
        // Math: p_GA = R_world_to_GA_inverse * R_world_to_base * p_body
        Eigen::Quaterniond e_q_world_to_base(
            drone_odom->pose.pose.orientation.w,
            drone_odom->pose.pose.orientation.x,
            drone_odom->pose.pose.orientation.y,
            drone_odom->pose.pose.orientation.z
        );
        Eigen::Quaterniond e_q_world_to_ga(
            q_world_to_ga.w(), q_world_to_ga.x(), q_world_to_ga.y(), q_world_to_ga.z()
        );

        Eigen::Vector3d p_rel_body(x_(0), x_(1), x_(2));
        Eigen::Vector3d v_tgt_body(x_(3), x_(4), x_(5));

        Eigen::Vector3d p_rel_ga = e_q_world_to_ga.inverse() * (e_q_world_to_base * p_rel_body);
        Eigen::Vector3d v_tgt_ga = e_q_world_to_ga.inverse() * (e_q_world_to_base * v_tgt_body);

        // --- 5. Publish Clean GA Observation to RL ---
        nav_msgs::Odometry out_msg;
        out_msg.header.stamp = stamp;
        out_msg.header.frame_id = "ga_body_frame"; 
        
        // Output in standard FLU (Front, Left, Up) relative to the horizon
        out_msg.pose.pose.position.x = p_rel_ga(0);
        out_msg.pose.pose.position.y = p_rel_ga(1);
        out_msg.pose.pose.position.z = p_rel_ga(2);
        
        out_msg.twist.twist.linear.x = v_tgt_ga(0); 
        out_msg.twist.twist.linear.y = v_tgt_ga(1); 
        out_msg.twist.twist.linear.z = v_tgt_ga(2);  

        filtered_state_pub_.publish(out_msg);

        // Publish pure body point for visualization (RViz)
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