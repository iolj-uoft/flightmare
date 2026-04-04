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
#include <deque>

class RelativeEKF {
private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber yolo_sub_;
    ros::Publisher filtered_state_pub_;
    ros::Publisher filtered_point_pub_;

    // --- The Core Math Variables ---
    // State now strictly lives in the Gravity-Aligned (GA) Frame
    Eigen::VectorXd x_; // State: [px_ga, py_ga, pz_ga, vx_tgt_ga, vy_tgt_ga, vz_tgt_ga]^T
    Eigen::MatrixXd P_; // Covariance Matrix (6x6)
    Eigen::MatrixXd Q_; // Process Noise (6x6)
    Eigen::MatrixXd R_; // Measurement Noise (3x3)

    ros::Time last_odom_time_;
    bool is_initialized_;
    
    // Store the latest orientation to sync Odometry and YOLO callbacks
    geometry_msgs::Quaternion latest_q_;

    // TF2 buffer, listener, and broadcaster
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    struct HistoryElement {
        ros::Time stamp;
        Eigen::VectorXd x;
        Eigen::MatrixXd P;
        Eigen::Vector3d v_ga; // Linear velocity input
        Eigen::Vector3d w_ga; // Angular velocity input
        double dt;            // Time step used to arrive at this state
    };

    std::deque<HistoryElement> history_buffer_;
    const size_t MAX_HISTORY_SIZE = 100; // Stores up to 1 second of data at 100 Hz
    
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

        // Initialize latest_q_ to identity quaternion
        latest_q_.x = 0.0; latest_q_.y = 0.0; latest_q_.z = 0.0; latest_q_.w = 1.0;

        // 2. Setup ROS Plumbing
        odom_sub_ = nh_.subscribe("noisy_odometry", 1, &RelativeEKF::predictionCallback, this);
        yolo_sub_ = nh_.subscribe("/target_position_relative", 1, &RelativeEKF::updateCallback, this);

        tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));
        
        filtered_state_pub_ = nh_.advertise<nav_msgs::Odometry>("filtered_relative_state", 10);
        filtered_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("filtered_target_position", 10);

        ROS_INFO("Robust GA-Frame Relative EKF Initialized. Waiting for YOLO...");
    }

    Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
        Eigen::Matrix3d S;
        S <<  0.0,  -v(2),   v(1),
              v(2),  0.0,  -v(0),
             -v(1),  v(0),   0.0;
        return S;
    }

    // Helper: Calculates the matrix that removes Roll and Pitch, keeping only Yaw
    // Helper: Calculates the matrix that transforms a vector from Body to GA Frame
    Eigen::Matrix3d getDerotationMatrix(const geometry_msgs::Quaternion& msg_q) {
        // 1. Get the World-to-Body quaternion
        tf2::Quaternion q_world_to_base;
        tf2::fromMsg(msg_q, q_world_to_base);

        // 2. Extract Yaw
        double roll, pitch, yaw;
        tf2::Matrix3x3(q_world_to_base).getRPY(roll, pitch, yaw);

        // 3. Create the World-to-GA quaternion (Yaw only)
        tf2::Quaternion q_world_to_ga;
        q_world_to_ga.setRPY(0.0, 0.0, yaw);

        // 4. Calculate Body-to-GA rotation
        // Math: v_ga = q_world_to_ga_inverse * q_world_to_base * v_body
        Eigen::Quaterniond e_q_world_to_base(msg_q.w, msg_q.x, msg_q.y, msg_q.z);
        Eigen::Quaterniond e_q_world_to_ga(q_world_to_ga.w(), q_world_to_ga.x(), q_world_to_ga.y(), q_world_to_ga.z());
        
        Eigen::Quaterniond q_body_to_ga = e_q_world_to_ga.inverse() * e_q_world_to_base;
        
        return q_body_to_ga.toRotationMatrix();
    }

    Eigen::Matrix3d computeDynamicCovariance(const Eigen::Vector3d& z_body) {
        // Calculate Euclidean distance to the target
        double distance = z_body.norm();
        
        // Clamp the minimum distance to avoid absolute zero uncertainty 
        // and prevent the covariance matrix from becoming singular
        distance = std::max(distance, 0.5); 

        // Define scaling coefficients (Tune these based on your specific YOLO/Stereo accuracy)
        // e.g., depth_coeff = 0.05 means a 5cm standard deviation at 1 meter.
        double depth_coeff = 0.05; 
        double lat_coeff = 0.02;   

        // Stereo vision physics:
        // - Depth standard deviation scales quadratically (distance^2)
        // - Lateral/Vertical standard deviation scales linearly (distance)
        double std_dev_depth = depth_coeff * (distance * distance); 
        double std_dev_lat   = lat_coeff * distance;                

        Eigen::Matrix3d R_dynamic = Eigen::Matrix3d::Zero();
        
        // Variances are the square of the standard deviations
        // Assuming standard ROS REP-103: X is Forward (Depth), Y is Left, Z is Up
        R_dynamic(0, 0) = std_dev_depth * std_dev_depth; // X (Forward/Depth)
        R_dynamic(1, 1) = std_dev_lat * std_dev_lat;     // Y (Lateral)
        R_dynamic(2, 2) = std_dev_lat * std_dev_lat;     // Z (Vertical)
        
        // Apply a noise floor (minimum constant variance) to maintain stability
        Eigen::Matrix3d R_floor = Eigen::Matrix3d::Identity() * 0.01;
        
        return R_dynamic + R_floor;
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

    void propagateState(Eigen::VectorXd& x, Eigen::MatrixXd& P, 
                        const Eigen::Vector3d& v_ga, const Eigen::Vector3d& w_ga, double dt) {
        // 1. RK4 Integration (using your existing computeDynamics)
        Eigen::VectorXd k1 = computeDynamics(x, v_ga, w_ga);
        Eigen::VectorXd k2 = computeDynamics(x + 0.5 * dt * k1, v_ga, w_ga);
        Eigen::VectorXd k3 = computeDynamics(x + 0.5 * dt * k2, v_ga, w_ga);
        Eigen::VectorXd k4 = computeDynamics(x + dt * k3, v_ga, w_ga);

        x += (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

        // 2. Covariance Update
        Eigen::MatrixXd F = Eigen::MatrixXd::Zero(6, 6);
        Eigen::Matrix3d w_skew = skew(w_ga);
        F.block<3, 3>(0, 0) = -w_skew;
        F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
        F.block<3, 3>(3, 3) = -w_skew;

        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
        Eigen::MatrixXd Phi = I + F * dt + 0.5 * F * F * dt * dt;
        P = Phi * P * Phi.transpose() + Q_ * dt;
    }

    void predictionCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Store attitude immediately for the YOLO update callback to use
        latest_q_ = msg->pose.pose.orientation;

        if (!is_initialized_) return; 

        ros::Time current_time = msg->header.stamp;
        double dt = (current_time - last_odom_time_).toSec();
        
        if (dt <= 0.0 || dt > 0.1) {
            last_odom_time_ = current_time;
            return; 
        }

        // 1. Extract pure body velocities
        Eigen::Vector3d v_body(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
        Eigen::Vector3d w_body(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);

        // 2. Transform linear velocity into the GA Frame
        Eigen::Matrix3d R_derotate = getDerotationMatrix(latest_q_);
        Eigen::Vector3d v_ga = R_derotate * v_body;

        // 3. Extract the true global Yaw Rate for the GA frame kinematics
        tf2::Quaternion q;
        tf2::fromMsg(latest_q_, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        // Euler Kinematics: true yaw rate (psi_dot) from body rates
        double cos_pitch = cos(pitch);
        double yaw_rate = w_body(2); // Fallback to raw yaw rate if perfectly vertical
        if (std::abs(cos_pitch) > 1e-3) {
            yaw_rate = (w_body(1) * sin(roll) + w_body(2) * cos(roll)) / cos_pitch;
        }
        
        // GA Frame only rotates around Z
        Eigen::Vector3d w_ga(0.0, 0.0, yaw_rate); 

        // 1. Propagate the current state forward
        propagateState(x_, P_, v_ga, w_ga, dt);

        // 2. Save to history buffer
        HistoryElement hist;
        hist.stamp = current_time;
        hist.x = x_;
        hist.P = P_;
        hist.v_ga = v_ga;
        hist.w_ga = w_ga;
        hist.dt = dt;

        history_buffer_.push_back(hist);
        if (history_buffer_.size() > MAX_HISTORY_SIZE) {
            history_buffer_.pop_front();
        }

        last_odom_time_ = current_time;
        publishState(current_time, msg);
    }

    void updateCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        ros::Time yolo_time = msg->header.stamp;
        
        // 1. Transform to pure body frame
        // (tf_buffer_ automatically uses yolo_time to get the historical transform)
        geometry_msgs::PointStamped pt_body;
        try {
            tf_buffer_.transform(*msg, pt_body, "chaser_drone/base_link", ros::Duration(0.1));
        } catch (const tf2::TransformException &ex) {
            ROS_WARN_THROTTLE(1.0, "TF transform to body frame failed: %s", ex.what());
            return;
        }

        Eigen::Vector3d z_body(pt_body.point.x, pt_body.point.y, pt_body.point.z);

        Eigen::Matrix3d R_dynamic_cov = computeDynamicCovariance(z_body);

        // 2. Derotate measurement into GA Frame BEFORE the EKF sees it
        // (Uses the latest orientation. For a 50ms delay, this approximation is usually fine)
        Eigen::Matrix3d R_derotate = getDerotationMatrix(latest_q_);
        Eigen::Matrix3d R_noise_ga = R_derotate * R_dynamic_cov * R_derotate.transpose();
        Eigen::Vector3d z_ga = R_derotate * z_body;

        // --- Initialization Block ---
        if (!is_initialized_) {
            x_(0) = z_ga(0); x_(1) = z_ga(1); x_(2) = z_ga(2);
            last_odom_time_ = pt_body.header.stamp;
            is_initialized_ = true;
            ROS_INFO("EKF Initialized directly in the GA Frame.");
            return;
        }

        if (history_buffer_.empty()) return;

        // 3. REWIND: Find the state in history closest to yolo_time
        auto it = history_buffer_.begin();
        
        // Find the first element that is AT or just AFTER the yolo_time
        while (it != history_buffer_.end() && it->stamp < yolo_time) {
            ++it;
        }

        // If we didn't find a valid time (e.g., measurement is too old), abort update
        if (it == history_buffer_.end() || it == history_buffer_.begin()) {
            ROS_WARN_THROTTLE(1.0, "YOLO measurement too old or out of buffer range.");
            return; 
        }

        // Step back one to get the state just BEFORE the measurement
        --it; 

        Eigen::VectorXd x_delayed = it->x;
        Eigen::MatrixXd P_delayed = it->P;

        // 4. EKF UPDATE (Comparing past GA observation to past GA prediction)
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

        Eigen::Vector3d y = z_ga - H * x_delayed;
        Eigen::MatrixXd S = H * P_delayed * H.transpose() + R_noise_ga;
        Eigen::MatrixXd K = P_delayed * H.transpose() * S.inverse();
        
        x_delayed = x_delayed + K * y;
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
        P_delayed = (I - K * H) * P_delayed;

        // Save the updated state back to that exact point in history
        it->x = x_delayed;
        it->P = P_delayed;

        // 5. REPLAY: Fast-forward the state back to the present
        auto replay_it = it;
        auto next_it = it + 1;
        
        while (next_it != history_buffer_.end()) {
            // Re-propagate using the old inputs, starting from the newly corrected state
            propagateState(replay_it->x, replay_it->P, next_it->v_ga, next_it->w_ga, next_it->dt);
            
            // Overwrite the next state in the buffer with the newly re-integrated state
            next_it->x = replay_it->x;
            next_it->P = replay_it->P;
            
            replay_it++;
            next_it++;
        }

        // 6. FINALIZE: Update the current active state to the end of the replayed buffer
        x_ = history_buffer_.back().x;
        P_ = history_buffer_.back().P;
    }

    void publishState(ros::Time stamp, const nav_msgs::Odometry::ConstPtr& drone_odom) {
        // --- 1. Broadcast TF: base_link -> ga_body_frame ---
        // (We still need to broadcast this so RViz knows how to draw the GA frame)
        tf2::Quaternion q_world_to_base;
        tf2::fromMsg(drone_odom->pose.pose.orientation, q_world_to_base);
        
        double roll, pitch, yaw;
        tf2::Matrix3x3(q_world_to_base).getRPY(roll, pitch, yaw);

        tf2::Quaternion q_world_to_ga;
        q_world_to_ga.setRPY(0.0, 0.0, yaw);

        tf2::Quaternion q_base_to_ga = q_world_to_base.inverse() * q_world_to_ga;
        q_base_to_ga.normalize();

        geometry_msgs::TransformStamped t_msg;
        t_msg.header.stamp = stamp;
        t_msg.header.frame_id = "chaser_drone/base_link"; 
        t_msg.child_frame_id = "ga_body_frame";            
        t_msg.transform.translation.x = 0.0;
        t_msg.transform.translation.y = 0.0;
        t_msg.transform.translation.z = 0.0;
        t_msg.transform.rotation = tf2::toMsg(q_base_to_ga);
        tf_broadcaster_.sendTransform(t_msg);

        // --- 2. Publish Clean GA Observation to RL ---
        // Because x_ is already calculated in the GA frame, no math is needed here!
        nav_msgs::Odometry out_msg;
        out_msg.header.stamp = stamp;
        out_msg.header.frame_id = "ga_body_frame"; 
        
        out_msg.pose.pose.position.x = x_(0);
        out_msg.pose.pose.position.y = x_(1);
        out_msg.pose.pose.position.z = x_(2);
        
        out_msg.twist.twist.linear.x = x_(3); 
        out_msg.twist.twist.linear.y = x_(4); 
        out_msg.twist.twist.linear.z = x_(5);  

        filtered_state_pub_.publish(out_msg);

        // --- 3. Publish RViz Point ---
        geometry_msgs::PointStamped point_msg;
        point_msg.header.stamp = stamp;
        point_msg.header.frame_id = "ga_body_frame";
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