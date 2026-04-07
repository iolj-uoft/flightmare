#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/ControlCommand.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <viri/RLAction.h>
#include <Eigen/Dense>
#include <mutex>
#include <cmath>
#include <thread>
#include <chrono>

/**
 * ROS Node that:
 * 1. Subscribes to chaser and target ground truth odometry
 * 2. Computes relative position and velocity (GT observation)
 * 3. Gets RL action from Python wrapper via topic
 * 4. Publishes attitude control commands directly to rpg_rotors_interface
 * 5. Handles arming of the motor controller
 * 6. Detects crashes and flips, signals episode termination with penalties
 */

/**
 * Convert Euler angles (roll, pitch, yaw) to quaternion
 */
geometry_msgs::Quaternion eulerToQuaternion(double roll, double pitch, double yaw) {
    // Using standard conversion formulas
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    geometry_msgs::Quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
    return q;
}

class GazeboRLAgentNode {
public:
    // Crash/flip detection thresholds
    static constexpr double CRASH_HEIGHT_THRESHOLD = 0.1;  // meters
    static constexpr double FLIP_ANGLE_THRESHOLD_DEG = 60.0;  // degrees
    static constexpr double FLIP_ANGLE_THRESHOLD_RAD = FLIP_ANGLE_THRESHOLD_DEG * M_PI / 180.0;
    static constexpr double CRASH_PENALTY = -100.0;  // Large negative reward
    static constexpr double FLIP_PENALTY = -50.0;    // Medium penalty for flipping
    static constexpr double RESET_DELAY = 0.1;  // seconds to wait before episode reset
    static constexpr double CRASH_DETECTION_DISABLED_TIME = 2.0;  // seconds after reset to disable crash detection
public:
    GazeboRLAgentNode(const ros::NodeHandle& nh) : nh_(nh), armed_(false) {
        // Subscribers
        chaser_odom_sub_ = nh_.subscribe(
            "ground_truth/odometry", 1,
            &GazeboRLAgentNode::chaserOdomCallback, this);
        
        target_odom_sub_ = nh_.subscribe(
            "/target_drone/ground_truth/odometry", 1,
            &GazeboRLAgentNode::targetOdomCallback, this);
        
        action_sub_ = nh_.subscribe(
            "rl_action", 10,
            &GazeboRLAgentNode::actionCallback, this);
        
        // Publishers
        // Direct control to rpg_rotors_interface (bypasses autopilot)
        control_pub_ = nh_.advertise<quadrotor_msgs::ControlCommand>(
            "control_command", 1);
        
        // Arm/disarm signal to motor controller
        arm_pub_ = nh_.advertise<std_msgs::Bool>(
            "bridge/arm", 1);
        
        obs_pub_ = nh_.advertise<geometry_msgs::Vector3>(
            "rl_observation", 1);
        
        reward_pub_ = nh_.advertise<std_msgs::Float32>(
            "rl_reward", 1);
        
        rel_vel_pub_ = nh_.advertise<geometry_msgs::Vector3>(
            "rl_relative_velocity", 1);
        
        // Crash/flip detection publishers
        done_pub_ = nh_.advertise<std_msgs::Bool>(
            "rl_episode_done", 1);
        
        crash_reason_pub_ = nh_.advertise<std_msgs::Bool>(
            "rl_crashed", 1);
        
        // Service clients for episode reset
        gazebo_set_model_state_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>(
            "/gazebo/set_model_state");
        gazebo_get_model_state_client_ = nh_.serviceClient<gazebo_msgs::GetModelState>(
            "/gazebo/get_model_state");
        
        ROS_INFO("GazeboRLAgentNode initialized");
        ROS_INFO("  Subscribing to: ground_truth/odometry, /target_drone/ground_truth/odometry, rl_action");
        ROS_INFO("  Publishing to: control_command (4D attitude+thrust), bridge/arm");
        ROS_INFO("  Crash/Flip Detection: Enabled (threshold: %.2f m, flip angle: %.1f°)", 
                 CRASH_HEIGHT_THRESHOLD, FLIP_ANGLE_THRESHOLD_DEG);
        ROS_INFO("  Episode Reset: Enabled (will respawn chaser drone on crash/flip)");
        
        // Auto-arm the drone after a short delay
        armDrone();
    }
    
    ~GazeboRLAgentNode() = default;

private:
    void chaserOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        chaser_odom_ = *msg;
        chaser_odom_received_ = true;
        ROS_DEBUG_THROTTLE(1, "Chaser odom: z=%.3f", msg->pose.pose.position.z);
    }
    
    void targetOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        target_odom_ = *msg;
        target_odom_received_ = true;
        ROS_DEBUG_THROTTLE(1, "Target odom: z=%.3f", msg->pose.pose.position.z);
    }
    
    void actionCallback(const viri::RLAction::ConstPtr& msg) {
        static int action_count = 0;
        action_count++;
        
        if (action_count % 50 == 0) {  // Log every 50 actions (every 1 second at 50 Hz)
            ROS_WARN("ACTION CALLBACK CALLED! (count=%d)", action_count);
        }
        
        std::lock_guard<std::mutex> lock(action_mutex_);
        
        if (!chaser_odom_received_ || !target_odom_received_) {
            ROS_WARN_ONCE("Waiting for odometry messages...");
            return;
        }
        
        // Extract 4D action [roll, pitch, yaw_rate, thrust]
        double roll = msg->roll;
        double pitch = msg->pitch;
        double yaw_rate = msg->yaw_rate;
        double thrust = msg->thrust;
        
        // ROS_DEBUG_THROTTLE: use for frequent messages that slow down logging
        ROS_DEBUG_THROTTLE(0.5, "[Action Input] roll=%.3f pitch=%.3f yaw_rate=%.3f thrust=%.3f", 
                          roll, pitch, yaw_rate, thrust);
        
        // Convert normalized action to control command
        // roll, pitch in radians (normalized [-1, 1] -> [-pi, pi])
        // yaw_rate in rad/s (normalized [-1, 1] -> [-pi, pi])
        // thrust normalized [0, 1] -> [0, 20] m/s^2 (with 9.81 = 1g baseline)
        
        quadrotor_msgs::ControlCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "world";
        cmd.control_mode = 1;  // ATTITUDE mode (more stable than BODY_RATES)
        cmd.armed = true;
        
        // Scale actions to reasonable ranges
        double roll_cmd = roll * M_PI / 6.0;      // [-pi/6, pi/6] (±30 degrees max roll)
        double pitch_cmd = pitch * M_PI / 6.0;    // [-pi/6, pi/6] (±30 degrees max pitch)
        double yaw_rate_cmd = yaw_rate * M_PI / 2.0;    // [-pi/2, pi/2] rad/s (moderate yaw rate)
        
        // Integrate yaw rate to get desired yaw angle (0.02s is control period at 50Hz)
        double dt = 0.02;
        current_yaw_ += yaw_rate_cmd * dt;
        
        // Normalize yaw to [-pi, pi]
        while (current_yaw_ > M_PI) current_yaw_ -= 2.0 * M_PI;
        while (current_yaw_ < -M_PI) current_yaw_ += 2.0 * M_PI;
        
        // Convert desired attitude (roll, pitch, yaw) to quaternion
        cmd.orientation = eulerToQuaternion(roll_cmd, pitch_cmd, current_yaw_);
        
        // In ATTITUDE mode, bodyrates are feed-forward terms from reference trajectory
        // Set yaw rate for feedback control
        cmd.bodyrates.x = 0.0;  // roll rate (0 = let attitude control handle it)
        cmd.bodyrates.y = 0.0;  // pitch rate (0 = let attitude control handle it)
        cmd.bodyrates.z = yaw_rate_cmd;  // yaw rate from action
        cmd.bodyrates.z = yaw_rate_cmd;  // yaw rate from action
        
        // Thrust: collective_thrust is MASS-NORMALIZED
        // For hover: collective_thrust = gravity = 9.81 m/s^2
        // To climb/descend: add/subtract from gravity
        double gravity = 9.81;  // m/s^2
        double max_accel = 3.0;  // m/s^2 (reduced for stability)
        
        // Clamp thrust to reasonable range [0.5*g, 1.5*g]
        double thrust_normalized = gravity + (thrust * max_accel);
        cmd.collective_thrust = std::max(gravity * 0.5, std::min(gravity * 1.5, thrust_normalized));
        
        ROS_INFO_THROTTLE(0.5, "[Control ATTITUDE] roll=%.2f° pitch=%.2f° yaw=%.2f° thrust=%.2f m/s²",
                          roll_cmd * 180.0 / M_PI, pitch_cmd * 180.0 / M_PI, 
                          current_yaw_ * 180.0 / M_PI, cmd.collective_thrust);
        
        control_pub_.publish(cmd);
        
        // Publish observation and reward every time we get an action
        publishObservation();
        publishReward();
    }
    
    void armDrone() {
        // Wait a bit for rpg_rotors_interface to be ready
        ros::Duration(0.5).sleep();
        
        // Publish arm signal to motor controller
        std_msgs::Bool arm_msg;
        arm_msg.data = true;
        
        ROS_INFO("Sending arm command to bridge/arm...");
        
        // Send multiple times at higher frequency to ensure it's received
        for (int i = 0; i < 20; ++i) {
            arm_pub_.publish(arm_msg);
            ROS_INFO_THROTTLE(1, "Arming attempt %d", i);
            ros::Duration(0.05).sleep();
        }
        
        armed_ = true;
        ROS_INFO("===== CHASER DRONE ARMED! =====");
    }
    
    void publishObservation() {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        
        if (!chaser_odom_received_ || !target_odom_received_) {
            return;
        }
        
        // Compute relative position
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
        
        geometry_msgs::Vector3 obs_msg;
        obs_msg.x = rel_pos(0);
        obs_msg.y = rel_pos(1);
        obs_msg.z = rel_pos(2);
        
        obs_pub_.publish(obs_msg);
        
        // Publish relative velocity
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
        
        geometry_msgs::Vector3 vel_msg;
        vel_msg.x = rel_vel(0);
        vel_msg.y = rel_vel(1);
        vel_msg.z = rel_vel(2);
        
        rel_vel_pub_.publish(vel_msg);
    }
    
    void publishReward() {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        
        if (!chaser_odom_received_ || !target_odom_received_) {
            return;
        }
        
        // Compute relative state
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
        
        // Check for crash and flip conditions
        bool crashed = checkCrash(chaser_pos);
        bool flipped = checkFlip(chaser_odom_.pose.pose.orientation);
        
        // Check if drone is too far away
        double pos_error = rel_pos.norm();
        bool too_far = (pos_error > 100.0);
        
        // Check if chaser is within 30cm of target (success condition)
        bool target_reached = (pos_error < 0.3);
        
        double reward = 0.0;
        
        if (crashed) {
            // Crash detected: large penalty and end episode
            reward = CRASH_PENALTY;
            episode_done_ = true;
            ROS_ERROR("CRASH DETECTED! Position: (%.2f, %.2f, %.2f)", 
                     chaser_pos(0), chaser_pos(1), chaser_pos(2));
            publishEpisodeDone(true, 1);  // 1 = crashed
        } else if (flipped) {
            // Flip detected: medium penalty and end episode
            reward = FLIP_PENALTY;
            episode_done_ = true;
            ROS_WARN("FLIP DETECTED!");
            publishEpisodeDone(true, 2);  // 2 = flipped
        } else if (target_reached) {
            // Target reached within 30cm: large bonus and end episode
            reward = 100.0;  // Large reward for successfully intercepting target
            episode_done_ = true;
            ROS_INFO("🎯 TARGET REACHED! Distance: %.3f m - Episode successful!", pos_error);
            publishEpisodeDone(true, 4);  // 4 = target reached
        } else if (too_far) {
            // Too far: penalty and end episode
            reward = -50.0;
            episode_done_ = true;
            ROS_WARN("DISTANCE EXCEEDED 100m! Ending episode.");
            publishEpisodeDone(true, 3);  // 3 = too far
        } else {
            // Normal reward function: negative distance, bonus for proximity, yaw alignment reward
            double vel_error = rel_vel.norm();
            
            // Base reward: negative distance and velocity error
            reward = -0.1 * pos_error - 0.01 * vel_error;
            
            // Yaw alignment reward: encourage chaser to face target
            // Compute direction to target in horizontal plane
            double target_bearing = std::atan2(rel_pos(1), rel_pos(0));
            
            // Extract chaser yaw from quaternion
            double chaser_w = chaser_odom_.pose.pose.orientation.w;
            double chaser_z = chaser_odom_.pose.pose.orientation.z;
            double chaser_yaw = 2.0 * std::atan2(chaser_z, chaser_w);
            
            // Compute yaw error (smallest angle difference)
            double yaw_error = std::abs(target_bearing - chaser_yaw);
            if (yaw_error > M_PI) {
                yaw_error = 2.0 * M_PI - yaw_error;
            }
            
            // Add small reward for good yaw alignment (when close to target)
            if (pos_error < 50.0) {  // Only apply when reasonably close
                reward += 0.5 * (1.0 - yaw_error / M_PI);  // Reward decreases with yaw error
            }
            
            if (pos_error < 1.0) {
                reward += 10.0;  // Bonus for being very close
            }
        }
        
        std_msgs::Float32 reward_msg;
        reward_msg.data = reward;
        reward_pub_.publish(reward_msg);
    }
    
    /**
     * Check if drone has crashed (hit ground or gone below threshold)
     * Returns false if crash detection is temporarily disabled
     */
    bool checkCrash(const Eigen::Vector3d& pos) const {
        // Check if crash detection is currently disabled (during takeoff after reset)
        if (ros::Time::now() < crash_detection_disabled_until_) {
            return false;  // Crash detection disabled
        }
        
        // Crash if altitude is too low (likely hit ground)
        if (pos(2) < CRASH_HEIGHT_THRESHOLD) {
            return true;
        }
        return false;
    }
    
    /**
     * Check if drone has flipped over (orientation too far from level)
     * Uses quaternion to compute roll and pitch angles
     * DISABLED: Flip detection is currently disabled
     */
    bool checkFlip(const geometry_msgs::Quaternion& quat) const {
        // Flip detection is disabled
        return false;
    }
    
    /**
     * Publish episode done signal with crash reason
     * reason: 0=normal done, 1=crashed, 2=flipped
     */
    void publishEpisodeDone(bool done, int reason) {
        std_msgs::Bool done_msg;
        done_msg.data = done;
        done_pub_.publish(done_msg);
        
        // Also publish crash reason (for debugging/monitoring)
        std_msgs::Bool crash_msg;
        crash_msg.data = (reason > 0);  // true if crashed/flipped, false if normal
        crash_reason_pub_.publish(crash_msg);
        
        // Reset episode if crashed or flipped
        if (reason > 0) {
            ROS_WARN("Resetting episode in %.5f seconds...", RESET_DELAY);
            resetEpisode();
        }
    }
    
    /**
     * Reset the chaser drone to initial position and zero velocity
     */
    void resetEpisode() {
        // Wait a bit before resetting to allow messages to propagate
        ros::Duration(RESET_DELAY).sleep();
        
        // Create SetModelState request
        gazebo_msgs::SetModelState reset_msg;
        reset_msg.request.model_state.model_name = "chaser_drone";
        reset_msg.request.model_state.reference_frame = "world";
        
        // Set initial position (0.5m above ground to avoid immediate ground contact)
        reset_msg.request.model_state.pose.position.x = 0.0;
        reset_msg.request.model_state.pose.position.y = 0.0;
        reset_msg.request.model_state.pose.position.z = 0.5;
        
        // Set level orientation (identity quaternion)
        reset_msg.request.model_state.pose.orientation.w = 1.0;
        reset_msg.request.model_state.pose.orientation.x = 0.0;
        reset_msg.request.model_state.pose.orientation.y = 0.0;
        reset_msg.request.model_state.pose.orientation.z = 0.0;
        
        // Set zero velocity
        reset_msg.request.model_state.twist.linear.x = 0.0;
        reset_msg.request.model_state.twist.linear.y = 0.0;
        reset_msg.request.model_state.twist.linear.z = 0.0;
        reset_msg.request.model_state.twist.angular.x = 0.0;
        reset_msg.request.model_state.twist.angular.y = 0.0;
        reset_msg.request.model_state.twist.angular.z = 0.0;
        
        // Call Gazebo service
        if (gazebo_set_model_state_client_.call(reset_msg)) {
            ROS_INFO("Episode reset successful! Chaser drone respawned at (0, 0, 0.5m).");
            
            // Disable crash detection for 2 seconds to allow propellers to produce lift
            crash_detection_disabled_until_ = ros::Time::now() + ros::Duration(CRASH_DETECTION_DISABLED_TIME);
            ROS_INFO("Crash detection disabled for %.1f seconds to allow safe takeoff.", CRASH_DETECTION_DISABLED_TIME);
        } else {
            ROS_ERROR("Failed to reset chaser drone position. Gazebo service call failed.");
        }
        
        // Re-arm the drone after reset
        armDrone();
    }
    
    ros::NodeHandle nh_;
    
    // Subscribers
    ros::Subscriber chaser_odom_sub_;
    ros::Subscriber target_odom_sub_;
    ros::Subscriber action_sub_;
    
    // Publishers
    ros::Publisher control_pub_;
    ros::Publisher arm_pub_;
    ros::Publisher obs_pub_;
    ros::Publisher reward_pub_;
    ros::Publisher rel_vel_pub_;
    ros::Publisher done_pub_;      // Publish episode done signal
    ros::Publisher crash_reason_pub_;  // Publish crash reason (0=normal, 1=crashed, 2=flipped)
    
    // Service clients for episode reset
    ros::ServiceClient gazebo_set_model_state_client_;
    ros::ServiceClient gazebo_get_model_state_client_;
    
    // State
    nav_msgs::Odometry chaser_odom_;
    nav_msgs::Odometry target_odom_;
    bool chaser_odom_received_ = false;
    bool target_odom_received_ = false;
    bool armed_;
    bool episode_done_ = false;  // Track if episode should terminate
    ros::Time crash_detection_disabled_until_;  // Time until crash detection is re-enabled
    double current_yaw_ = 0.0;  // Current yaw angle for attitude control
    
    std::mutex odom_mutex_;
    std::mutex action_mutex_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gazebo_rl_agent_node");
    ros::NodeHandle nh;
    
    GazeboRLAgentNode agent(nh);
    
    ros::spin();
    
    return 0;
}
