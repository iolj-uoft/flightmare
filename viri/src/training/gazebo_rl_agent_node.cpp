#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/ControlCommand.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
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
 */
class GazeboRLAgentNode {
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
        
        ROS_INFO("GazeboRLAgentNode initialized");
        ROS_INFO("  Subscribing to: ground_truth/odometry, /target_drone/ground_truth/odometry, rl_action");
        ROS_INFO("  Publishing to: control_command (4D attitude+thrust), bridge/arm");
        
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
        cmd.control_mode = 2;  // BODY_RATES mode (direct rate control - simpler)
        cmd.armed = true;
        
        // Scale actions to reasonable ranges
        double roll_rad = roll * M_PI / 6.0;      // [-pi/6, pi/6] (±30 degrees max roll)
        double pitch_rad = pitch * M_PI / 6.0;    // [-pi/6, pi/6] (±30 degrees max pitch)
        double yaw_rate_rad = yaw_rate * M_PI;    // [-pi, pi] rad/s (yaw angular velocity)
        
        // In BODY_RATES mode, we command angular velocities (p, q, r) directly
        // Use roll/pitch inputs as feedforward rates, scale conservatively
        double roll_rate_cmd = roll * 1.0;        // [-1, 1] rad/s (roll rate)
        double pitch_rate_cmd = pitch * 1.0;      // [-1, 1] rad/s (pitch rate)
        
        // Set body rates
        cmd.bodyrates.x = roll_rate_cmd;      // p (roll rate)
        cmd.bodyrates.y = pitch_rate_cmd;     // q (pitch rate)
        cmd.bodyrates.z = yaw_rate_rad;       // r (yaw rate)
        
        // Orientation can be left at default (identity) for BODY_RATES mode
        // The controller will try to maintain current attitude while applying rates
        cmd.orientation.w = 1.0;  // Identity quaternion
        cmd.orientation.x = 0.0;
        cmd.orientation.y = 0.0;
        cmd.orientation.z = 0.0;
        
        // Thrust: collective_thrust is MASS-NORMALIZED
        // For hover: collective_thrust = gravity = 9.81 m/s^2
        // To climb/descend: add/subtract from gravity
        // The rpg_rotors_interface internally multiplies by mass to get actual motor thrust
        double gravity = 9.81;  // m/s^2
        double max_accel = 5.0;  // m/s^2 (safe limit)
        
        // Normalize thrust input from [-1, 1] to [-5, +5] m/s^2 then add gravity
        cmd.collective_thrust = gravity + (thrust * max_accel);
        
        ROS_INFO_THROTTLE(0.5, "[Control BODY_RATES] p=%.2f q=%.2f r=%.2f rad/s thrust=%.2f m/s²(hover=%.2f)",
                          roll_rate_cmd, pitch_rate_cmd, yaw_rate_rad,
                          cmd.collective_thrust, gravity);
        
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
        
        // Reward function: negative distance, bonus for proximity
        double pos_error = rel_pos.norm();
        double vel_error = rel_vel.norm();
        
        double reward = -0.1 * pos_error - 0.01 * vel_error;
        
        if (pos_error < 1.0) {
            reward += 10.0;  // Bonus for being close
        }
        
        if (pos_error > 100.0) {
            reward -= 50.0;  // Penalty for going too far
        }
        
        std_msgs::Float32 reward_msg;
        reward_msg.data = reward;
        reward_pub_.publish(reward_msg);
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
    
    // State
    nav_msgs::Odometry chaser_odom_;
    nav_msgs::Odometry target_odom_;
    bool chaser_odom_received_ = false;
    bool target_odom_received_ = false;
    bool armed_;
    
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
