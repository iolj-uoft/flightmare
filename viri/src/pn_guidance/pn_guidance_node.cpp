/**
 * Proportional Navigation Guidance Node
 *
 * Implements 3D Pure Proportional Navigation for quadrotor interception.
 * Uses the rpg autopilot for takeoff/hover, then takes over via command
 * feedthrough for PN interception.
 *
 * State machine:
 *   WAITING  — sends autopilot/start, waits for autopilot feedback == HOVER
 *   INTERCEPT — publishes PN commands to autopilot/control_command_input
 *
 * Two input modes (selected via ~use_ekf param):
 *
 *   GT mode  (use_ekf=false, default): subscribes to two separate ground-truth
 *            odometry topics and computes relative state internally.
 *
 *   EKF mode (use_ekf=true): subscribes to the relative_eskf_node output
 *            (/chaser_drone/filtered_relative_state) for relative position and
 *            target velocity (both in gravity-aligned/GA frame), plus
 *            /chaser_drone/noisy_odometry for chaser orientation and body-frame
 *            velocity. Computes v_rel = v_target_GA - v_chaser_GA and adjusts
 *            the yaw command to world frame.
 *
 * Math (all vectors in world/GA frame):
 *   r       = target_pos - chaser_pos          (LOS vector)
 *   v_rel   = target_vel - chaser_vel          (relative velocity)
 *   Vc      = -dot(r, v_rel) / |r|            (closing speed, positive = approaching)
 *   Omega   = (r × v_rel) / |r|²              (LOS rate vector)
 *   a_PN    = N * |Vc| * (Omega × r_hat)      (lateral PN acceleration)
 *   a_fwd   = Kp * (Vd - Vc) * r_hat         (forward speed controller)
 *   a_total = a_PN + a_fwd + [0,0,9.81]       (with gravity compensation)
 *
 * Tuning parameters (ROS params):
 *   ~N   (default 4.0) — navigation constant, tune 3–5
 *   ~Kp  (default 1.5) — forward speed P-gain
 *   ~Vd  (default 5.0) — desired closing speed (m/s)
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/ControlCommand.h>
#include <quadrotor_msgs/AutopilotFeedback.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <gazebo_msgs/SetModelState.h>
#include <Eigen/Dense>
#include <mutex>
#include <cmath>

geometry_msgs::Quaternion eulerToQuaternion(double roll, double pitch, double yaw) {
    double cy = std::cos(yaw   * 0.5), sy = std::sin(yaw   * 0.5);
    double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll  * 0.5), sr = std::sin(roll  * 0.5);
    geometry_msgs::Quaternion q;
    q.w =  cy * cp * cr + sy * sp * sr;
    q.x =  cy * cp * sr - sy * sp * cr;
    q.y =  sy * cp * sr + cy * sp * cr;
    q.z =  sy * cp * cr - cy * sp * sr;
    return q;
}

class PNGuidanceNode {
public:
    static constexpr double GRAVITY             = 9.81;
    static constexpr double INTERCEPT_RADIUS    = 0.5;    // m — success distance
    static constexpr double MAX_ROLL_PITCH_RAD  = 20.0 * M_PI / 180.0;
    static constexpr double MIN_THRUST          = 9.81;   // m/s²
    static constexpr double MAX_THRUST          = 20.0;   // m/s²
    static constexpr double ALTITUDE_FLOOR      = 0.8;    // m — soft floor when chasing low
    static constexpr double ALTITUDE_FLOOR_GAIN = 8.0;
    static constexpr double MAX_RANGE           = 30.0;   // m — reset if too far
    static constexpr double CONTROL_HZ          = 50.0;
    static constexpr double RESET_SPAWN_Z       = 0.0;    // m — spawn on ground

    // Autopilot state values (from AutopilotFeedback.msg)
    static constexpr uint8_t AP_OFF   = 0;
    static constexpr uint8_t AP_HOVER = 2;

    enum class State { WAITING, INTERCEPT };

    PNGuidanceNode(ros::NodeHandle& nh) : nh_(nh) {
        ros::NodeHandle pnh("~");
        pnh.param<bool>       ("use_ekf",               use_ekf_,           false);
        pnh.param<std::string>("chaser_odom_topic",     chaser_odom_topic_,
                               "/chaser_drone_0/ground_truth/odometry");
        pnh.param<std::string>("target_odom_topic",     target_odom_topic_,
                               "/target_drone_0/ground_truth/odometry");
        pnh.param<std::string>("filtered_relative_state_topic", ekf_state_topic_,
                               "/chaser_drone/filtered_relative_state");
        pnh.param<std::string>("control_topic",         control_topic_,
                               "/chaser_drone_0/autopilot/control_command_input");
        pnh.param<std::string>("autopilot_start_topic", autopilot_start_topic_,
                               "/chaser_drone_0/autopilot/start");
        pnh.param<std::string>("autopilot_off_topic",   autopilot_off_topic_,
                               "/chaser_drone_0/autopilot/off");
        pnh.param<std::string>("autopilot_fb_topic",    autopilot_fb_topic_,
                               "/chaser_drone_0/autopilot/feedback");
        pnh.param<std::string>("arm_topic",             arm_topic_,
                               "/chaser_drone_0/bridge/arm");
        pnh.param<std::string>("chaser_model_name",     chaser_model_name_, "chaser_drone_0");
        pnh.param<double>     ("N",  N_,  4.0);
        pnh.param<double>     ("Kp", Kp_, 1.5);
        pnh.param<double>     ("Vd", Vd_, 1.5);

        // chaser_odom_sub_ always subscribes — used for crash-height check and,
        // in EKF mode, for chaser orientation + body-frame velocity
        chaser_odom_sub_  = nh_.subscribe(chaser_odom_topic_, 1,
                                          &PNGuidanceNode::chaserOdomCb, this);
        if (use_ekf_) {
            ekf_state_sub_ = nh_.subscribe(ekf_state_topic_, 1,
                                           &PNGuidanceNode::ekfStateCb, this);
            ROS_INFO("[PN] EKF mode — relative state from: %s", ekf_state_topic_.c_str());
        } else {
            target_odom_sub_ = nh_.subscribe(target_odom_topic_, 1,
                                             &PNGuidanceNode::targetOdomCb, this);
            ROS_INFO("[PN] GT mode — target odom from: %s", target_odom_topic_.c_str());
        }
        autopilot_fb_sub_ = nh_.subscribe(autopilot_fb_topic_, 1,
                                          &PNGuidanceNode::autopilotFbCb, this);

        control_pub_         = nh_.advertise<quadrotor_msgs::ControlCommand>(control_topic_, 1);
        arm_pub_             = nh_.advertise<std_msgs::Bool>(arm_topic_, 1);
        autopilot_start_pub_ = nh_.advertise<std_msgs::Empty>(autopilot_start_topic_, 1);
        autopilot_off_pub_   = nh_.advertise<std_msgs::Empty>(autopilot_off_topic_,   1);

        gazebo_set_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>(
            "/gazebo/set_model_state");

        ROS_INFO("[PN] N=%.1f  Kp=%.1f  Vd=%.1f m/s  use_ekf=%s",
                 N_, Kp_, Vd_, use_ekf_ ? "true" : "false");

        // Arm bridge + send autopilot/start to begin hover
        armAndStart();

        control_timer_ = nh_.createTimer(
            ros::Duration(1.0 / CONTROL_HZ),
            &PNGuidanceNode::pnControlLoop, this);
    }

private:
    void chaserOdomCb(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard<std::mutex> lk(odom_mtx_);
        chaser_odom_          = *msg;
        chaser_odom_received_  = true;
    }

    void targetOdomCb(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard<std::mutex> lk(odom_mtx_);
        target_odom_          = *msg;
        target_odom_received_  = true;
    }

    void ekfStateCb(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard<std::mutex> lk(odom_mtx_);
        ekf_state_          = *msg;
        ekf_state_received_  = true;
    }

    void autopilotFbCb(const quadrotor_msgs::AutopilotFeedback::ConstPtr& msg) {
        std::lock_guard<std::mutex> lk(odom_mtx_);
        autopilot_state_ = msg->autopilot_state;
    }

    void pnControlLoop(const ros::TimerEvent&) {
        nav_msgs::Odometry chaser, ekf_state, target;
        uint8_t ap_state;
        {
            std::lock_guard<std::mutex> lk(odom_mtx_);
            if (!chaser_odom_received_) return;
            if (use_ekf_ && !ekf_state_received_) return;
            if (!use_ekf_ && !target_odom_received_) return;
            chaser    = chaser_odom_;
            ekf_state = ekf_state_;
            target    = target_odom_;
            ap_state  = autopilot_state_;
        }

        // ── State: WAITING for autopilot to reach HOVER ──────────────────────────
        if (state_ == State::WAITING) {
            if (ap_state == AP_HOVER) {
                ROS_INFO("[PN] Autopilot in HOVER — switching to INTERCEPT");
                state_ = State::INTERCEPT;
            } else {
                ROS_INFO_THROTTLE(2.0, "[PN] Waiting for autopilot HOVER (state=%d)...", ap_state);
            }
            return;
        }

        // ── State: INTERCEPT ─────────────────────────────────────────────────────
        // Crash check uses chaser_odom_ (GT or noisy) — always in world frame
        const double chaser_z = chaser.pose.pose.position.z;
        if (chaser_z < 0.1) {
            ROS_WARN("[PN] CRASH — chaser z=%.2f m. Resetting.", chaser_z);
            state_ = State::WAITING;
            resetEpisode();
            return;
        }

        // ── Compute r and v_rel ──────────────────────────────────────────────────
        Eigen::Vector3d r, v_rel;
        double yaw_offset = 0.0;  // added to PN yaw to convert GA→world

        if (use_ekf_) {
            // EKF gives r (relative position) and v_tgt in GA frame
            r = Eigen::Vector3d(ekf_state.pose.pose.position.x,
                                ekf_state.pose.pose.position.y,
                                ekf_state.pose.pose.position.z);
            const Eigen::Vector3d v_tgt_GA(ekf_state.twist.twist.linear.x,
                                           ekf_state.twist.twist.linear.y,
                                           ekf_state.twist.twist.linear.z);

            // Rotate chaser body-frame velocity → world → GA frame
            // chaser_odom_ is noisy_odometry: twist in body frame, pose is world orientation
            const auto& q_msg = chaser.pose.pose.orientation;
            const Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
            const Eigen::Vector3d v_body(chaser.twist.twist.linear.x,
                                         chaser.twist.twist.linear.y,
                                         chaser.twist.twist.linear.z);
            // body → world
            const Eigen::Vector3d v_world = q.toRotationMatrix() * v_body;
            // world → GA (remove yaw rotation only)
            yaw_offset = std::atan2(2.0*(q.w()*q.z() + q.x()*q.y()),
                                    1.0 - 2.0*(q.y()*q.y() + q.z()*q.z()));
            const Eigen::AngleAxisd yaw_rot(-yaw_offset, Eigen::Vector3d::UnitZ());
            const Eigen::Vector3d v_chaser_GA = yaw_rot.toRotationMatrix() * v_world;
            v_rel = v_tgt_GA - v_chaser_GA;
        } else {
            // GT mode: compute from two separate odometry topics
            const Eigen::Vector3d chaser_pos(chaser.pose.pose.position.x,
                                              chaser.pose.pose.position.y,
                                              chaser.pose.pose.position.z);
            const Eigen::Vector3d chaser_vel(chaser.twist.twist.linear.x,
                                              chaser.twist.twist.linear.y,
                                              chaser.twist.twist.linear.z);
            const Eigen::Vector3d target_pos(target.pose.pose.position.x,
                                              target.pose.pose.position.y,
                                              target.pose.pose.position.z);
            const Eigen::Vector3d target_vel(target.twist.twist.linear.x,
                                              target.twist.twist.linear.y,
                                              target.twist.twist.linear.z);
            r     = target_pos - chaser_pos;
            v_rel = target_vel - chaser_vel;
        }

        const double r_mag = r.norm();

        if (r_mag < INTERCEPT_RADIUS) {
            ROS_INFO("[PN] *** INTERCEPT! distance=%.3f m ***", r_mag);
            intercept_count_++;
            ROS_INFO("[PN] Total intercepts: %d", intercept_count_);
            state_ = State::WAITING;
            resetEpisode();
            return;
        }

        if (r_mag > MAX_RANGE) {
            ROS_WARN("[PN] Target lost — range=%.1f m. Resetting.", r_mag);
            state_ = State::WAITING;
            resetEpisode();
            return;
        }

        const Eigen::Vector3d r_hat = r / r_mag;
        const double Vc             = -r.dot(v_rel) / r_mag;
        const Eigen::Vector3d Omega = r.cross(v_rel) / (r_mag * r_mag);

        // PN lateral acceleration
        const Eigen::Vector3d a_PN  = N_ * std::abs(Vc) * Omega.cross(r_hat);

        // Forward pursuit: P-controller on closing speed
        const Eigen::Vector3d a_fwd = Kp_ * (Vd_ - Vc) * r_hat;

        // Altitude floor: soft upward spring when chasing low targets
        double az_floor = 0.0;
        if (chaser_z < ALTITUDE_FLOOR) {
            az_floor = ALTITUDE_FLOOR_GAIN * (ALTITUDE_FLOOR - chaser_z);
        }

        const Eigen::Vector3d a_total = a_PN + a_fwd
            + Eigen::Vector3d(0.0, 0.0, GRAVITY + az_floor);

        const double thrust         = std::max(MIN_THRUST, std::min(MAX_THRUST, a_total.norm()));
        const Eigen::Vector3d z_des = a_total.normalized();
        // In EKF mode r is in GA frame, so atan2 gives GA-frame heading.
        // Add yaw_offset to convert to world frame. In GT mode yaw_offset=0.
        const double yaw            = yaw_offset + std::atan2(r.y(), r.x());
        const double roll_cmd       = std::max(-MAX_ROLL_PITCH_RAD,
                                        std::min(MAX_ROLL_PITCH_RAD,
                                            std::atan2(-z_des.y(), z_des.z())));
        const double pitch_cmd      = std::max(-MAX_ROLL_PITCH_RAD,
                                        std::min(MAX_ROLL_PITCH_RAD,
                                            std::atan2( z_des.x(), z_des.z())));

        // Publish via autopilot command feedthrough
        // The autopilot transitions to COMMAND_FEEDTHROUGH state automatically
        // when it receives an armed command on this topic.
        quadrotor_msgs::ControlCommand cmd;
        cmd.header.stamp      = ros::Time::now();
        cmd.header.frame_id   = "world";
        cmd.control_mode      = 1;   // ATTITUDE
        cmd.armed             = true;
        cmd.orientation       = eulerToQuaternion(roll_cmd, pitch_cmd, yaw);
        cmd.bodyrates.x       = 0.0;
        cmd.bodyrates.y       = 0.0;
        cmd.bodyrates.z       = 0.0;
        cmd.collective_thrust = thrust;
        control_pub_.publish(cmd);

        ROS_INFO_THROTTLE(1.0,
            "[PN|%s] dist=%.2fm  Vc=%.2fm/s  z=%.2fm  r=%.1f°  p=%.1f°  thr=%.2f  n=%d",
            use_ekf_ ? "EKF" : "GT",
            r_mag, Vc, chaser_z,
            roll_cmd * 180.0 / M_PI, pitch_cmd * 180.0 / M_PI,
            thrust, intercept_count_);
    }

    void armAndStart() {
        // 1. Arm the rotors bridge
        ros::Duration(1.0).sleep();
        std_msgs::Bool arm_msg;
        arm_msg.data = true;
        ROS_INFO("[PN] Arming chaser bridge...");
        for (int i = 0; i < 20; ++i) {
            arm_pub_.publish(arm_msg);
            ros::Duration(0.05).sleep();
        }
        ROS_INFO("[PN] Bridge armed.");

        // 2. Send autopilot/start — autopilot will run START → HOVER sequence
        ros::Duration(0.5).sleep();
        std_msgs::Empty empty;
        ROS_INFO("[PN] Sending autopilot/start...");
        for (int i = 0; i < 5; ++i) {
            autopilot_start_pub_.publish(empty);
            ros::Duration(0.1).sleep();
        }
        ROS_INFO("[PN] Waiting for autopilot to reach HOVER...");
    }

    void resetEpisode() {
        // 1. Tell autopilot to go off so it doesn't fight the Gazebo teleport
        std_msgs::Empty empty;
        for (int i = 0; i < 3; ++i) {
            autopilot_off_pub_.publish(empty);
            ros::Duration(0.05).sleep();
        }

        ros::Duration(0.1).sleep();

        // 2. Teleport chaser to origin with fully zeroed physics state
        gazebo_msgs::SetModelState req;
        req.request.model_state.model_name         = chaser_model_name_;
        req.request.model_state.reference_frame    = "world";
        req.request.model_state.pose.position.x    = 0.0;
        req.request.model_state.pose.position.y    = 0.0;
        req.request.model_state.pose.position.z    = RESET_SPAWN_Z;
        req.request.model_state.pose.orientation.x = 0.0;
        req.request.model_state.pose.orientation.y = 0.0;
        req.request.model_state.pose.orientation.z = 0.0;
        req.request.model_state.pose.orientation.w = 1.0;
        req.request.model_state.twist.linear.x     = 0.0;
        req.request.model_state.twist.linear.y     = 0.0;
        req.request.model_state.twist.linear.z     = 0.0;
        req.request.model_state.twist.angular.x    = 0.0;
        req.request.model_state.twist.angular.y    = 0.0;
        req.request.model_state.twist.angular.z    = 0.0;

        if (gazebo_set_client_.call(req)) {
            ros::Duration(0.05).sleep();
            gazebo_set_client_.call(req);  // second call flushes residual contact forces
            ROS_INFO("[PN] Reset — chaser at (0, 0, %.1f).", RESET_SPAWN_Z);
        } else {
            ROS_ERROR("[PN] Gazebo reset service call failed.");
        }

        // 3. Re-arm bridge and restart autopilot hover
        armAndStart();
    }

    // Node infrastructure
    ros::NodeHandle     nh_;
    ros::Subscriber     chaser_odom_sub_;
    ros::Subscriber     target_odom_sub_;   // GT mode only
    ros::Subscriber     ekf_state_sub_;     // EKF mode only
    ros::Subscriber     autopilot_fb_sub_;
    ros::Publisher      control_pub_;
    ros::Publisher      arm_pub_;
    ros::Publisher      autopilot_start_pub_;
    ros::Publisher      autopilot_off_pub_;
    ros::ServiceClient  gazebo_set_client_;
    ros::Timer          control_timer_;

    // Odometry + autopilot state
    nav_msgs::Odometry  chaser_odom_;
    nav_msgs::Odometry  target_odom_;    // GT mode
    nav_msgs::Odometry  ekf_state_;     // EKF mode
    bool                chaser_odom_received_  = false;
    bool                target_odom_received_  = false;
    bool                ekf_state_received_    = false;
    uint8_t             autopilot_state_       = AP_OFF;
    std::mutex          odom_mtx_;

    // Episode state
    int                 intercept_count_ = 0;
    State               state_           = State::WAITING;

    // Parameters
    bool                use_ekf_            = false;
    std::string         chaser_odom_topic_;
    std::string         target_odom_topic_;
    std::string         ekf_state_topic_;
    std::string         control_topic_;
    std::string         autopilot_start_topic_;
    std::string         autopilot_off_topic_;
    std::string         autopilot_fb_topic_;
    std::string         arm_topic_;
    std::string         chaser_model_name_;
    double              N_;
    double              Kp_;
    double              Vd_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pn_guidance_node");
    ros::NodeHandle nh;
    PNGuidanceNode node(nh);
    ros::spin();
    return 0;
}
