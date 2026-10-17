// Guidance node: LTVKF relative state -> ProNav / Hybrid ProNav -> RPG autopilot feedthrough.
//
// Subscribes:  relative_state            (viri_msgs/RelativeState, from the LTVKF)
//              odometry                  (nav_msgs/Odometry, chaser attitude)
//              autopilot/feedback        (quadrotor_msgs/AutopilotFeedback)
//              /engagement/intercepted   (std_msgs/Bool, ground-truth intercept detection)
// Publishes:   autopilot/control_command_input (quadrotor_msgs/ControlCommand) while engaged
//              ~status                   (viri_msgs/GuidanceStatus)
// Services:    ~engage, ~disengage       (std_srvs/Trigger)
//
// When the node stops publishing (idle, intercept, disengage), the autopilot leaves command
// feedthrough after its input timeout and holds hover. A stale estimate does not stop it:
// pursuit coasts on the LTVKF prediction and the status reports STALE.

#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/AutopilotFeedback.h>
#include <quadrotor_msgs/ControlCommand.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Trigger.h>
#include <viri_msgs/GuidanceStatus.h>
#include <viri_msgs/RelativeState.h>

#include <memory>
#include <string>

#include "viri_estimation/frame_utils.hpp"
#include "viri_guidance/hybrid_pro_nav.hpp"
#include "viri_guidance/pro_nav.hpp"

namespace viri_guidance {

namespace {

constexpr double kDegToRad = M_PI / 180.0;

Eigen::Vector3d toEigen(const geometry_msgs::Vector3& v) { return {v.x, v.y, v.z}; }

geometry_msgs::Vector3 toMsg(const Eigen::Vector3d& v) {
  geometry_msgs::Vector3 m;
  m.x = v.x();
  m.y = v.y();
  m.z = v.z();
  return m;
}

}  // namespace

class GuidanceNode {
 public:
  GuidanceNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh) {
    ProNavParams pn;
    pnh.param("navigation_gain", pn.navigation_gain, pn.navigation_gain);
    pnh.param("speed_gain", pn.speed_gain, pn.speed_gain);
    pnh.param("desired_closing_speed", pn.desired_closing_speed, pn.desired_closing_speed);
    pnh.param("gravity", pn.gravity, pn.gravity);
    pnh.param("min_range", pn.min_range, pn.min_range);
    double max_tilt_deg = pn.limits.max_tilt / kDegToRad;
    pnh.param("max_tilt_deg", max_tilt_deg, max_tilt_deg);
    pn.limits.max_tilt = max_tilt_deg * kDegToRad;
    pnh.param("thrust_min", pn.limits.thrust_min, pn.limits.thrust_min);
    pnh.param("thrust_max", pn.limits.thrust_max, pn.limits.thrust_max);

    HybridParams hp;
    double threshold_deg = hp.heading_threshold / kDegToRad, hysteresis_deg = 0.0;
    pnh.param("heading_threshold_deg", threshold_deg, threshold_deg);
    pnh.param("hysteresis_deg", hysteresis_deg, hysteresis_deg);
    hp.heading_threshold = threshold_deg * kDegToRad;
    hp.hysteresis = hysteresis_deg * kDegToRad;
    pnh.param("yaw_gain", hp.yaw_gain, hp.yaw_gain);
    pnh.param("max_yaw_rate", hp.max_yaw_rate, hp.max_yaw_rate);

    std::string law;
    pnh.param<std::string>("law", law, "hybrid");
    if (law == "pronav") {
      law_ = std::make_unique<ProNav>(pn);
    } else {
      if (law != "hybrid") ROS_WARN("[guidance] Unknown law '%s', using 'hybrid'.", law.c_str());
      law_ = std::make_unique<HybridProNav>(pn, hp);
    }

    std::string command_mode;
    pnh.param<std::string>("command_mode", command_mode, "body_rates");
    use_body_rates_ = command_mode != "attitude";
    pnh.param("k_roll_pitch", k_rp_, 12.0);
    pnh.param("k_yaw", k_yaw_, 5.0);

    pnh.param("auto_takeoff", auto_takeoff_, true);
    pnh.param("auto_engage", auto_engage_, true);
    pnh.param("takeoff_delay", takeoff_delay_, 1.0);
    pnh.param("max_measurement_age", max_measurement_age_, 1.0);
    double rate = 50.0;
    pnh.param("rate", rate, rate);

    std::string state_topic, odom_topic, intercept_topic;
    pnh.param<std::string>("relative_state_topic", state_topic, "relative_state");
    pnh.param<std::string>("odometry_topic", odom_topic, "noisy_odometry");
    pnh.param<std::string>("intercepted_topic", intercept_topic, "/engagement/intercepted");

    state_sub_ = nh_.subscribe(state_topic, 1, &GuidanceNode::stateCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic, 1, &GuidanceNode::odomCallback, this);
    feedback_sub_ = nh_.subscribe("autopilot/feedback", 1, &GuidanceNode::feedbackCallback, this);
    intercept_sub_ = nh_.subscribe(intercept_topic, 1, &GuidanceNode::interceptCallback, this);

    command_pub_ = nh_.advertise<quadrotor_msgs::ControlCommand>("autopilot/control_command_input", 1);
    arm_pub_ = nh_.advertise<std_msgs::Bool>("bridge/arm", 1, true);
    start_pub_ = nh_.advertise<std_msgs::Empty>("autopilot/start", 1, true);
    status_pub_ = pnh.advertise<viri_msgs::GuidanceStatus>("status", 10);

    engage_srv_ = pnh.advertiseService("engage", &GuidanceNode::engageService, this);
    disengage_srv_ = pnh.advertiseService("disengage", &GuidanceNode::disengageService, this);

    timer_ = nh_.createTimer(ros::Duration(1.0 / rate), &GuidanceNode::timerCallback, this);

    ROS_INFO("[guidance] law=%s command_mode=%s rate=%.0f Hz auto_takeoff=%d auto_engage=%d",
             law_->name().c_str(), use_body_rates_ ? "body_rates" : "attitude", rate,
             auto_takeoff_, auto_engage_);
  }

 private:
  void stateCallback(const viri_msgs::RelativeState::ConstPtr& msg) { state_ = msg; }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (!odom_) first_odom_time_ = ros::Time::now();
    odom_ = msg;
  }

  void feedbackCallback(const quadrotor_msgs::AutopilotFeedback::ConstPtr& msg) {
    autopilot_state_ = msg->autopilot_state;
  }

  void interceptCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data && !intercepted_) ROS_INFO("[guidance] Intercept reported; disengaging.");
    intercepted_ = msg->data;
  }

  bool engageService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    intercepted_ = false;
    engage();
    res.success = true;
    res.message = "engaged (" + law_->name() + ")";
    return true;
  }

  bool disengageService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    engaged_ = false;
    res.success = true;
    res.message = "disengaged";
    return true;
  }

  void engage() {
    if (!engaged_) ROS_INFO("[guidance] Engaging with %s guidance.", law_->name().c_str());
    engaged_ = true;
    law_->reset();
  }

  void maybeTakeOff(const ros::Time& now) {
    if (!auto_takeoff_ || takeoff_sent_ || !odom_) return;
    if ((now - first_odom_time_).toSec() < takeoff_delay_) return;
    std_msgs::Bool arm;
    arm.data = true;
    arm_pub_.publish(arm);
    start_pub_.publish(std_msgs::Empty());
    takeoff_sent_ = true;
    ROS_INFO("[guidance] Armed chaser and requested autopilot take-off.");
  }

  void timerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    maybeTakeOff(now);

    viri_msgs::GuidanceStatus status;
    status.header.stamp = now;
    status.header.frame_id = state_ ? state_->header.frame_id : "";
    status.law = law_->name();
    status.mode = viri_msgs::GuidanceStatus::IDLE;

    const bool estimate_ready = state_ && state_->initialized && odom_;

    if (intercepted_) {
      engaged_ = false;
      status.mode = viri_msgs::GuidanceStatus::INTERCEPTED;
    } else if (!engaged_ && auto_engage_ && estimate_ready &&
               autopilot_state_ == quadrotor_msgs::AutopilotFeedback::HOVER) {
      engage();
    }

    if (engaged_ && estimate_ready) {
      // Without stereo updates the LTVKF keeps predicting at odometry rate (constant target
      // velocity), so pursuit continues on that prediction rather than handing back to hover.
      runGuidance(now, status);
      const double age = (now - state_->last_measurement_stamp).toSec();
      if (age > max_measurement_age_) {
        status.mode = viri_msgs::GuidanceStatus::STALE;
        ROS_WARN_THROTTLE(1.0, "[guidance] No stereo update for %.2f s; coasting on prediction.", age);
      }
    }

    status_pub_.publish(status);
  }

  void runGuidance(const ros::Time& now, viri_msgs::GuidanceStatus& status) {
    const auto& o = odom_->pose.pose.orientation;
    const Eigen::Quaterniond q_WB = Eigen::Quaterniond(o.w, o.x, o.y, o.z).normalized();
    const double yaw = viri_estimation::rotationToRpy(q_WB.toRotationMatrix())(2);

    GuidanceInput in;
    in.r = toEigen(state_->p_rel);
    in.v_tgt = toEigen(state_->v_tgt);
    in.v_chaser = toEigen(state_->v_chaser);
    in.R_GB = viri_estimation::derotationMatrix(q_WB);

    const GuidanceOutput out = law_->compute(in);
    const AttitudeCommand& c = out.command;
    const Eigen::Quaterniond q_des = attitudeToQuaternion(c.roll, c.pitch, yaw);

    quadrotor_msgs::ControlCommand cmd;
    cmd.header.stamp = now;
    cmd.header.frame_id = "world";
    cmd.armed = true;
    cmd.expected_execution_time = now;
    cmd.orientation.w = q_des.w();
    cmd.orientation.x = q_des.x();
    cmd.orientation.y = q_des.y();
    cmd.orientation.z = q_des.z();
    cmd.collective_thrust = c.thrust;
    const Eigen::Vector3d yaw_ff(0.0, 0.0, c.yaw_rate);
    if (use_body_rates_) {
      cmd.control_mode = quadrotor_msgs::ControlCommand::BODY_RATES;
      cmd.bodyrates = toMsg(attitudeErrorToBodyRates(q_WB, q_des, k_rp_, k_yaw_, yaw_ff));
    } else {
      cmd.control_mode = quadrotor_msgs::ControlCommand::ATTITUDE;
      cmd.bodyrates = toMsg(yaw_ff);
    }
    command_pub_.publish(cmd);

    status.mode = out.mode == GuidanceMode::kHeading ? viri_msgs::GuidanceStatus::HEADING
                                                     : viri_msgs::GuidanceStatus::PRONAV;
    status.range = out.range;
    status.closing_speed = out.closing_speed;
    status.azimuth_error = out.azimuth_error;
    status.los_rate = toMsg(out.los_rate);
    status.accel_cmd = toMsg(out.accel);
    status.roll_cmd = c.roll;
    status.pitch_cmd = c.pitch;
    status.yaw_rate_cmd = c.yaw_rate;
    status.thrust_cmd = c.thrust;
  }

  ros::NodeHandle nh_;
  ros::Subscriber state_sub_, odom_sub_, feedback_sub_, intercept_sub_;
  ros::Publisher command_pub_, arm_pub_, start_pub_, status_pub_;
  ros::ServiceServer engage_srv_, disengage_srv_;
  ros::Timer timer_;

  std::unique_ptr<GuidanceLaw> law_;
  bool use_body_rates_ = true;
  double k_rp_ = 12.0, k_yaw_ = 5.0;
  bool auto_takeoff_ = true, auto_engage_ = true;
  double takeoff_delay_ = 1.0, max_measurement_age_ = 1.0;

  viri_msgs::RelativeState::ConstPtr state_;
  nav_msgs::Odometry::ConstPtr odom_;
  ros::Time first_odom_time_;
  uint8_t autopilot_state_ = quadrotor_msgs::AutopilotFeedback::OFF;
  bool takeoff_sent_ = false;
  bool engaged_ = false;
  bool intercepted_ = false;
};

}  // namespace viri_guidance

int main(int argc, char** argv) {
  ros::init(argc, argv, "guidance");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  viri_guidance::GuidanceNode node(nh, pnh);
  ros::spin();
  return 0;
}
