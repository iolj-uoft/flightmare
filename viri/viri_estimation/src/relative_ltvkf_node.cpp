// ROS wrapper around viri_estimation::RelativeLTVKF.
//
// Subscribes:  odometry     (nav_msgs/Odometry, chaser, body-frame twist)   -> prediction @ odom rate
//              measurement  (geometry_msgs/PointStamped, stereo, image stamp) -> delayed update
// Publishes:   relative_state            (viri_msgs/RelativeState)  for guidance
//              filtered_relative_state   (nav_msgs/Odometry)        for logging / RViz
//              filtered_target_position  (geometry_msgs/PointStamped) for RViz
//              TF body_frame -> ga_frame

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <viri_msgs/RelativeState.h>

#include <Eigen/Geometry>
#include <memory>
#include <string>

#include "viri_estimation/frame_utils.hpp"
#include "viri_estimation/relative_ltvkf.hpp"

namespace viri_estimation {

class RelativeLtvkfNode {
 public:
  RelativeLtvkfNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh), tf_buffer_(ros::Duration(10.0)), tf_listener_(tf_buffer_) {
    LtvkfParams p;
    pnh.param("p0_pos", p.p0_pos, p.p0_pos);
    pnh.param("p0_vel", p.p0_vel, p.p0_vel);
    pnh.param("q_pos", p.q_pos, p.q_pos);
    pnh.param("q_vel", p.q_vel, p.q_vel);
    pnh.param("depth_noise_coeff", p.depth_noise_coeff, p.depth_noise_coeff);
    pnh.param("lateral_noise_coeff", p.lateral_noise_coeff, p.lateral_noise_coeff);
    pnh.param("noise_floor", p.noise_floor, p.noise_floor);
    pnh.param("min_range", p.min_range, p.min_range);
    pnh.param("max_dt", p.max_dt, p.max_dt);
    int history_size = static_cast<int>(p.history_size);
    pnh.param("history_size", history_size, history_size);
    p.history_size = static_cast<std::size_t>(std::max(1, history_size));
    filter_ = std::make_unique<RelativeLTVKF>(p);

    pnh.param<std::string>("body_frame", body_frame_, "chaser_drone/base_link");
    pnh.param<std::string>("ga_frame", ga_frame_, "ga_body_frame");
    pnh.param("tf_timeout", tf_timeout_, 0.1);

    std::string odom_topic, meas_topic;
    pnh.param<std::string>("odometry_topic", odom_topic, "noisy_odometry");
    pnh.param<std::string>("measurement_topic", meas_topic, "/target_position_relative");

    odom_sub_ = nh_.subscribe(odom_topic, 10, &RelativeLtvkfNode::odometryCallback, this);
    meas_sub_ = nh_.subscribe(meas_topic, 10, &RelativeLtvkfNode::measurementCallback, this);

    state_pub_ = nh_.advertise<viri_msgs::RelativeState>("relative_state", 10);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("filtered_relative_state", 10);
    point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("filtered_target_position", 10);

    ROS_INFO("[relative_ltvkf] GA-frame LTVKF ready (odom: %s, meas: %s). Waiting for first detection.",
             odom_sub_.getTopic().c_str(), meas_sub_.getTopic().c_str());
  }

 private:
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    const auto& o = msg->pose.pose.orientation;
    const Eigen::Quaterniond q_WB(o.w, o.x, o.y, o.z);
    const Eigen::Matrix3d R_WB = q_WB.normalized().toRotationMatrix();
    const Eigen::Vector3d rpy = rotationToRpy(R_WB);
    const Eigen::Matrix3d R_GA = derotationMatrix(q_WB);

    const auto& tw = msg->twist.twist;
    const Eigen::Vector3d v_body(tw.linear.x, tw.linear.y, tw.linear.z);
    const Eigen::Vector3d w_body(tw.angular.x, tw.angular.y, tw.angular.z);

    v_chaser_ga_ = R_GA * v_body;                                             // eq. 4
    const Eigen::Vector3d w_ga(0.0, 0.0, eulerYawRate(rpy(0), rpy(1), w_body));  // eq. 7
    chaser_yaw_ = rpy(2);

    filter_->predict(msg->header.stamp.toSec(), v_chaser_ga_, w_ga, R_GA);

    publishGaTransform(msg->header.stamp, R_GA);
    if (filter_->initialized()) publishState(msg->header.stamp);
  }

  void measurementCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    // Camera -> body at the image time (the depth node preserves the image stamp)
    geometry_msgs::PointStamped pt_body;
    try {
      tf_buffer_.transform(*msg, pt_body, body_frame_, ros::Duration(tf_timeout_));
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(1.0, "[relative_ltvkf] TF to %s failed: %s", body_frame_.c_str(), ex.what());
      return;
    }
    const Eigen::Vector3d z_body(pt_body.point.x, pt_body.point.y, pt_body.point.z);

    switch (filter_->update(msg->header.stamp.toSec(), z_body)) {
      case RelativeLTVKF::UpdateResult::kInitialized:
        ROS_INFO("[relative_ltvkf] Initialized at p_rel = [%.2f, %.2f, %.2f] (GA).",
                 filter_->state()(0), filter_->state()(1), filter_->state()(2));
        break;
      case RelativeLTVKF::UpdateResult::kNoAttitude:
        ROS_WARN_THROTTLE(1.0, "[relative_ltvkf] No odometry yet; dropping measurement.");
        break;
      case RelativeLTVKF::UpdateResult::kTooOld:
        ROS_WARN_THROTTLE(1.0, "[relative_ltvkf] Measurement older than the history buffer; dropped.");
        break;
      case RelativeLTVKF::UpdateResult::kApplied:
        break;
    }
  }

  void publishGaTransform(const ros::Time& stamp, const Eigen::Matrix3d& R_GA) {
    // base_link -> ga_frame rotation is R_GA^T (GA vectors -> body vectors)
    const Eigen::Quaterniond q_body_ga(R_GA.transpose());
    geometry_msgs::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = body_frame_;
    t.child_frame_id = ga_frame_;
    t.transform.rotation.w = q_body_ga.w();
    t.transform.rotation.x = q_body_ga.x();
    t.transform.rotation.y = q_body_ga.y();
    t.transform.rotation.z = q_body_ga.z();
    tf_broadcaster_.sendTransform(t);
  }

  void publishState(const ros::Time& stamp) {
    const Vector6d& x = filter_->state();
    const Matrix6d& P = filter_->covariance();
    const Eigen::Vector3d v_rel = x.tail<3>() - v_chaser_ga_;

    viri_msgs::RelativeState s;
    s.header.stamp = stamp;
    s.header.frame_id = ga_frame_;
    s.initialized = true;
    s.last_measurement_stamp = ros::Time(filter_->lastMeasurementTime());
    s.p_rel = toVector3(x.head<3>());
    s.v_tgt = toVector3(x.tail<3>());
    s.v_chaser = toVector3(v_chaser_ga_);
    s.v_rel = toVector3(v_rel);
    s.chaser_yaw = chaser_yaw_;
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j) s.covariance[i * 6 + j] = P(i, j);
    state_pub_.publish(s);

    nav_msgs::Odometry odom;
    odom.header = s.header;
    odom.pose.pose.position.x = x(0);
    odom.pose.pose.position.y = x(1);
    odom.pose.pose.position.z = x(2);
    odom.pose.pose.orientation.w = 1.0;
    odom.twist.twist.linear = s.v_tgt;  // target inertial velocity (not relative)
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) {
        odom.pose.covariance[i * 6 + j] = P(i, j);
        odom.twist.covariance[i * 6 + j] = P(3 + i, 3 + j);
      }
    odom_pub_.publish(odom);

    geometry_msgs::PointStamped pt;
    pt.header = s.header;
    pt.point.x = x(0);
    pt.point.y = x(1);
    pt.point.z = x(2);
    point_pub_.publish(pt);
  }

  static geometry_msgs::Vector3 toVector3(const Eigen::Vector3d& v) {
    geometry_msgs::Vector3 m;
    m.x = v.x();
    m.y = v.y();
    m.z = v.z();
    return m;
  }

  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber meas_sub_;
  ros::Publisher state_pub_;
  ros::Publisher odom_pub_;
  ros::Publisher point_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::unique_ptr<RelativeLTVKF> filter_;
  std::string body_frame_;
  std::string ga_frame_;
  double tf_timeout_ = 0.1;

  Eigen::Vector3d v_chaser_ga_ = Eigen::Vector3d::Zero();
  double chaser_yaw_ = 0.0;
};

}  // namespace viri_estimation

int main(int argc, char** argv) {
  ros::init(argc, argv, "relative_ltvkf");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  viri_estimation::RelativeLtvkfNode node(nh, pnh);
  ros::spin();
  return 0;
}
