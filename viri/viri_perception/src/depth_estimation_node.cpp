// Stereo depth estimation: YOLO stereo box centres -> 3-D target point in the left camera
// optical frame, stamped with the original image time (rejection rule 4, paper Sec. III-D.3).
//
// The YOLO node packs the stereo correspondence into a vision_msgs/Detection2D:
//   center.x = (x_L + x_R) / 2,  center.y = (y_L + y_R) / 2,
//   size_x   =  x_R - x_L,       size_y   =  y_R - y_L

#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <vision_msgs/Detection2DArray.h>

#include <array>
#include <string>

#include "viri_perception/stereo_triangulator.hpp"

namespace viri_perception {

class DepthEstimationNode {
 public:
  DepthEstimationNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    StereoParams p;
    pnh.param("focal_length", p.focal_length, p.focal_length);
    pnh.param("baseline", p.baseline, p.baseline);
    pnh.param("c_x", p.cx, p.cx);
    pnh.param("c_y", p.cy, p.cy);
    pnh.param("image_width", p.image_width, p.image_width);
    pnh.param("edge_margin", p.edge_margin, p.edge_margin);
    pnh.param("max_y_jitter", p.max_y_jitter, p.max_y_jitter);
    pnh.param("min_disparity", p.min_disparity, p.min_disparity);
    triangulator_ = StereoTriangulator(p);

    std::string bbox_topic, target_topic;
    pnh.param<std::string>("bbox_topic", bbox_topic, "/ultralytics/detection/target_centers");
    pnh.param<std::string>("target_position_topic", target_topic, "/target_position_relative");
    pnh.param<std::string>("camera_frame", camera_frame_, "chaser_drone/camera_left_optical");

    bbox_sub_ = nh.subscribe(bbox_topic, 1, &DepthEstimationNode::bboxCallback, this);
    point_pub_ = nh.advertise<geometry_msgs::PointStamped>(target_topic, 1);

    ROS_INFO("[depth_estimation] f=%.2f px, B=%.3f m, c=(%.1f, %.1f), publishing on %s",
             p.focal_length, p.baseline, p.cx, p.cy, point_pub_.getTopic().c_str());
  }

 private:
  void bboxCallback(const vision_msgs::Detection2DArray::ConstPtr& msg) {
    if (msg->detections.empty()) return;  // no target: the LTVKF coasts

    const auto& bbox = msg->detections[0].bbox;
    const StereoDetection det{bbox.center.x - 0.5 * bbox.size_x, bbox.center.y - 0.5 * bbox.size_y,
                              bbox.center.x + 0.5 * bbox.size_x, bbox.center.y + 0.5 * bbox.size_y};

    const TriangulationResult res = triangulator_.triangulate(det);
    if (!res.point) {
      ++rejections_[static_cast<std::size_t>(res.rejection)];
      ROS_WARN_THROTTLE(2.0, "[depth_estimation] Rejected frames - edge: %zu, jitter: %zu, disparity: %zu",
                        rejections_[1], rejections_[2], rejections_[3]);
      return;
    }

    geometry_msgs::PointStamped out;
    out.header.stamp = msg->header.stamp;  // image acquisition time, not processing time
    out.header.frame_id = camera_frame_;
    out.point.x = res.point->x();
    out.point.y = res.point->y();
    out.point.z = res.point->z();
    point_pub_.publish(out);
  }

  StereoTriangulator triangulator_;
  std::string camera_frame_;
  ros::Subscriber bbox_sub_;
  ros::Publisher point_pub_;
  std::array<std::size_t, 4> rejections_{};
};

}  // namespace viri_perception

int main(int argc, char** argv) {
  ros::init(argc, argv, "depth_estimation");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  viri_perception::DepthEstimationNode node(nh, pnh);
  ros::spin();
  return 0;
}
