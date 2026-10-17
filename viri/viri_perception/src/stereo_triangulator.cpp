#include "viri_perception/stereo_triangulator.hpp"

#include <cmath>

namespace viri_perception {

const char* toString(Rejection r) {
  switch (r) {
    case Rejection::kNone: return "none";
    case Rejection::kEdgeMargin: return "edge_margin";
    case Rejection::kVerticalJitter: return "vertical_jitter";
    case Rejection::kDegenerateDisparity: return "degenerate_disparity";
  }
  return "unknown";
}

TriangulationResult StereoTriangulator::triangulate(const StereoDetection& det) const {
  TriangulationResult result;
  const StereoParams& p = params_;

  // 1. Edge margin: a clipped target biases the box centre and therefore the disparity
  if (det.x_left < p.edge_margin || det.x_left > p.image_width - p.edge_margin) {
    result.rejection = Rejection::kEdgeMargin;
    return result;
  }

  // 2. Vertical jitter: epipolar lines are horizontal in a rectified pair
  if (std::abs(det.y_left - det.y_right) > p.max_y_jitter) {
    result.rejection = Rejection::kVerticalJitter;
    return result;
  }

  // 3. Degenerate disparity: too far for measurable parallax (or a mismatch)
  const double d = det.x_left - det.x_right;
  if (d <= p.min_disparity) {
    result.rejection = Rejection::kDegenerateDisparity;
    return result;
  }

  // Triangulation (eq. 3)
  const double Z = p.focal_length * p.baseline / d;
  const double X = (det.x_left - p.cx) * Z / p.focal_length;
  const double y_bar = 0.5 * (det.y_left + det.y_right);
  const double Y = (y_bar - p.cy) * Z / p.focal_length;
  result.point = Eigen::Vector3d(X, Y, Z);
  return result;
}

}  // namespace viri_perception
