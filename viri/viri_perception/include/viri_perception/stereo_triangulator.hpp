#pragma once

#include <Eigen/Dense>
#include <optional>

namespace viri_perception {

/// Rectified stereo rig + outlier rejection thresholds (paper Sec. III-D.3).
struct StereoParams {
  double focal_length = 311.77;  // [px]
  double baseline = 0.2;         // [m]
  double cx = 320.0;             // [px]
  double cy = 180.0;             // [px]
  double image_width = 640.0;    // [px]
  double edge_margin = 60.0;     // reject x_L outside [margin, width - margin] [px]
  double max_y_jitter = 15.0;    // reject |y_L - y_R| above this [px]
  double min_disparity = 0.5;    // reject d = x_L - x_R at or below this [px]
};

/// Bounding-box centres of the target in the left and right images.
struct StereoDetection {
  double x_left, y_left, x_right, y_right;
};

enum class Rejection { kNone, kEdgeMargin, kVerticalJitter, kDegenerateDisparity };

struct TriangulationResult {
  Rejection rejection = Rejection::kNone;
  /// Target position in the left camera optical frame (x right, y down, z forward).
  std::optional<Eigen::Vector3d> point;
};

const char* toString(Rejection r);

/**
 * Pinhole stereo triangulation of a single correspondence (eq. 3):
 *   d = x_L - x_R,  Z = f B / d,  X = (x_L - c_x) Z / f,  Y = (y_bar - c_y) Z / f
 * after the edge-margin, vertical-jitter and degenerate-disparity checks.
 */
class StereoTriangulator {
 public:
  explicit StereoTriangulator(const StereoParams& params = StereoParams()) : params_(params) {}

  TriangulationResult triangulate(const StereoDetection& det) const;

  const StereoParams& params() const { return params_; }

 private:
  StereoParams params_;
};

}  // namespace viri_perception
