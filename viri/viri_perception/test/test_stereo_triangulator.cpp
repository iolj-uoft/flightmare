#include <gtest/gtest.h>

#include "viri_perception/stereo_triangulator.hpp"

using namespace viri_perception;

TEST(StereoTriangulator, PinholeModel) {
  StereoTriangulator tri;
  const auto& p = tri.params();
  // 10 px disparity -> Z = f B / 10
  const TriangulationResult r = tri.triangulate({400.0, 200.0, 390.0, 204.0});
  ASSERT_TRUE(r.point.has_value());
  const double Z = p.focal_length * p.baseline / 10.0;
  EXPECT_NEAR(r.point->z(), Z, 1e-12);
  EXPECT_NEAR(r.point->x(), (400.0 - p.cx) * Z / p.focal_length, 1e-12);
  EXPECT_NEAR(r.point->y(), (202.0 - p.cy) * Z / p.focal_length, 1e-12);
  EXPECT_EQ(r.rejection, Rejection::kNone);
}

TEST(StereoTriangulator, PaperFocalLengthAtTwoMetres) {
  // f B / Z = 311.77 * 0.2 / 2 = 31.18 px disparity at 2 m
  StereoTriangulator tri;
  const double d = 311.77 * 0.2 / 2.0;
  const TriangulationResult r = tri.triangulate({320.0, 180.0, 320.0 - d, 180.0});
  ASSERT_TRUE(r.point.has_value());
  EXPECT_NEAR(r.point->z(), 2.0, 1e-9);
  EXPECT_NEAR(r.point->x(), 0.0, 1e-9);
}

TEST(StereoTriangulator, RejectsEdgeMargin) {
  StereoTriangulator tri;
  EXPECT_EQ(tri.triangulate({59.0, 180.0, 40.0, 180.0}).rejection, Rejection::kEdgeMargin);
  EXPECT_EQ(tri.triangulate({581.0, 180.0, 560.0, 180.0}).rejection, Rejection::kEdgeMargin);
  // Only x_L is checked (paper): x_R may sit inside the margin
  EXPECT_EQ(tri.triangulate({70.0, 180.0, 50.0, 180.0}).rejection, Rejection::kNone);
}

TEST(StereoTriangulator, RejectsVerticalJitter) {
  StereoTriangulator tri;
  EXPECT_EQ(tri.triangulate({300.0, 180.0, 290.0, 196.0}).rejection, Rejection::kVerticalJitter);
  EXPECT_EQ(tri.triangulate({300.0, 180.0, 290.0, 195.0}).rejection, Rejection::kNone);
}

TEST(StereoTriangulator, RejectsDegenerateDisparity) {
  StereoTriangulator tri;
  EXPECT_EQ(tri.triangulate({300.0, 180.0, 299.5, 180.0}).rejection, Rejection::kDegenerateDisparity);
  EXPECT_EQ(tri.triangulate({300.0, 180.0, 310.0, 180.0}).rejection, Rejection::kDegenerateDisparity);
  EXPECT_FALSE(tri.triangulate({300.0, 180.0, 300.0, 180.0}).point.has_value());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
