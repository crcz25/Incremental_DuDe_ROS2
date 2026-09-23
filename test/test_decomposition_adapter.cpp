#include <gtest/gtest.h>

#include <cmath>

#include "inc_dude/decomposition_adapter.hpp"

using namespace inc_dude;

namespace {

// Contour of the boundary pixels of an axis-aligned pixel block, as
// cv::findContours(CHAIN_APPROX_SIMPLE) would return it.
std::vector<cv::Point> block(int c0, int r0, int c1, int r1) {
  return {{c0, r0}, {c0, r1}, {c1, r1}, {c1, r0}};
}

}  // namespace

TEST(DecompositionAdapter, GridToMapUsesCellCentersOriginAndYaw) {
  GridGeometry g;
  g.origin_x = -10.0;
  g.origin_y = -5.0;
  g.resolution = 0.05;
  Point2 p = gridToMap(g, 0, 0);
  EXPECT_NEAR(p.x, -9.975, 1e-12);
  EXPECT_NEAR(p.y, -4.975, 1e-12);
  p = gridToMap(g, 10, 20);
  EXPECT_NEAR(p.x, -10.0 + 10.5 * 0.05, 1e-12);
  EXPECT_NEAR(p.y, -5.0 + 20.5 * 0.05, 1e-12);

  g.origin_yaw = M_PI / 2.0;
  p = gridToMap(g, 1, 0);  // local (0.075, 0.025) rotated by +90 deg
  EXPECT_NEAR(p.x, -10.0 - 0.025, 1e-12);
  EXPECT_NEAR(p.y, -5.0 + 0.075, 1e-12);
}

TEST(DecompositionAdapter, ConvertsContoursKeepsLocalIdsAndComputesAdjacency) {
  GridGeometry g;
  g.origin_x = 2.0;
  g.origin_y = 3.0;
  g.resolution = 0.05;

  const std::vector<std::vector<cv::Point>> contours = {
      block(0, 0, 39, 39),    // 0: 2 m x 2 m
      {{5, 5}, {6, 6}},       // 1: degenerate, skipped
      block(40, 0, 79, 39),   // 2: right of 0, touching
      block(40, 40, 79, 79),  // 3: above 2, only a corner touches 0
      block(200, 0, 239, 39)  // 4: far away
  };
  const std::vector<Region2D> regions =
      regionsFromContours(contours, g, AdapterConfig());

  ASSERT_EQ(regions.size(), 4u);
  EXPECT_EQ(regions[0].local_id, 0);
  EXPECT_EQ(regions[1].local_id, 2);
  EXPECT_EQ(regions[2].local_id, 3);
  EXPECT_EQ(regions[3].local_id, 4);

  // Polygon through boundary-pixel centers: 39 x 39 cells.
  EXPECT_NEAR(regions[0].area, (39 * 0.05) * (39 * 0.05), 1e-9);
  EXPECT_NEAR(regions[0].centroid.x, 2.0 + 20.0 * 0.05, 1e-9);
  EXPECT_NEAR(regions[0].centroid.y, 3.0 + 20.0 * 0.05, 1e-9);

  EXPECT_EQ(regions[0].adjacent_local_ids, (std::vector<int>{2}));
  EXPECT_EQ(regions[1].adjacent_local_ids, (std::vector<int>{0, 3}));
  EXPECT_EQ(regions[2].adjacent_local_ids, (std::vector<int>{2}));
  EXPECT_TRUE(regions[3].adjacent_local_ids.empty());
}
