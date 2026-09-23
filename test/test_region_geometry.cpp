#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "inc_dude/hungarian.hpp"
#include "inc_dude/region_geometry.hpp"
#include "test_helpers.hpp"

using namespace inc_dude;
using inc_dude::test::rect;

TEST(RegionGeometry, AreaAndCentroidOfRectangle) {
  const Polygon2 r = rect(1.0, 2.0, 3.0, 5.0);
  EXPECT_DOUBLE_EQ(geometry::polygonArea(r), 6.0);
  const Point2 c = geometry::polygonCentroid(r);
  EXPECT_NEAR(c.x, 2.0, 1e-12);
  EXPECT_NEAR(c.y, 3.5, 1e-12);
}

TEST(RegionGeometry, CentroidOfLShapeFarFromOrigin) {
  // Two unit squares side by side plus one on top of the left one.
  const double o = 1000.0;
  const Polygon2 l = {{o, o},         {o + 2, o},     {o + 2, o + 1},
                      {o + 1, o + 1}, {o + 1, o + 2}, {o, o + 2}};
  EXPECT_NEAR(geometry::polygonArea(l), 3.0, 1e-9);
  const Point2 c = geometry::polygonCentroid(l);
  EXPECT_NEAR(c.x, o + 5.0 / 6.0, 1e-9);
  EXPECT_NEAR(c.y, o + 5.0 / 6.0, 1e-9);
}

TEST(RegionGeometry, NormalizeRemovesDuplicatesAndOrientsCounterClockwise) {
  Polygon2 cw = {{0, 0}, {0, 0}, {0, 1}, {1, 1}, {1, 0}, {0, 0}};
  const Polygon2 n = geometry::normalizePolygon(cw);
  ASSERT_EQ(n.size(), 4u);
  EXPECT_GT(geometry::signedArea(n), 0.0);
}

TEST(RegionGeometry, ScanlineAreaMatchesShoelace) {
  const Polygon2 r = rect(0.0, 0.0, 2.0, 3.0);
  EXPECT_NEAR(geometry::scanlineArea(r, 0.05), 6.0, 2.0 * 0.05);
}

TEST(RegionGeometry, IntersectionOfOverlappingRectangles) {
  EXPECT_NEAR(geometry::intersectionArea(rect(0, 0, 2, 2), rect(1, 0, 3, 2),
                                         0.05),
              2.0, 0.1);
  EXPECT_NEAR(geometry::intersectionArea(rect(0, 0, 4, 4), rect(1, 1, 2, 2),
                                         0.05),
              1.0, 0.1);
  EXPECT_EQ(geometry::intersectionArea(rect(0, 0, 1, 1), rect(5, 5, 6, 6),
                                       0.05),
            0.0);
}

TEST(RegionGeometry, SharedEdgeIsNotCountedAsOverlap) {
  EXPECT_NEAR(geometry::intersectionArea(rect(0, 0, 2, 2), rect(2, 0, 4, 2),
                                         0.05),
              0.0, 1e-9);
  EXPECT_NEAR(geometry::intersectionArea(rect(0, 0, 2, 2), rect(0, 2, 2, 4),
                                         0.05),
              0.0, 1e-9);
}

TEST(RegionGeometry, ContactLengthAlongSharedEdge) {
  EXPECT_NEAR(geometry::contactLength(rect(0, 0, 2, 2), rect(2.05, 0, 4, 2),
                                      0.1, 0.05),
              2.0, 0.15);
  EXPECT_EQ(geometry::contactLength(rect(0, 0, 2, 2), rect(3, 0, 4, 2), 0.1,
                                    0.05),
            0.0);
}

TEST(Hungarian, PrefersGloballyConsistentAssignmentOverGreedy) {
  const double inf = std::numeric_limits<double>::infinity();
  // Greedy would take (0,0) at cost 0.1 and leave row 1 unassigned.
  const std::vector<std::vector<double>> cost = {{0.1, 0.6}, {0.6, inf}};
  const std::vector<int> a = solveAssignment(cost);
  ASSERT_EQ(a.size(), 2u);
  EXPECT_EQ(a[0], 1);
  EXPECT_EQ(a[1], 0);
}

TEST(Hungarian, MinimizesCostAndHandlesRectangularAndInfeasible) {
  const double inf = std::numeric_limits<double>::infinity();
  {
    const std::vector<std::vector<double>> cost = {
        {0.9, 0.1, 0.5}, {0.2, 0.8, 0.9}};
    const std::vector<int> a = solveAssignment(cost);
    EXPECT_EQ(a, (std::vector<int>{1, 0}));
  }
  {
    const std::vector<std::vector<double>> cost = {{0.3}, {0.1}, {0.2}};
    const std::vector<int> a = solveAssignment(cost);
    EXPECT_EQ(a, (std::vector<int>{-1, 0, -1}));
  }
  {
    const std::vector<std::vector<double>> cost = {{inf, inf}, {inf, 0.4}};
    const std::vector<int> a = solveAssignment(cost);
    EXPECT_EQ(a, (std::vector<int>{-1, 1}));
  }
  EXPECT_TRUE(solveAssignment({}).empty());
}
