#pragma once

#include "inc_dude/region_types.hpp"

namespace inc_dude {
namespace geometry {

bool isFinite(const Point2 &p);
bool isFinite(const Polygon2 &polygon);

// Absolute shoelace area.
double polygonArea(const Polygon2 &polygon);
double signedArea(const Polygon2 &polygon);

// Area centroid; falls back to the vertex mean for (near) zero-area input.
Point2 polygonCentroid(const Polygon2 &polygon);

BoundingBox boundingBox(const Polygon2 &polygon);
bool boxesOverlap(const BoundingBox &a, const BoundingBox &b,
                  double margin = 0.0);

// Removes consecutive duplicate vertices (and a duplicated closing vertex)
// and orients the polygon counter-clockwise.
Polygon2 normalizePolygon(const Polygon2 &polygon);

double distance(const Point2 &a, const Point2 &b);
double pointSegmentDistance(const Point2 &p, const Point2 &a, const Point2 &b);

// Area estimates computed with horizontal scanlines spaced `step` meters on a
// global grid, using the even-odd rule with half-open crossings. Adjacent
// polygons that share an edge therefore do not double-count it, so overlap
// between two pieces of a partition is (close to) zero. The horizontal
// extent of every scanline is exact; the vertical error is bounded by `step`.
double scanlineArea(const Polygon2 &polygon, double step);
double intersectionArea(const Polygon2 &a, const Polygon2 &b, double step);

// Length of the boundary of `a` that lies within `tolerance` of the boundary
// of `b`, sampled every `step` meters.
double contactLength(const Polygon2 &a, const Polygon2 &b, double tolerance,
                     double step);

}  // namespace geometry
}  // namespace inc_dude
