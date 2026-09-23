#include "inc_dude/region_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace inc_dude {
namespace geometry {

namespace {

// Scanlines sit at (j + kScanOffset) * step on a global grid so that every
// polygon is sampled on the same lines. The offset keeps the lines away from
// the half-cell lattice on which occupancy-grid derived vertices lie.
constexpr double kScanOffset = 0.25;

using Intervals = std::vector<std::pair<double, double>>;

void scanlineIntervals(const Polygon2 &polygon, double y, Intervals &out,
                       std::vector<double> &crossings) {
  out.clear();
  crossings.clear();
  const size_t n = polygon.size();
  for (size_t i = 0; i < n; ++i) {
    const Point2 &a = polygon[i];
    const Point2 &b = polygon[(i + 1) % n];
    // Half-open rule: an edge covers [min_y, max_y).
    const bool crosses = (a.y <= y && y < b.y) || (b.y <= y && y < a.y);
    if (!crosses) {
      continue;
    }
    crossings.push_back(a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y));
  }
  std::sort(crossings.begin(), crossings.end());
  for (size_t i = 0; i + 1 < crossings.size(); i += 2) {
    out.emplace_back(crossings[i], crossings[i + 1]);
  }
}

bool rowRange(double min_y, double max_y, double step, long &first,
              long &last) {
  first = static_cast<long>(std::ceil(min_y / step - kScanOffset));
  last = static_cast<long>(std::floor(max_y / step - kScanOffset));
  return first <= last;
}

double overlapLength(const Intervals &a, const Intervals &b) {
  double length = 0.0;
  size_t i = 0;
  size_t j = 0;
  while (i < a.size() && j < b.size()) {
    const double lo = std::max(a[i].first, b[j].first);
    const double hi = std::min(a[i].second, b[j].second);
    if (hi > lo) {
      length += hi - lo;
    }
    if (a[i].second < b[j].second) {
      ++i;
    } else {
      ++j;
    }
  }
  return length;
}

}  // namespace

bool isFinite(const Point2 &p) { return std::isfinite(p.x) && std::isfinite(p.y); }

bool isFinite(const Polygon2 &polygon) {
  return std::all_of(polygon.begin(), polygon.end(),
                     [](const Point2 &p) { return isFinite(p); });
}

double signedArea(const Polygon2 &polygon) {
  const size_t n = polygon.size();
  if (n < 3) {
    return 0.0;
  }
  double twice_area = 0.0;
  for (size_t i = 0; i < n; ++i) {
    const Point2 &a = polygon[i];
    const Point2 &b = polygon[(i + 1) % n];
    twice_area += a.x * b.y - b.x * a.y;
  }
  return 0.5 * twice_area;
}

double polygonArea(const Polygon2 &polygon) {
  return std::abs(signedArea(polygon));
}

Point2 polygonCentroid(const Polygon2 &polygon) {
  Point2 mean;
  if (polygon.empty()) {
    return mean;
  }
  // Work relative to the first vertex to limit cancellation for map
  // coordinates far from the origin.
  const Point2 ref = polygon.front();
  const size_t n = polygon.size();
  double twice_area = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  for (size_t i = 0; i < n; ++i) {
    const double ax = polygon[i].x - ref.x;
    const double ay = polygon[i].y - ref.y;
    const double bx = polygon[(i + 1) % n].x - ref.x;
    const double by = polygon[(i + 1) % n].y - ref.y;
    const double cross = ax * by - bx * ay;
    twice_area += cross;
    cx += (ax + bx) * cross;
    cy += (ay + by) * cross;
    mean.x += polygon[i].x;
    mean.y += polygon[i].y;
  }
  mean.x /= static_cast<double>(n);
  mean.y /= static_cast<double>(n);
  if (std::abs(twice_area) < 1e-12) {
    return mean;
  }
  return Point2{ref.x + cx / (3.0 * twice_area),
                ref.y + cy / (3.0 * twice_area)};
}

BoundingBox boundingBox(const Polygon2 &polygon) {
  BoundingBox box;
  if (polygon.empty()) {
    return box;
  }
  box.min_x = box.max_x = polygon.front().x;
  box.min_y = box.max_y = polygon.front().y;
  for (const Point2 &p : polygon) {
    box.min_x = std::min(box.min_x, p.x);
    box.max_x = std::max(box.max_x, p.x);
    box.min_y = std::min(box.min_y, p.y);
    box.max_y = std::max(box.max_y, p.y);
  }
  return box;
}

bool boxesOverlap(const BoundingBox &a, const BoundingBox &b, double margin) {
  return a.min_x <= b.max_x + margin && b.min_x <= a.max_x + margin &&
         a.min_y <= b.max_y + margin && b.min_y <= a.max_y + margin;
}

Polygon2 normalizePolygon(const Polygon2 &polygon) {
  Polygon2 out;
  out.reserve(polygon.size());
  for (const Point2 &p : polygon) {
    if (out.empty() || out.back().x != p.x || out.back().y != p.y) {
      out.push_back(p);
    }
  }
  while (out.size() > 1 && out.front().x == out.back().x &&
         out.front().y == out.back().y) {
    out.pop_back();
  }
  if (signedArea(out) < 0.0) {
    std::reverse(out.begin(), out.end());
  }
  return out;
}

double distance(const Point2 &a, const Point2 &b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

double pointSegmentDistance(const Point2 &p, const Point2 &a, const Point2 &b) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double len2 = dx * dx + dy * dy;
  if (len2 <= 0.0) {
    return distance(p, a);
  }
  double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2;
  t = std::clamp(t, 0.0, 1.0);
  return distance(p, Point2{a.x + t * dx, a.y + t * dy});
}

double scanlineArea(const Polygon2 &polygon, double step) {
  if (polygon.size() < 3 || !(step > 0.0)) {
    return 0.0;
  }
  const BoundingBox box = boundingBox(polygon);
  long first = 0;
  long last = 0;
  if (!rowRange(box.min_y, box.max_y, step, first, last)) {
    return 0.0;
  }
  Intervals intervals;
  std::vector<double> crossings;
  double length = 0.0;
  for (long j = first; j <= last; ++j) {
    scanlineIntervals(polygon, (static_cast<double>(j) + kScanOffset) * step,
                      intervals, crossings);
    for (const auto &iv : intervals) {
      length += iv.second - iv.first;
    }
  }
  return length * step;
}

double intersectionArea(const Polygon2 &a, const Polygon2 &b, double step) {
  if (a.size() < 3 || b.size() < 3 || !(step > 0.0)) {
    return 0.0;
  }
  const BoundingBox box_a = boundingBox(a);
  const BoundingBox box_b = boundingBox(b);
  if (!boxesOverlap(box_a, box_b)) {
    return 0.0;
  }
  long first = 0;
  long last = 0;
  if (!rowRange(std::max(box_a.min_y, box_b.min_y),
                std::min(box_a.max_y, box_b.max_y), step, first, last)) {
    return 0.0;
  }
  Intervals ia;
  Intervals ib;
  std::vector<double> crossings;
  double length = 0.0;
  for (long j = first; j <= last; ++j) {
    const double y = (static_cast<double>(j) + kScanOffset) * step;
    scanlineIntervals(a, y, ia, crossings);
    if (ia.empty()) {
      continue;
    }
    scanlineIntervals(b, y, ib, crossings);
    length += overlapLength(ia, ib);
  }
  return length * step;
}

double contactLength(const Polygon2 &a, const Polygon2 &b, double tolerance,
                     double step) {
  if (a.size() < 2 || b.size() < 2 || !(step > 0.0)) {
    return 0.0;
  }
  const BoundingBox box_b = boundingBox(b);
  if (!boxesOverlap(boundingBox(a), box_b, tolerance)) {
    return 0.0;
  }
  const size_t na = a.size();
  const size_t nb = b.size();
  double contact = 0.0;
  for (size_t i = 0; i < na; ++i) {
    const Point2 &p0 = a[i];
    const Point2 &p1 = a[(i + 1) % na];
    const double len = distance(p0, p1);
    if (len <= 0.0) {
      continue;
    }
    const int samples = std::max(1, static_cast<int>(std::ceil(len / step)));
    const double piece = len / samples;
    for (int k = 0; k < samples; ++k) {
      const double t = (k + 0.5) / samples;
      const Point2 s{p0.x + t * (p1.x - p0.x), p0.y + t * (p1.y - p0.y)};
      if (s.x < box_b.min_x - tolerance || s.x > box_b.max_x + tolerance ||
          s.y < box_b.min_y - tolerance || s.y > box_b.max_y + tolerance) {
        continue;
      }
      for (size_t e = 0; e < nb; ++e) {
        if (pointSegmentDistance(s, b[e], b[(e + 1) % nb]) <= tolerance) {
          contact += piece;
          break;
        }
      }
    }
  }
  return contact;
}

}  // namespace geometry
}  // namespace inc_dude
