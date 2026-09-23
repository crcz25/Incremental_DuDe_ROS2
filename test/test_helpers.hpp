#pragma once

#include <algorithm>
#include <string>
#include <vector>

#include "inc_dude/region_geometry.hpp"
#include "inc_dude/region_tracker.hpp"

namespace inc_dude {
namespace test {

inline Polygon2 rect(double x0, double y0, double x1, double y1) {
  return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
}

inline Region2D region(int local_id, const Polygon2 &polygon,
                       std::vector<int> adjacent = {}) {
  Region2D r;
  r.local_id = local_id;
  r.polygon = polygon;
  r.area = geometry::polygonArea(polygon);
  r.centroid = geometry::polygonCentroid(polygon);
  std::sort(adjacent.begin(), adjacent.end());
  r.adjacent_local_ids = adjacent;
  return r;
}

inline RegionUpdate update(std::vector<Region2D> regions,
                           const std::string &frame = "map",
                           double resolution = 0.05) {
  RegionUpdate u;
  u.frame_id = frame;
  u.resolution = resolution;
  u.regions = std::move(regions);
  return u;
}

// Canonical id assigned to a frame-local region in an accepted update.
inline uint64_t canonicalOf(const TrackerUpdateResult &result, int local_id) {
  for (const auto &[local, canonical] : result.local_to_canonical) {
    if (local == local_id) {
      return canonical;
    }
  }
  return 0;
}

inline bool hasEvent(const TrackerUpdateResult &result, TrackEventType type,
                     uint64_t id, const std::vector<uint64_t> &related = {},
                     const std::string &reason = "") {
  for (const TrackEvent &e : result.events) {
    if (e.type == type && e.id == id &&
        (related.empty() || e.related_ids == related) &&
        (reason.empty() || e.reason == reason)) {
      return true;
    }
  }
  return false;
}

}  // namespace test
}  // namespace inc_dude
