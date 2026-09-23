#include "inc_dude/decomposition_adapter.hpp"

#include <algorithm>
#include <cmath>

#include "inc_dude/region_geometry.hpp"

namespace inc_dude {

Point2 gridToMap(const GridGeometry &grid, double col, double row) {
  const double lx = (col + 0.5) * grid.resolution;
  const double ly = (row + 0.5) * grid.resolution;
  const double c = std::cos(grid.origin_yaw);
  const double s = std::sin(grid.origin_yaw);
  return Point2{grid.origin_x + c * lx - s * ly,
                grid.origin_y + s * lx + c * ly};
}

std::vector<Region2D>
regionsFromContours(const std::vector<std::vector<cv::Point>> &contours,
                    const GridGeometry &grid, const AdapterConfig &config) {
  std::vector<Region2D> regions;
  regions.reserve(contours.size());
  for (size_t i = 0; i < contours.size(); ++i) {
    Polygon2 polygon;
    polygon.reserve(contours[i].size());
    for (const cv::Point &p : contours[i]) {
      polygon.push_back(gridToMap(grid, p.x, p.y));
    }
    polygon = geometry::normalizePolygon(polygon);
    if (polygon.size() < 3) {
      continue;
    }
    Region2D region;
    region.local_id = static_cast<int>(i);
    region.area = geometry::polygonArea(polygon);
    region.centroid = geometry::polygonCentroid(polygon);
    region.polygon = std::move(polygon);
    regions.push_back(std::move(region));
  }
  computeAdjacency(regions, config.adjacency_distance_cells * grid.resolution,
                   config.adjacency_min_contact_m, grid.resolution);
  return regions;
}

void computeAdjacency(std::vector<Region2D> &regions, double tolerance_m,
                      double min_contact_m, double sample_step_m) {
  for (Region2D &r : regions) {
    r.adjacent_local_ids.clear();
  }
  std::vector<BoundingBox> boxes;
  boxes.reserve(regions.size());
  for (const Region2D &r : regions) {
    boxes.push_back(geometry::boundingBox(r.polygon));
  }
  for (size_t i = 0; i < regions.size(); ++i) {
    for (size_t j = i + 1; j < regions.size(); ++j) {
      if (!geometry::boxesOverlap(boxes[i], boxes[j], tolerance_m)) {
        continue;
      }
      // Symmetric: a short edge of one region may lie along a long edge of
      // the other, so take the larger of the two one-sided contacts.
      const double contact = std::max(
          geometry::contactLength(regions[i].polygon, regions[j].polygon,
                                  tolerance_m, sample_step_m),
          geometry::contactLength(regions[j].polygon, regions[i].polygon,
                                  tolerance_m, sample_step_m));
      if (contact > 0.0 && contact >= min_contact_m) {
        regions[i].adjacent_local_ids.push_back(regions[j].local_id);
        regions[j].adjacent_local_ids.push_back(regions[i].local_id);
      }
    }
  }
  for (Region2D &r : regions) {
    std::sort(r.adjacent_local_ids.begin(), r.adjacent_local_ids.end());
  }
}

}  // namespace inc_dude
