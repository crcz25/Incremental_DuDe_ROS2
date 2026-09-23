#pragma once

#include <string>
#include <vector>

#include <opencv2/core/types.hpp>

#include "inc_dude/region_types.hpp"

namespace inc_dude {

// Placement of the occupancy grid the decomposition was computed on
// (nav_msgs/MapMetaData, restricted to a planar origin).
struct GridGeometry {
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_yaw{0.0};
  double resolution{0.0};
};

struct AdapterConfig {
  // Two regions are adjacent when their boundaries come within this many
  // cells of each other. 2 cells matches the contact threshold used by
  // Incremental_Decomposer::are_contours_connected().
  double adjacency_distance_cells{2.0};
  // ...along at least this much boundary (meters). Rejects corner contacts.
  double adjacency_min_contact_m{0.25};
};

// Cell (col, row) of the grid -> map coordinates of the cell center.
Point2 gridToMap(const GridGeometry &grid, double col, double row);

// Converts the frame-local contours of Stable_graph::Region_contour (pixel
// coordinates of the unflipped occupancy image: x = column, y = row) into
// map-frame regions. The local id of each region is its contour index.
// Contours with fewer than three distinct vertices are skipped.
std::vector<Region2D>
regionsFromContours(const std::vector<std::vector<cv::Point>> &contours,
                    const GridGeometry &grid, const AdapterConfig &config);

// Fills Region2D::adjacent_local_ids from boundary contact.
void computeAdjacency(std::vector<Region2D> &regions, double tolerance_m,
                      double min_contact_m, double sample_step_m);

}  // namespace inc_dude
