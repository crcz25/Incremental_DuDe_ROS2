#pragma once

#include <string>
#include <vector>

#include "inc_dude/decomposition_adapter.hpp"
#include "inc_dude/msg/region2_d_array.hpp"
#include "inc_dude/region_types.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace inc_dude {

// Validates frame, resolution, size and origin of an occupancy grid and
// extracts its planar placement. Returns false with a reason otherwise.
bool gridGeometryFromMap(const nav_msgs::msg::OccupancyGrid &map,
                         GridGeometry &grid, std::string &reason);

msg::Region2D toMsg(const TrackedRegion &track);
msg::RegionEvent toMsg(const TrackEvent &event);

msg::Region2DArray toMsg(const std::vector<TrackedRegion> &tracks,
                         const std::vector<TrackEvent> &events,
                         const std_msgs::msg::Header &header,
                         uint64_t update_index);

// Debug visualization: region outlines, canonical-ID labels and adjacency
// edges. Starts with a DELETEALL marker so stale regions disappear.
visualization_msgs::msg::MarkerArray
toMarkers(const msg::Region2DArray &regions);

}  // namespace inc_dude
