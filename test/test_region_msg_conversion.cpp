// Characterization of region_msg_conversion: grid validation, Region2D and
// RegionEvent messages and the debug markers, as the code behaves today.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "inc_dude/region_msg_conversion.hpp"

using namespace inc_dude;

namespace {

nav_msgs::msg::OccupancyGrid validMap() {
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.resolution = 0.05f;
  map.info.width = 4;
  map.info.height = 3;
  map.info.origin.position.x = -1.5;
  map.info.origin.position.y = 2.0;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(12, 0);
  return map;
}

std::string rejection(const nav_msgs::msg::OccupancyGrid &map) {
  GridGeometry grid;
  std::string reason;
  EXPECT_FALSE(gridGeometryFromMap(map, grid, reason));
  return reason;
}

TrackedRegion track(uint64_t id, TrackStatus status) {
  TrackedRegion t;
  t.id = id;
  t.status = status;
  t.geometry.local_id = 7;
  t.geometry.centroid = {1.0, 2.0};
  t.geometry.polygon = {{0.0, 0.0}, {2.0, 0.0}, {2.0, 4.0}};
  t.geometry.area = 4.0;
  t.adjacent_ids = {3, 9};
  t.first_seen_update = 2;
  t.last_seen_update = 5;
  t.age = 4;
  t.hits = 3;
  t.missed_updates = 1;
  t.split_from = {11};
  t.merged_ids = {12, 13};
  return t;
}

msg::Region2D regionMsg(uint64_t id, double x, double y,
                        std::vector<uint64_t> adjacent,
                        uint8_t status = msg::Region2D::STATUS_ACTIVE) {
  msg::Region2D r;
  r.id = id;
  r.status = status;
  r.centroid.x = x;
  r.centroid.y = y;
  r.adjacent_ids = std::move(adjacent);
  r.source_region_index =
      status == msg::Region2D::STATUS_ACTIVE ? static_cast<int32_t>(id) : -1;
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  r.polygon = {p, p, p};
  r.polygon[1].x += 1.0;
  r.polygon[2].y += 1.0;
  return r;
}

}  // namespace

TEST(GridGeometryFromMap, AcceptsAValidGridAndExtractsItsPlacement) {
  auto map = validMap();
  // 90 degrees about z.
  map.info.origin.orientation.w = std::sqrt(0.5);
  map.info.origin.orientation.z = std::sqrt(0.5);
  GridGeometry grid;
  std::string reason;
  ASSERT_TRUE(gridGeometryFromMap(map, grid, reason)) << reason;
  EXPECT_TRUE(reason.empty());
  EXPECT_DOUBLE_EQ(grid.origin_x, -1.5);
  EXPECT_DOUBLE_EQ(grid.origin_y, 2.0);
  EXPECT_NEAR(grid.origin_yaw, M_PI / 2.0, 1e-12);
  EXPECT_DOUBLE_EQ(grid.resolution, 0.05f);
}

TEST(GridGeometryFromMap, RejectsInvalidGridsWithAReason) {
  auto map = validMap();
  map.header.frame_id.clear();
  EXPECT_EQ(rejection(map), "empty frame_id");

  map = validMap();
  map.info.resolution = 0.0f;
  EXPECT_EQ(rejection(map), "invalid resolution");
  map.info.resolution = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(rejection(map), "invalid resolution");

  map = validMap();
  map.data.pop_back();
  EXPECT_EQ(rejection(map), "data size 11 does not match 4x3");
  map = validMap();
  map.info.width = 0;
  map.data.clear();
  EXPECT_EQ(rejection(map), "data size 0 does not match 0x3");

  map = validMap();
  map.info.origin.position.z = std::numeric_limits<double>::infinity();
  EXPECT_EQ(rejection(map), "non-finite origin");

  map = validMap();
  map.info.origin.orientation.w = 0.5;
  EXPECT_EQ(rejection(map), "origin orientation is not a unit quaternion");

  map = validMap();
  map.info.origin.orientation.w = std::sqrt(0.5);
  map.info.origin.orientation.x = std::sqrt(0.5);
  EXPECT_EQ(rejection(map), "origin orientation is not planar");
}

TEST(RegionMsgConversion, ActiveTrackCopiesEveryFieldAndItsLocalIndex) {
  const msg::Region2D m = toMsg(track(42, TrackStatus::kActive));
  EXPECT_EQ(m.id, 42u);
  EXPECT_EQ(m.status, msg::Region2D::STATUS_ACTIVE);
  EXPECT_DOUBLE_EQ(m.centroid.x, 1.0);
  EXPECT_DOUBLE_EQ(m.centroid.y, 2.0);
  EXPECT_DOUBLE_EQ(m.centroid.z, 0.0);
  ASSERT_EQ(m.polygon.size(), 3u);
  EXPECT_DOUBLE_EQ(m.polygon[2].x, 2.0);
  EXPECT_DOUBLE_EQ(m.polygon[2].y, 4.0);
  EXPECT_DOUBLE_EQ(m.polygon[2].z, 0.0);
  EXPECT_DOUBLE_EQ(m.area, 4.0);
  EXPECT_EQ(m.adjacent_ids, (std::vector<uint64_t>{3, 9}));
  EXPECT_EQ(m.first_seen_update, 2u);
  EXPECT_EQ(m.last_seen_update, 5u);
  EXPECT_EQ(m.age, 4u);
  EXPECT_EQ(m.hits, 3u);
  EXPECT_EQ(m.missed_updates, 1u);
  EXPECT_EQ(m.source_region_index, 7);
  EXPECT_EQ(m.split_from, (std::vector<uint64_t>{11}));
  EXPECT_EQ(m.merged_ids, (std::vector<uint64_t>{12, 13}));
}

TEST(RegionMsgConversion, MissingTrackHasNoSourceRegionIndex) {
  const msg::Region2D m = toMsg(track(42, TrackStatus::kMissing));
  EXPECT_EQ(m.status, msg::Region2D::STATUS_MISSING);
  EXPECT_EQ(m.source_region_index, -1);
}

TEST(RegionMsgConversion, EventTypesMatchTheMessageConstants) {
  const std::vector<std::pair<TrackEventType, uint8_t>> types = {
      {TrackEventType::kCreated, msg::RegionEvent::CREATED},
      {TrackEventType::kSplit, msg::RegionEvent::SPLIT},
      {TrackEventType::kMerged, msg::RegionEvent::MERGED},
      {TrackEventType::kMissing, msg::RegionEvent::MISSING},
      {TrackEventType::kRecovered, msg::RegionEvent::RECOVERED},
      {TrackEventType::kRemoved, msg::RegionEvent::REMOVED}};
  for (const auto &[type, expected] : types) {
    TrackEvent e;
    e.type = type;
    e.id = 5;
    e.related_ids = {6, 7};
    e.reason = "merged";
    const msg::RegionEvent m = toMsg(e);
    EXPECT_EQ(m.type, expected);
    EXPECT_EQ(m.id, 5u);
    EXPECT_EQ(m.related_ids, (std::vector<uint64_t>{6, 7}));
    EXPECT_EQ(m.reason, "merged");
  }
}

TEST(RegionMsgConversion, ArrayKeepsHeaderIndexAndOrder) {
  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp.sec = 12;
  TrackEvent e;
  e.type = TrackEventType::kSplit;
  e.id = 2;
  const msg::Region2DArray a =
      toMsg({track(9, TrackStatus::kActive), track(2, TrackStatus::kMissing)},
            {e}, header, 17);
  EXPECT_EQ(a.header.frame_id, "map");
  EXPECT_EQ(a.header.stamp.sec, 12);
  EXPECT_EQ(a.update_index, 17u);
  ASSERT_EQ(a.regions.size(), 2u);
  EXPECT_EQ(a.regions[0].id, 9u);
  EXPECT_EQ(a.regions[1].id, 2u);
  ASSERT_EQ(a.events.size(), 1u);
  EXPECT_EQ(a.events[0].type, msg::RegionEvent::SPLIT);
}

TEST(RegionMarkers, StartWithDeleteAllThenOutlineAndLabelPerRegionThenEdges) {
  msg::Region2DArray regions;
  regions.header.frame_id = "map";
  regions.regions = {
      regionMsg(1, 0.0, 0.0, {2, 3}),
      regionMsg(2, 4.0, 0.0, {1}, msg::Region2D::STATUS_MISSING),
  };
  const auto out = toMarkers(regions);
  using visualization_msgs::msg::Marker;
  ASSERT_EQ(out.markers.size(), 1u + 2u * 2u + 1u);
  EXPECT_EQ(out.markers.front().action, Marker::DELETEALL);
  EXPECT_EQ(out.markers.front().header.frame_id, "map");

  const Marker &outline = out.markers[1];
  EXPECT_EQ(outline.ns, "outline");
  EXPECT_EQ(outline.id, 1);
  EXPECT_EQ(outline.type, Marker::LINE_STRIP);
  // Closed: the first vertex is repeated at the end.
  ASSERT_EQ(outline.points.size(), 4u);
  EXPECT_EQ(outline.points.back(), outline.points.front());
  EXPECT_DOUBLE_EQ(outline.scale.x, 0.05);
  EXPECT_FLOAT_EQ(outline.color.a, 1.0f);

  const Marker &label = out.markers[2];
  EXPECT_EQ(label.ns, "label");
  EXPECT_EQ(label.type, Marker::TEXT_VIEW_FACING);
  EXPECT_EQ(label.text, "<1,1>");
  EXPECT_DOUBLE_EQ(label.pose.position.z, 0.2);

  const Marker &missing_outline = out.markers[3];
  EXPECT_DOUBLE_EQ(missing_outline.scale.x, 0.02);
  EXPECT_FLOAT_EQ(missing_outline.color.a, 0.4f);
  EXPECT_EQ(out.markers[4].text, "<-,2>?");

  // One edge 1-2: drawn once (from the lower id), and none to the absent 3.
  const Marker &edges = out.markers.back();
  EXPECT_EQ(edges.ns, "adjacency");
  EXPECT_EQ(edges.type, Marker::LINE_LIST);
  ASSERT_EQ(edges.points.size(), 2u);
  EXPECT_DOUBLE_EQ(edges.points[0].x, 0.0);
  EXPECT_DOUBLE_EQ(edges.points[1].x, 4.0);
}

TEST(RegionMarkers, EmptyArrayGivesDeleteAllAndAnEmptyEdgeList) {
  const auto out = toMarkers(msg::Region2DArray{});
  ASSERT_EQ(out.markers.size(), 2u);
  EXPECT_EQ(out.markers[0].action, visualization_msgs::msg::Marker::DELETEALL);
  EXPECT_TRUE(out.markers[1].points.empty());
}
