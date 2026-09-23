#include "inc_dude/region_msg_conversion.hpp"

#include <cmath>
#include <map>
#include <string>

namespace inc_dude {

namespace {

geometry_msgs::msg::Point toPoint(const Point2 &p) {
  geometry_msgs::msg::Point out;
  out.x = p.x;
  out.y = p.y;
  out.z = 0.0;
  return out;
}

std_msgs::msg::ColorRGBA colorForId(uint64_t id, float alpha) {
  // Golden-ratio hue spacing: stable, distinct colors per canonical id.
  const double hue = std::fmod(static_cast<double>(id) * 0.618033988749895, 1.0);
  const double h6 = hue * 6.0;
  const double x = 1.0 - std::abs(std::fmod(h6, 2.0) - 1.0);
  double r = 0, g = 0, b = 0;
  switch (static_cast<int>(h6)) {
  case 0: r = 1; g = x; break;
  case 1: r = x; g = 1; break;
  case 2: g = 1; b = x; break;
  case 3: g = x; b = 1; break;
  case 4: r = x; b = 1; break;
  default: r = 1; b = x; break;
  }
  std_msgs::msg::ColorRGBA c;
  c.r = static_cast<float>(0.2 + 0.8 * r);
  c.g = static_cast<float>(0.2 + 0.8 * g);
  c.b = static_cast<float>(0.2 + 0.8 * b);
  c.a = alpha;
  return c;
}

}  // namespace

bool gridGeometryFromMap(const nav_msgs::msg::OccupancyGrid &map,
                         GridGeometry &grid, std::string &reason) {
  const auto &info = map.info;
  if (map.header.frame_id.empty()) {
    reason = "empty frame_id";
    return false;
  }
  if (!(std::isfinite(info.resolution) && info.resolution > 0.0f)) {
    reason = "invalid resolution";
    return false;
  }
  if (info.width == 0 || info.height == 0 ||
      map.data.size() != static_cast<size_t>(info.width) * info.height) {
    reason = "data size " + std::to_string(map.data.size()) +
             " does not match " + std::to_string(info.width) + "x" +
             std::to_string(info.height);
    return false;
  }
  const auto &p = info.origin.position;
  const auto &q = info.origin.orientation;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
      !std::isfinite(q.w)) {
    reason = "non-finite origin";
    return false;
  }
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (std::abs(norm - 1.0) > 1e-3) {
    reason = "origin orientation is not a unit quaternion";
    return false;
  }
  // Only rotations about z are meaningful for a 2D grid.
  if (std::abs(q.x) > 1e-3 * norm || std::abs(q.y) > 1e-3 * norm) {
    reason = "origin orientation is not planar";
    return false;
  }
  grid.origin_x = p.x;
  grid.origin_y = p.y;
  grid.origin_yaw = 2.0 * std::atan2(q.z, q.w);
  grid.resolution = info.resolution;
  return true;
}

msg::Region2D toMsg(const TrackedRegion &track) {
  msg::Region2D out;
  out.id = track.id;
  out.status = track.status == TrackStatus::kActive
                   ? msg::Region2D::STATUS_ACTIVE
                   : msg::Region2D::STATUS_MISSING;
  out.centroid = toPoint(track.geometry.centroid);
  out.polygon.reserve(track.geometry.polygon.size());
  for (const Point2 &p : track.geometry.polygon) {
    out.polygon.push_back(toPoint(p));
  }
  out.area = track.geometry.area;
  out.adjacent_ids = track.adjacent_ids;
  out.first_seen_update = track.first_seen_update;
  out.last_seen_update = track.last_seen_update;
  out.age = track.age;
  out.hits = track.hits;
  out.missed_updates = track.missed_updates;
  out.source_region_index = track.status == TrackStatus::kActive
                                ? track.geometry.local_id
                                : -1;
  out.split_from = track.split_from;
  out.merged_ids = track.merged_ids;
  return out;
}

msg::RegionEvent toMsg(const TrackEvent &event) {
  msg::RegionEvent out;
  out.type = static_cast<uint8_t>(event.type);
  out.id = event.id;
  out.related_ids = event.related_ids;
  out.reason = event.reason;
  return out;
}

msg::Region2DArray toMsg(const std::vector<TrackedRegion> &tracks,
                         const std::vector<TrackEvent> &events,
                         const std_msgs::msg::Header &header,
                         uint64_t update_index) {
  msg::Region2DArray out;
  out.header = header;
  out.update_index = update_index;
  out.regions.reserve(tracks.size());
  for (const TrackedRegion &t : tracks) {
    out.regions.push_back(toMsg(t));
  }
  out.events.reserve(events.size());
  for (const TrackEvent &e : events) {
    out.events.push_back(toMsg(e));
  }
  return out;
}

visualization_msgs::msg::MarkerArray
toMarkers(const msg::Region2DArray &regions) {
  using visualization_msgs::msg::Marker;
  visualization_msgs::msg::MarkerArray out;

  Marker clear;
  clear.header = regions.header;
  clear.action = Marker::DELETEALL;
  out.markers.push_back(clear);

  std::map<uint64_t, const msg::Region2D *> by_id;
  for (const msg::Region2D &r : regions.regions) {
    by_id[r.id] = &r;
  }

  Marker edges;
  edges.header = regions.header;
  edges.ns = "adjacency";
  edges.id = 0;
  edges.type = Marker::LINE_LIST;
  edges.action = Marker::ADD;
  edges.pose.orientation.w = 1.0;
  edges.scale.x = 0.03;
  edges.color.r = edges.color.g = edges.color.b = 0.1f;
  edges.color.a = 0.8f;

  for (const msg::Region2D &r : regions.regions) {
    const bool missing = r.status == msg::Region2D::STATUS_MISSING;
    const int marker_id = static_cast<int>(r.id % 0x7fffffff);

    Marker outline;
    outline.header = regions.header;
    outline.ns = "outline";
    outline.id = marker_id;
    outline.type = Marker::LINE_STRIP;
    outline.action = Marker::ADD;
    outline.pose.orientation.w = 1.0;
    outline.scale.x = missing ? 0.02 : 0.05;
    outline.color = colorForId(r.id, missing ? 0.4f : 1.0f);
    outline.points = r.polygon;
    if (!r.polygon.empty()) {
      outline.points.push_back(r.polygon.front());
    }
    out.markers.push_back(outline);

    Marker label;
    label.header = regions.header;
    label.ns = "label";
    label.id = marker_id;
    label.type = Marker::TEXT_VIEW_FACING;
    label.action = Marker::ADD;
    label.pose.position = r.centroid;
    label.pose.position.z = 0.2;
    label.pose.orientation.w = 1.0;
    label.scale.z = 0.4;
    label.color = colorForId(r.id, 1.0f);
    label.text = std::to_string(r.id) + (missing ? "?" : "");
    out.markers.push_back(label);

    for (uint64_t adj : r.adjacent_ids) {
      auto it = by_id.find(adj);
      if (adj > r.id && it != by_id.end()) {
        edges.points.push_back(r.centroid);
        edges.points.push_back(it->second->centroid);
      }
    }
  }
  out.markers.push_back(edges);
  return out;
}

}  // namespace inc_dude
