#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace inc_dude {

struct Point2 {
  double x{0.0};
  double y{0.0};
};

// Simple polygon in map coordinates (meters), counter-clockwise, not closed
// (the last vertex is implicitly connected to the first).
using Polygon2 = std::vector<Point2>;

struct BoundingBox {
  double min_x{0.0};
  double min_y{0.0};
  double max_x{0.0};
  double max_y{0.0};
};

// One region of a single decomposition update. `local_id` is only meaningful
// inside that update (index into Stable_graph::Region_contour).
struct Region2D {
  int local_id{-1};
  Point2 centroid;
  Polygon2 polygon;
  double area{0.0};                     // m^2
  std::vector<int> adjacent_local_ids;  // sorted, unique, frame-local
};

// A complete decomposition update handed to the tracker.
struct RegionUpdate {
  std::string frame_id;
  int64_t stamp_ns{0};
  double resolution{0.0};  // map resolution in meters per cell
  std::vector<Region2D> regions;
};

enum class TrackStatus : uint8_t { kActive = 1, kMissing = 2 };

struct TrackedRegion {
  uint64_t id{0};
  TrackStatus status{TrackStatus::kActive};
  Region2D geometry;  // last observed geometry; local_id valid only if observed
  BoundingBox bbox;
  std::vector<uint64_t> adjacent_ids;  // canonical, sorted, unique
  uint64_t first_seen_update{0};
  uint64_t last_seen_update{0};
  int64_t last_seen_stamp_ns{0};
  uint32_t age{0};             // accepted updates since creation
  uint32_t hits{0};            // accepted updates in which it was observed
  uint32_t missed_updates{0};  // consecutive accepted updates without a match
  double last_match_score{0.0};
  // Lineage from the most recent accepted update only.
  std::vector<uint64_t> split_from;
  std::vector<uint64_t> merged_ids;
};

enum class TrackEventType : uint8_t {
  kCreated = 0,
  kSplit = 1,    // id = parent, related = new children split off it
  kMerged = 2,   // id = surviving track, related = absorbed tracks
  kMissing = 3,  // track not observed for the first time
  kRecovered = 4,
  kRemoved = 5,  // see reason: "expired", "merged", "superseded"
};

struct TrackEvent {
  TrackEventType type{TrackEventType::kCreated};
  uint64_t id{0};
  std::vector<uint64_t> related_ids;
  std::string reason;
};

// Per-candidate matching evidence, recorded for debugging.
struct MatchCandidate {
  uint64_t track_id{0};
  int local_id{-1};
  double iou{0.0};
  double containment{0.0};  // intersection / min(area_track, area_region)
  double centroid_distance{0.0};
  double area_ratio{0.0};  // min(area) / max(area)
  double score{0.0};
  bool gated{false};
  bool assigned{false};
};

struct TrackerUpdateResult {
  bool accepted{false};
  bool forced{false};  // accepted despite failing the plausibility check
  std::string rejection_reason;
  uint64_t update_index{0};  // index of this accepted update (1-based)
  std::vector<int> dropped_local_ids;  // degenerate regions ignored
  std::vector<std::pair<int, uint64_t>> local_to_canonical;
  std::vector<uint64_t> matched_ids;
  std::vector<uint64_t> new_ids;
  std::vector<uint64_t> missing_ids;
  std::vector<uint64_t> removed_ids;
  std::vector<TrackEvent> events;
  std::vector<MatchCandidate> candidates;
};

}  // namespace inc_dude
