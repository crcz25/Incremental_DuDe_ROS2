#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "inc_dude/region_types.hpp"

namespace inc_dude {

struct TrackerConfig {
  // Regions smaller than this are ignored. 0.25 m^2 = safety_distance^2, the
  // smallest new-space contour Incremental_Decomposer itself decomposes.
  double min_region_area_m2{0.25};
  // Scanline spacing for overlap areas; <= 0 uses the update's resolution.
  double overlap_sample_step_m{0.0};

  // Gating: a (track, region) pair is a match candidate when the overlap
  // covers at least `gate_min_containment` of the smaller of the two, or,
  // failing that, when the centroids are close and the areas are similar.
  double gate_min_containment{0.5};
  double gate_centroid_distance_m{0.5};
  double gate_min_area_ratio{0.5};

  // Match score = weighted mean of IoU, containment, centroid proximity
  // exp(-d / centroid_distance_scale_m) and area ratio.
  double weight_iou{0.5};
  double weight_containment{0.2};
  double weight_centroid{0.2};
  double weight_area{0.1};
  double centroid_distance_scale_m{1.0};

  // A new region lying at least this fraction inside an existing track is a
  // split of it; an unmatched track lying at least this fraction inside a
  // matched region was merged into it.
  double split_merge_min_fraction{0.5};
  // Unmatched tracks whose area is covered at least this much by the
  // current regions are retired (their space is now described by other
  // regions) instead of being kept as missing.
  double retire_min_coverage{0.5};
  // Unmatched, uncovered tracks survive this many consecutive updates.
  int max_missed_updates{5};

  // Plausibility checks. An update whose total region area shrinks by more
  // than `max_area_loss_fraction`, or whose regions overlap each other by
  // more than `max_region_overlap_fraction` of the smaller one, is rejected
  // as incomplete/corrupt, up to `max_consecutive_rejections` times in a row
  // (after that the map is assumed to have genuinely changed).
  double max_area_loss_fraction{0.5};
  double max_region_overlap_fraction{0.25};
  int max_consecutive_rejections{3};
};

// Returns an empty string for a valid configuration, otherwise the reason.
std::string validateConfig(const TrackerConfig &config);

// Assigns persistent canonical IDs to the frame-local regions of successive
// decomposition updates. Canonical IDs are allocated from a monotonically
// increasing counter and are never reused.
class RegionTracker {
public:
  explicit RegionTracker(const TrackerConfig &config = TrackerConfig());

  // Processes one decomposition update. A rejected update leaves the tracking
  // state untouched.
  TrackerUpdateResult update(const RegionUpdate &update);

  // Tracks in canonical-ID order. Missing tracks are included only on
  // request; adjacency is restricted to the returned set.
  std::vector<TrackedRegion> publishableTracks(bool include_missing) const;

  const std::map<uint64_t, TrackedRegion> &tracks() const { return tracks_; }
  const TrackerConfig &config() const { return config_; }
  const std::string &frameId() const { return frame_id_; }
  uint64_t acceptedUpdates() const { return accepted_updates_; }
  uint64_t rejectedUpdates() const { return rejected_updates_; }
  uint64_t nextId() const { return next_id_; }

private:
  struct PreparedRegion {
    Region2D region;
    BoundingBox bbox;
    double scan_area{0.0};
  };

  bool validate(const RegionUpdate &update, std::vector<PreparedRegion> &out,
                TrackerUpdateResult &result, double step) const;
  std::string plausibilityFailure(const std::vector<PreparedRegion> &regions,
                                  double step) const;
  TrackerUpdateResult reject(TrackerUpdateResult result,
                             const std::string &reason);

  TrackerConfig config_;
  std::map<uint64_t, TrackedRegion> tracks_;
  std::string frame_id_;
  uint64_t next_id_{1};
  uint64_t accepted_updates_{0};
  uint64_t rejected_updates_{0};
  int consecutive_soft_rejections_{0};
  double last_total_area_{0.0};
};

// Human-readable, multi-line summary of an update for debug logging.
std::string formatUpdateReport(const TrackerUpdateResult &result,
                               bool include_candidates);

const char *toString(TrackEventType type);

}  // namespace inc_dude
