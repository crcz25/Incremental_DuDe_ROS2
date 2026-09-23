#include "inc_dude/region_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <unordered_map>

#include "inc_dude/hungarian.hpp"
#include "inc_dude/region_geometry.hpp"

namespace inc_dude {

namespace {

bool inUnitInterval(double v) { return std::isfinite(v) && v >= 0.0 && v <= 1.0; }

void sortUnique(std::vector<uint64_t> &ids) {
  std::sort(ids.begin(), ids.end());
  ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
}

}  // namespace

std::string validateConfig(const TrackerConfig &c) {
  if (!(std::isfinite(c.min_region_area_m2) && c.min_region_area_m2 >= 0.0)) {
    return "min_region_area_m2 must be finite and >= 0";
  }
  if (!std::isfinite(c.overlap_sample_step_m)) {
    return "overlap_sample_step_m must be finite";
  }
  if (!inUnitInterval(c.gate_min_containment) ||
      !inUnitInterval(c.gate_min_area_ratio) ||
      !inUnitInterval(c.split_merge_min_fraction) ||
      !inUnitInterval(c.retire_min_coverage) ||
      !inUnitInterval(c.max_area_loss_fraction) ||
      !inUnitInterval(c.max_region_overlap_fraction)) {
    return "fractions and ratios must lie in [0, 1]";
  }
  if (!(std::isfinite(c.gate_centroid_distance_m) &&
        c.gate_centroid_distance_m >= 0.0)) {
    return "gate_centroid_distance_m must be finite and >= 0";
  }
  if (!(std::isfinite(c.centroid_distance_scale_m) &&
        c.centroid_distance_scale_m > 0.0)) {
    return "centroid_distance_scale_m must be finite and > 0";
  }
  const double weights[] = {c.weight_iou, c.weight_containment,
                            c.weight_centroid, c.weight_area};
  double sum = 0.0;
  for (double w : weights) {
    if (!(std::isfinite(w) && w >= 0.0)) {
      return "score weights must be finite and >= 0";
    }
    sum += w;
  }
  if (!(sum > 0.0)) {
    return "at least one score weight must be positive";
  }
  if (c.max_missed_updates < 0 || c.max_consecutive_rejections < 0) {
    return "max_missed_updates and max_consecutive_rejections must be >= 0";
  }
  return "";
}

RegionTracker::RegionTracker(const TrackerConfig &config) : config_(config) {}

TrackerUpdateResult RegionTracker::reject(TrackerUpdateResult result,
                                          const std::string &reason) {
  ++rejected_updates_;
  result.accepted = false;
  result.rejection_reason = reason;
  return result;
}

bool RegionTracker::validate(const RegionUpdate &update,
                             std::vector<PreparedRegion> &out,
                             TrackerUpdateResult &result, double step) const {
  if (update.frame_id.empty()) {
    result.rejection_reason = "empty frame_id";
    return false;
  }
  if (!frame_id_.empty() && update.frame_id != frame_id_) {
    result.rejection_reason =
        "frame_id '" + update.frame_id + "' differs from tracked frame '" +
        frame_id_ + "'";
    return false;
  }
  if (!(std::isfinite(update.resolution) && update.resolution > 0.0)) {
    result.rejection_reason = "invalid resolution";
    return false;
  }
  if (!(std::isfinite(step) && step > 0.0)) {
    result.rejection_reason = "invalid overlap sample step";
    return false;
  }

  std::set<int> ids;
  for (const Region2D &r : update.regions) {
    if (!ids.insert(r.local_id).second) {
      result.rejection_reason =
          "duplicate local region id " + std::to_string(r.local_id);
      return false;
    }
    if (!geometry::isFinite(r.polygon) || !geometry::isFinite(r.centroid) ||
        !std::isfinite(r.area)) {
      result.rejection_reason =
          "non-finite geometry in local region " + std::to_string(r.local_id);
      return false;
    }
  }
  for (const Region2D &r : update.regions) {
    for (int adj : r.adjacent_local_ids) {
      if (adj == r.local_id || ids.count(adj) == 0) {
        result.rejection_reason = "local region " + std::to_string(r.local_id) +
                                  " has invalid adjacency to " +
                                  std::to_string(adj);
        return false;
      }
    }
  }

  // Degenerate regions are dropped individually; geometry is recomputed from
  // the polygon so the tracker never depends on caller-provided area/centroid.
  std::set<int> kept;
  for (const Region2D &r : update.regions) {
    PreparedRegion p;
    p.region.local_id = r.local_id;
    p.region.polygon = geometry::normalizePolygon(r.polygon);
    p.region.area = geometry::polygonArea(p.region.polygon);
    if (p.region.polygon.size() < 3 ||
        p.region.area < config_.min_region_area_m2 || p.region.area <= 0.0) {
      result.dropped_local_ids.push_back(r.local_id);
      continue;
    }
    p.region.centroid = geometry::polygonCentroid(p.region.polygon);
    p.region.adjacent_local_ids = r.adjacent_local_ids;
    p.bbox = geometry::boundingBox(p.region.polygon);
    p.scan_area = geometry::scanlineArea(p.region.polygon, step);
    if (!(p.scan_area > 0.0)) {
      result.dropped_local_ids.push_back(r.local_id);
      continue;
    }
    kept.insert(r.local_id);
    out.push_back(std::move(p));
  }
  for (PreparedRegion &p : out) {
    auto &adj = p.region.adjacent_local_ids;
    adj.erase(std::remove_if(adj.begin(), adj.end(),
                             [&](int id) { return kept.count(id) == 0; }),
              adj.end());
    std::sort(adj.begin(), adj.end());
    adj.erase(std::unique(adj.begin(), adj.end()), adj.end());
  }
  if (out.empty()) {
    result.rejection_reason = "update contains no valid regions";
    return false;
  }
  return true;
}

std::string
RegionTracker::plausibilityFailure(const std::vector<PreparedRegion> &regions,
                                   double step) const {
  double total_area = 0.0;
  for (const PreparedRegion &p : regions) {
    total_area += p.region.area;
  }
  if (last_total_area_ > 0.0 &&
      total_area < (1.0 - config_.max_area_loss_fraction) * last_total_area_) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2) << "total region area dropped from "
       << last_total_area_ << " to " << total_area << " m^2";
    return ss.str();
  }
  for (size_t i = 0; i < regions.size(); ++i) {
    for (size_t j = i + 1; j < regions.size(); ++j) {
      if (!geometry::boxesOverlap(regions[i].bbox, regions[j].bbox)) {
        continue;
      }
      const double inter = geometry::intersectionArea(
          regions[i].region.polygon, regions[j].region.polygon, step);
      const double smaller =
          std::min(regions[i].scan_area, regions[j].scan_area);
      if (inter > config_.max_region_overlap_fraction * smaller) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2) << "local regions "
           << regions[i].region.local_id << " and "
           << regions[j].region.local_id << " overlap by "
           << 100.0 * inter / smaller << "%";
        return ss.str();
      }
    }
  }
  return "";
}

TrackerUpdateResult RegionTracker::update(const RegionUpdate &update) {
  TrackerUpdateResult result;
  const double step = config_.overlap_sample_step_m > 0.0
                          ? config_.overlap_sample_step_m
                          : update.resolution;

  std::vector<PreparedRegion> regions;
  if (!validate(update, regions, result, step)) {
    const std::string reason = result.rejection_reason;
    return reject(std::move(result), reason);
  }

  const std::string implausible = plausibilityFailure(regions, step);
  if (!implausible.empty()) {
    if (consecutive_soft_rejections_ < config_.max_consecutive_rejections) {
      ++consecutive_soft_rejections_;
      return reject(std::move(result), implausible);
    }
    result.forced = true;
    result.rejection_reason = implausible;  // kept for reporting
  }
  consecutive_soft_rejections_ = 0;

  // ---- Accepted: from here on the tracker state is modified. ----
  const uint64_t index = ++accepted_updates_;
  result.accepted = true;
  result.update_index = index;
  if (frame_id_.empty()) {
    frame_id_ = update.frame_id;
  }

  // Tracks in canonical order (std::map) for deterministic assignment.
  std::vector<uint64_t> track_ids;
  std::vector<double> track_scan_area;
  track_ids.reserve(tracks_.size());
  for (auto &[id, track] : tracks_) {
    track.split_from.clear();
    track.merged_ids.clear();
    track_ids.push_back(id);
    track_scan_area.push_back(
        geometry::scanlineArea(track.geometry.polygon, step));
  }

  const size_t nt = track_ids.size();
  const size_t nr = regions.size();
  // Sparse intersection areas, keyed by (track index, region index).
  std::vector<std::vector<double>> inter(nt, std::vector<double>(nr, 0.0));
  std::vector<std::vector<double>> cost(
      nt, std::vector<double>(nr, std::numeric_limits<double>::infinity()));
  std::vector<std::vector<int>> candidate_index(nt, std::vector<int>(nr, -1));
  const double weight_sum = config_.weight_iou + config_.weight_containment +
                            config_.weight_centroid + config_.weight_area;

  for (size_t t = 0; t < nt; ++t) {
    const TrackedRegion &track = tracks_.at(track_ids[t]);
    for (size_t r = 0; r < nr; ++r) {
      const PreparedRegion &region = regions[r];
      if (!geometry::boxesOverlap(track.bbox, region.bbox,
                                  config_.gate_centroid_distance_m)) {
        continue;
      }
      const double a_t = track_scan_area[t];
      const double a_r = region.scan_area;
      const double i = geometry::intersectionArea(track.geometry.polygon,
                                                  region.region.polygon, step);
      inter[t][r] = i;

      MatchCandidate c;
      c.track_id = track.id;
      c.local_id = region.region.local_id;
      const double uni = a_t + a_r - i;
      c.iou = uni > 0.0 ? std::clamp(i / uni, 0.0, 1.0) : 0.0;
      const double smaller = std::min(a_t, a_r);
      c.containment = smaller > 0.0 ? std::clamp(i / smaller, 0.0, 1.0) : 0.0;
      c.centroid_distance =
          geometry::distance(track.geometry.centroid, region.region.centroid);
      const double larger = std::max(track.geometry.area, region.region.area);
      c.area_ratio =
          larger > 0.0
              ? std::min(track.geometry.area, region.region.area) / larger
              : 0.0;
      c.gated = c.containment >= config_.gate_min_containment ||
                (c.centroid_distance <= config_.gate_centroid_distance_m &&
                 c.area_ratio >= config_.gate_min_area_ratio);
      if (c.containment <= 0.0 && !c.gated) {
        continue;  // no evidence at all, do not clutter the debug output
      }
      c.score = (config_.weight_iou * c.iou +
                 config_.weight_containment * c.containment +
                 config_.weight_centroid *
                     std::exp(-c.centroid_distance /
                              config_.centroid_distance_scale_m) +
                 config_.weight_area * c.area_ratio) /
                weight_sum;
      if (c.gated) {
        cost[t][r] = 1.0 - c.score;
      }
      candidate_index[t][r] = static_cast<int>(result.candidates.size());
      result.candidates.push_back(c);
    }
  }

  const std::vector<int> assignment = solveAssignment(cost);
  std::vector<int> region_to_track(nr, -1);
  for (size_t t = 0; t < nt; ++t) {
    if (assignment[t] >= 0) {
      region_to_track[assignment[t]] = static_cast<int>(t);
      result.candidates[candidate_index[t][assignment[t]]].assigned = true;
    }
  }

  std::vector<uint64_t> region_canonical(nr, 0);

  // 1. Matched tracks take over the current geometry.
  for (size_t t = 0; t < nt; ++t) {
    const int r = assignment[t];
    if (r < 0) {
      continue;
    }
    TrackedRegion &track = tracks_.at(track_ids[t]);
    if (track.status == TrackStatus::kMissing) {
      result.events.push_back(
          {TrackEventType::kRecovered, track.id, {}, "rematched"});
    }
    track.status = TrackStatus::kActive;
    track.geometry = regions[r].region;
    track.bbox = regions[r].bbox;
    track.last_seen_update = index;
    track.last_seen_stamp_ns = update.stamp_ns;
    track.missed_updates = 0;
    ++track.hits;
    track.last_match_score =
        result.candidates[candidate_index[t][r]].score;
    region_canonical[r] = track.id;
    result.matched_ids.push_back(track.id);
  }

  // 2. Unmatched regions become new tracks; detect splits.
  std::map<uint64_t, std::vector<uint64_t>> split_children;
  for (size_t r = 0; r < nr; ++r) {
    if (region_to_track[r] >= 0) {
      continue;
    }
    int parent = -1;
    double best = 0.0;
    for (size_t t = 0; t < nt; ++t) {
      const double frac = inter[t][r] / regions[r].scan_area;
      if (frac >= config_.split_merge_min_fraction && inter[t][r] > best) {
        best = inter[t][r];
        parent = static_cast<int>(t);
      }
    }
    TrackedRegion track;
    track.id = next_id_++;
    track.status = TrackStatus::kActive;
    track.geometry = regions[r].region;
    track.bbox = regions[r].bbox;
    track.first_seen_update = index;
    track.last_seen_update = index;
    track.last_seen_stamp_ns = update.stamp_ns;
    track.hits = 1;
    TrackEvent created{TrackEventType::kCreated, track.id, {}, "new region"};
    if (parent >= 0) {
      const uint64_t parent_id = track_ids[parent];
      track.split_from.push_back(parent_id);
      split_children[parent_id].push_back(track.id);
      created.related_ids.push_back(parent_id);
      created.reason = "split";
    }
    region_canonical[r] = track.id;
    result.new_ids.push_back(track.id);
    result.events.push_back(created);
    tracks_.emplace(track.id, std::move(track));
  }
  for (auto &[parent_id, children] : split_children) {
    result.events.push_back({TrackEventType::kSplit, parent_id, children,
                             tracks_.count(parent_id) &&
                                     tracks_.at(parent_id).last_seen_update ==
                                         index
                                 ? "parent kept its id"
                                 : "parent not observed"});
  }

  // 3. Unmatched existing tracks: merged, superseded, missing or expired.
  std::map<uint64_t, std::vector<uint64_t>> merged_into;
  std::vector<uint64_t> to_remove;
  for (size_t t = 0; t < nt; ++t) {
    if (assignment[t] >= 0) {
      continue;
    }
    TrackedRegion &track = tracks_.at(track_ids[t]);
    const double a_t = track_scan_area[t];
    double covered = 0.0;
    int best_r = -1;
    for (size_t r = 0; r < nr; ++r) {
      covered += inter[t][r];
      if (inter[t][r] > 0.0 && (best_r < 0 || inter[t][r] > inter[t][best_r])) {
        best_r = static_cast<int>(r);
      }
    }
    const double coverage = a_t > 0.0 ? std::min(1.0, covered / a_t) : 0.0;

    if (coverage >= config_.retire_min_coverage && best_r >= 0) {
      const uint64_t owner = region_canonical[best_r];
      if (inter[t][best_r] / a_t >= config_.split_merge_min_fraction) {
        merged_into[owner].push_back(track.id);
        result.events.push_back(
            {TrackEventType::kRemoved, track.id, {owner}, "merged"});
      } else {
        std::vector<uint64_t> owners;
        for (size_t r = 0; r < nr; ++r) {
          if (inter[t][r] > 0.0) {
            owners.push_back(region_canonical[r]);
          }
        }
        sortUnique(owners);
        result.events.push_back(
            {TrackEventType::kRemoved, track.id, owners, "superseded"});
      }
      to_remove.push_back(track.id);
      continue;
    }

    ++track.missed_updates;
    if (static_cast<int>(track.missed_updates) > config_.max_missed_updates) {
      result.events.push_back(
          {TrackEventType::kRemoved, track.id, {}, "expired"});
      to_remove.push_back(track.id);
      continue;
    }
    if (track.status == TrackStatus::kActive) {
      result.events.push_back(
          {TrackEventType::kMissing, track.id, {}, "not observed"});
    }
    track.status = TrackStatus::kMissing;
    track.geometry.local_id = -1;
  }
  for (auto &[owner, absorbed] : merged_into) {
    TrackedRegion &survivor = tracks_.at(owner);
    survivor.merged_ids = absorbed;
    result.events.push_back(
        {TrackEventType::kMerged, owner, absorbed, "absorbed tracks"});
  }
  for (uint64_t id : to_remove) {
    tracks_.erase(id);
    result.removed_ids.push_back(id);
  }

  // 4. Adjacency remapped from frame-local ids to canonical ids.
  std::unordered_map<int, uint64_t> local_to_canonical;
  for (size_t r = 0; r < nr; ++r) {
    local_to_canonical[regions[r].region.local_id] = region_canonical[r];
    result.local_to_canonical.emplace_back(regions[r].region.local_id,
                                           region_canonical[r]);
  }
  std::set<uint64_t> removed(to_remove.begin(), to_remove.end());
  for (auto &[id, track] : tracks_) {
    track.age = static_cast<uint32_t>(index - track.first_seen_update + 1);
    if (track.status == TrackStatus::kActive) {
      track.adjacent_ids.clear();
      for (int local : track.geometry.adjacent_local_ids) {
        track.adjacent_ids.push_back(local_to_canonical.at(local));
      }
      sortUnique(track.adjacent_ids);
    } else {
      auto &adj = track.adjacent_ids;
      adj.erase(std::remove_if(adj.begin(), adj.end(),
                               [&](uint64_t a) { return removed.count(a) > 0; }),
                adj.end());
      result.missing_ids.push_back(id);
    }
  }

  double total_area = 0.0;
  for (const PreparedRegion &p : regions) {
    total_area += p.region.area;
  }
  last_total_area_ = total_area;

  sortUnique(result.matched_ids);
  sortUnique(result.removed_ids);
  return result;
}

std::vector<TrackedRegion>
RegionTracker::publishableTracks(bool include_missing) const {
  std::vector<TrackedRegion> out;
  std::set<uint64_t> published;
  for (const auto &[id, track] : tracks_) {
    if (track.status == TrackStatus::kActive || include_missing) {
      out.push_back(track);
      published.insert(id);
    }
  }
  for (TrackedRegion &track : out) {
    auto &adj = track.adjacent_ids;
    adj.erase(std::remove_if(adj.begin(), adj.end(),
                             [&](uint64_t a) { return published.count(a) == 0; }),
              adj.end());
  }
  return out;
}

const char *toString(TrackEventType type) {
  switch (type) {
  case TrackEventType::kCreated:
    return "CREATED";
  case TrackEventType::kSplit:
    return "SPLIT";
  case TrackEventType::kMerged:
    return "MERGED";
  case TrackEventType::kMissing:
    return "MISSING";
  case TrackEventType::kRecovered:
    return "RECOVERED";
  case TrackEventType::kRemoved:
    return "REMOVED";
  }
  return "UNKNOWN";
}

namespace {

std::string joinIds(const std::vector<uint64_t> &ids) {
  std::ostringstream ss;
  ss << "[";
  for (size_t i = 0; i < ids.size(); ++i) {
    ss << (i ? " " : "") << ids[i];
  }
  ss << "]";
  return ss.str();
}

}  // namespace

std::string formatUpdateReport(const TrackerUpdateResult &result,
                               bool include_candidates) {
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(3);
  if (!result.accepted) {
    ss << "update REJECTED: " << result.rejection_reason;
    return ss.str();
  }
  ss << "update #" << result.update_index;
  if (result.forced) {
    ss << " (FORCED despite: " << result.rejection_reason << ")";
  }
  ss << "\n  local->canonical:";
  for (const auto &[local, canonical] : result.local_to_canonical) {
    ss << " " << local << "->" << canonical;
  }
  if (!result.dropped_local_ids.empty()) {
    ss << "\n  dropped degenerate local ids:";
    for (int id : result.dropped_local_ids) {
      ss << " " << id;
    }
  }
  ss << "\n  matched " << joinIds(result.matched_ids) << " new "
     << joinIds(result.new_ids) << " missing " << joinIds(result.missing_ids)
     << " removed " << joinIds(result.removed_ids);
  for (const TrackEvent &e : result.events) {
    ss << "\n  event " << toString(e.type) << " id=" << e.id << " related="
       << joinIds(e.related_ids) << " (" << e.reason << ")";
  }
  if (include_candidates) {
    for (const MatchCandidate &c : result.candidates) {
      ss << "\n  cand track=" << c.track_id << " local=" << c.local_id
         << " iou=" << c.iou << " cont=" << c.containment
         << " dist=" << c.centroid_distance << " area=" << c.area_ratio
         << " score=" << c.score << (c.gated ? " gated" : "")
         << (c.assigned ? " ASSIGNED" : "");
    }
  }
  return ss.str();
}

}  // namespace inc_dude
