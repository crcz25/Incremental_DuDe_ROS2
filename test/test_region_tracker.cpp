#include <gtest/gtest.h>

#include <limits>
#include <map>
#include <set>

#include "inc_dude/region_tracker.hpp"
#include "test_helpers.hpp"

using namespace inc_dude;
using inc_dude::test::canonicalOf;
using inc_dude::test::hasEvent;
using inc_dude::test::rect;
using inc_dude::test::region;
using inc_dude::test::update;

namespace {

// Three 4 m x 4 m rooms in a row: A | B | C.
std::vector<Region2D> threeRooms(double dx = 0.0) {
  return {region(0, rect(0 + dx, 0, 4 + dx, 4), {1}),
          region(1, rect(4 + dx, 0, 8 + dx, 4), {0, 2}),
          region(2, rect(8 + dx, 0, 12 + dx, 4), {1})};
}

std::map<uint64_t, TrackedRegion> snapshot(const RegionTracker &t) {
  return t.tracks();
}

void expectSameState(const std::map<uint64_t, TrackedRegion> &a,
                     const std::map<uint64_t, TrackedRegion> &b) {
  ASSERT_EQ(a.size(), b.size());
  for (const auto &[id, ta] : a) {
    ASSERT_TRUE(b.count(id));
    const TrackedRegion &tb = b.at(id);
    EXPECT_EQ(ta.status, tb.status);
    EXPECT_EQ(ta.age, tb.age);
    EXPECT_EQ(ta.hits, tb.hits);
    EXPECT_EQ(ta.missed_updates, tb.missed_updates);
    EXPECT_EQ(ta.adjacent_ids, tb.adjacent_ids);
    EXPECT_DOUBLE_EQ(ta.geometry.area, tb.geometry.area);
  }
}

}  // namespace

TEST(RegionTracker, DefaultConfigIsValid) {
  EXPECT_EQ(validateConfig(TrackerConfig()), "");
  TrackerConfig bad;
  bad.gate_min_containment = 1.5;
  EXPECT_NE(validateConfig(bad), "");
}

TEST(RegionTracker, UnchangedRegionsKeepIds) {
  RegionTracker tracker;
  const TrackerUpdateResult first = tracker.update(update(threeRooms()));
  ASSERT_TRUE(first.accepted);
  EXPECT_EQ(first.new_ids, (std::vector<uint64_t>{1, 2, 3}));
  EXPECT_TRUE(hasEvent(first, TrackEventType::kCreated, 1));

  for (int i = 0; i < 5; ++i) {
    const TrackerUpdateResult r = tracker.update(update(threeRooms()));
    ASSERT_TRUE(r.accepted);
    EXPECT_TRUE(r.new_ids.empty());
    EXPECT_TRUE(r.removed_ids.empty());
    EXPECT_TRUE(r.events.empty());
    EXPECT_EQ(r.matched_ids, (std::vector<uint64_t>{1, 2, 3}));
    EXPECT_EQ(canonicalOf(r, 0), 1u);
    EXPECT_EQ(canonicalOf(r, 1), 2u);
    EXPECT_EQ(canonicalOf(r, 2), 3u);
  }
  const TrackedRegion &b = tracker.tracks().at(2);
  EXPECT_EQ(b.age, 6u);
  EXPECT_EQ(b.hits, 6u);
  EXPECT_EQ(b.first_seen_update, 1u);
  EXPECT_EQ(b.last_seen_update, 6u);
  EXPECT_EQ(b.adjacent_ids, (std::vector<uint64_t>{1, 3}));
}

TEST(RegionTracker, IdsFollowGeometryNotFrameLocalOrder) {
  RegionTracker tracker;
  ASSERT_TRUE(tracker.update(update(threeRooms())).accepted);

  // Same rooms, local ids permuted as the incremental decomposer does when
  // it re-orders Stable_graph::Region_contour.
  const TrackerUpdateResult r = tracker.update(
      update({region(0, rect(8, 0, 12, 4), {2}),
              region(1, rect(0, 0, 4, 4), {2}),
              region(2, rect(4, 0, 8, 4), {0, 1})}));
  ASSERT_TRUE(r.accepted);
  EXPECT_EQ(canonicalOf(r, 0), 3u);
  EXPECT_EQ(canonicalOf(r, 1), 1u);
  EXPECT_EQ(canonicalOf(r, 2), 2u);
  EXPECT_TRUE(r.new_ids.empty());
}

TEST(RegionTracker, TranslatedAndReshapedRegionsKeepIds) {
  RegionTracker tracker;
  ASSERT_TRUE(tracker.update(update(threeRooms())).accepted);

  // Small shift of every boundary (e.g. map re-alignment).
  TrackerUpdateResult r = tracker.update(update(threeRooms(0.3)));
  ASSERT_TRUE(r.accepted);
  EXPECT_TRUE(r.new_ids.empty());
  EXPECT_EQ(r.matched_ids, (std::vector<uint64_t>{1, 2, 3}));

  // Boundary between A and B moves, C grows a lot while being explored.
  r = tracker.update(update({region(0, rect(0, 0, 4.8, 4), {1}),
                             region(1, rect(4.8, 0, 8, 4), {0, 2}),
                             region(2, rect(8, 0, 16, 6), {1})}));
  ASSERT_TRUE(r.accepted);
  EXPECT_TRUE(r.new_ids.empty());
  EXPECT_EQ(canonicalOf(r, 0), 1u);
  EXPECT_EQ(canonicalOf(r, 1), 2u);
  EXPECT_EQ(canonicalOf(r, 2), 3u);
  EXPECT_NEAR(tracker.tracks().at(3).geometry.area, 48.0, 1e-9);
}

TEST(RegionTracker, AppearanceAndDisappearance) {
  TrackerConfig config;
  config.max_missed_updates = 2;
  RegionTracker tracker(config);
  ASSERT_TRUE(tracker.update(update(threeRooms())).accepted);

  // A new room D appears above A.
  auto rooms = threeRooms();
  rooms[0].adjacent_local_ids = {1, 3};
  rooms.push_back(region(3, rect(0, 4, 4, 8), {0}));
  TrackerUpdateResult r = tracker.update(update(rooms));
  ASSERT_TRUE(r.accepted);
  EXPECT_EQ(r.new_ids, (std::vector<uint64_t>{4}));
  EXPECT_TRUE(hasEvent(r, TrackEventType::kCreated, 4));
  EXPECT_EQ(tracker.tracks().at(1).adjacent_ids,
            (std::vector<uint64_t>{2, 4}));

  // C disappears and nothing covers its space.
  rooms.erase(rooms.begin() + 2);
  rooms[1].adjacent_local_ids = {0};
  r = tracker.update(update(rooms));
  ASSERT_TRUE(r.accepted);
  EXPECT_TRUE(hasEvent(r, TrackEventType::kMissing, 3));
  EXPECT_EQ(r.missing_ids, (std::vector<uint64_t>{3}));
  EXPECT_EQ(tracker.tracks().at(3).status, TrackStatus::kMissing);

  // Missing regions are only published on request, without stale adjacency.
  const auto active = tracker.publishableTracks(false);
  EXPECT_EQ(active.size(), 3u);
  const auto all = tracker.publishableTracks(true);
  EXPECT_EQ(all.size(), 4u);
  for (const TrackedRegion &t : active) {
    EXPECT_NE(t.id, 3u);
    for (uint64_t adj : t.adjacent_ids) {
      EXPECT_NE(adj, 3u);
    }
  }

  r = tracker.update(update(rooms));  // second miss: still kept
  ASSERT_TRUE(r.accepted);
  EXPECT_TRUE(tracker.tracks().count(3));
  EXPECT_FALSE(hasEvent(r, TrackEventType::kMissing, 3));  // reported once

  r = tracker.update(update(rooms));  // third miss > max_missed_updates
  ASSERT_TRUE(r.accepted);
  EXPECT_EQ(r.removed_ids, (std::vector<uint64_t>{3}));
  EXPECT_TRUE(hasEvent(r, TrackEventType::kRemoved, 3, {}, "expired"));
  EXPECT_FALSE(tracker.tracks().count(3));
}

TEST(RegionTracker, TemporarilyMissedRegionRecoversItsId) {
  RegionTracker tracker;
  ASSERT_TRUE(tracker.update(update(threeRooms())).accepted);

  auto partial = threeRooms();
  partial.pop_back();
  partial[1].adjacent_local_ids = {0};
  for (int i = 0; i < 2; ++i) {
    ASSERT_TRUE(tracker.update(update(partial)).accepted);
  }
  EXPECT_EQ(tracker.tracks().at(3).missed_updates, 2u);

  // C comes back, slightly different.
  auto back = threeRooms();
  back[2] = region(2, rect(8, 0, 12.3, 4.2), {1});
  const TrackerUpdateResult r = tracker.update(update(back));
  ASSERT_TRUE(r.accepted);
  EXPECT_EQ(canonicalOf(r, 2), 3u);
  EXPECT_TRUE(r.new_ids.empty());
  EXPECT_TRUE(hasEvent(r, TrackEventType::kRecovered, 3));
  const TrackedRegion &c = tracker.tracks().at(3);
  EXPECT_EQ(c.status, TrackStatus::kActive);
  EXPECT_EQ(c.missed_updates, 0u);
  EXPECT_EQ(c.hits, 2u);
  EXPECT_EQ(c.age, 4u);
}

TEST(RegionTracker, CompetingAssignmentsAreResolvedGlobally) {
  RegionTracker tracker;
  // A = [0,4]x[0,4] (16 m^2), B = [4,5]x[0,4] (4 m^2).
  ASSERT_TRUE(tracker
                  .update(update({region(0, rect(0, 0, 4, 4), {1}),
                                  region(1, rect(4, 0, 5, 4), {0})}))
                  .accepted);

  // The cut moves: R0 = [0,1]x[0,4], R1 = [1,5]x[0,4].
  // Scores: A-R1 0.62 (best), A-R0 0.39, B-R1 0.39, B-R0 not a candidate.
  // Greedy would give R1 to A and leave B unmatched; the global assignment
  // keeps both ids (A-R0, B-R1).
  const TrackerUpdateResult r =
      tracker.update(update({region(0, rect(0, 0, 1, 4), {1}),
                             region(1, rect(1, 0, 5, 4), {0})}));
  ASSERT_TRUE(r.accepted);
  EXPECT_EQ(canonicalOf(r, 0), 1u);
  EXPECT_EQ(canonicalOf(r, 1), 2u);
  EXPECT_TRUE(r.new_ids.empty());
  EXPECT_TRUE(r.removed_ids.empty());

  double best_score = 0.0;
  uint64_t best_track = 0;
  int best_local = -1;
  for (const MatchCandidate &c : r.candidates) {
    if (c.gated && c.score > best_score) {
      best_score = c.score;
      best_track = c.track_id;
      best_local = c.local_id;
    }
  }
  EXPECT_EQ(best_track, 1u);  // the greedy choice (A-R1) was not taken
  EXPECT_EQ(best_local, 1);
}

TEST(RegionTracker, SplitKeepsParentIdForLargestPart) {
  RegionTracker tracker;
  ASSERT_TRUE(tracker.update(update({region(0, rect(0, 0, 8, 4))})).accepted);

  const TrackerUpdateResult r =
      tracker.update(update({region(0, rect(5, 0, 8, 4), {1}),
                             region(1, rect(0, 0, 5, 4), {0})}));
  ASSERT_TRUE(r.accepted);
  EXPECT_EQ(canonicalOf(r, 1), 1u);  // larger part keeps the parent id
  const uint64_t child = canonicalOf(r, 0);
  EXPECT_EQ(child, 2u);
  EXPECT_TRUE(hasEvent(r, TrackEventType::kSplit, 1, {child}));
  EXPECT_TRUE(hasEvent(r, TrackEventType::kCreated, child, {1}, "split"));
  EXPECT_EQ(tracker.tracks().at(child).split_from,
            (std::vector<uint64_t>{1}));
  EXPECT_EQ(tracker.tracks().at(1).adjacent_ids,
            (std::vector<uint64_t>{child}));

  // Lineage metadata only describes the latest update.
  ASSERT_TRUE(tracker
                  .update(update({region(0, rect(5, 0, 8, 4), {1}),
                                  region(1, rect(0, 0, 5, 4), {0})}))
                  .accepted);
  EXPECT_TRUE(tracker.tracks().at(child).split_from.empty());
}

TEST(RegionTracker, MergeRetiresAbsorbedTrack) {
  RegionTracker tracker;
  ASSERT_TRUE(tracker
                  .update(update({region(0, rect(0, 0, 5, 4), {1}),
                                  region(1, rect(5, 0, 8, 4), {0}),
                                  region(2, rect(8, 0, 12, 4), {1})}))
                  .accepted);

  const TrackerUpdateResult r =
      tracker.update(update({region(0, rect(0, 0, 8, 4), {1}),
                             region(1, rect(8, 0, 12, 4), {0})}));
  ASSERT_TRUE(r.accepted);
  EXPECT_EQ(canonicalOf(r, 0), 1u);  // larger absorbed track survives
  EXPECT_EQ(canonicalOf(r, 1), 3u);
  EXPECT_EQ(r.removed_ids, (std::vector<uint64_t>{2}));
  EXPECT_TRUE(hasEvent(r, TrackEventType::kMerged, 1, {2}));
  EXPECT_TRUE(hasEvent(r, TrackEventType::kRemoved, 2, {1}, "merged"));
  EXPECT_FALSE(tracker.tracks().count(2));
  EXPECT_EQ(tracker.tracks().at(1).merged_ids, (std::vector<uint64_t>{2}));
  EXPECT_EQ(tracker.tracks().at(1).adjacent_ids,
            (std::vector<uint64_t>{3}));
  EXPECT_EQ(tracker.tracks().at(3).adjacent_ids,
            (std::vector<uint64_t>{1}));
}

TEST(RegionTracker, AdjacencyIsRemappedToCanonicalIds) {
  RegionTracker tracker;
  // Frame-local ids need not be contiguous or ordered.
  ASSERT_TRUE(tracker
                  .update(update({region(10, rect(0, 0, 4, 4), {20}),
                                  region(20, rect(4, 0, 8, 4), {10, 30}),
                                  region(30, rect(8, 0, 12, 4), {20})}))
                  .accepted);
  const TrackerUpdateResult r =
      tracker.update(update({region(7, rect(4, 0, 8, 4), {3, 5}),
                             region(5, rect(8, 0, 12, 4), {7}),
                             region(3, rect(0, 0, 4, 4), {7})}));
  ASSERT_TRUE(r.accepted);

  const auto published = tracker.publishableTracks(false);
  std::map<uint64_t, std::vector<uint64_t>> adjacency;
  for (const TrackedRegion &t : published) {
    adjacency[t.id] = t.adjacent_ids;
  }
  EXPECT_EQ(adjacency.at(1), (std::vector<uint64_t>{2}));
  EXPECT_EQ(adjacency.at(2), (std::vector<uint64_t>{1, 3}));
  EXPECT_EQ(adjacency.at(3), (std::vector<uint64_t>{2}));
  // Symmetric and only canonical ids of published regions.
  for (const auto &[id, adj] : adjacency) {
    for (uint64_t other : adj) {
      ASSERT_TRUE(adjacency.count(other));
      const auto &back = adjacency.at(other);
      EXPECT_NE(std::find(back.begin(), back.end(), id), back.end());
    }
  }
}

TEST(RegionTracker, InvalidUpdatesAreRejectedWithoutTouchingState) {
  RegionTracker tracker;
  ASSERT_TRUE(tracker.update(update(threeRooms())).accepted);
  const auto before = snapshot(tracker);
  const uint64_t next_id = tracker.nextId();

  std::vector<RegionUpdate> invalid;
  {
    auto rooms = threeRooms();
    rooms[1].polygon[2].x = std::numeric_limits<double>::quiet_NaN();
    invalid.push_back(update(rooms));
  }
  invalid.push_back(update(threeRooms(), "odom"));
  invalid.push_back(update(threeRooms(), ""));
  invalid.push_back(update(threeRooms(), "map", 0.0));
  invalid.push_back(update({}));
  {
    auto rooms = threeRooms();
    rooms[0].adjacent_local_ids = {1, 42};
    invalid.push_back(update(rooms));
  }
  {
    auto rooms = threeRooms();
    rooms[2].local_id = 1;
    invalid.push_back(update(rooms));
  }
  // Only degenerate regions left after filtering.
  invalid.push_back(update({region(0, rect(0, 0, 0.1, 0.1))}));

  for (const RegionUpdate &u : invalid) {
    const TrackerUpdateResult r = tracker.update(u);
    EXPECT_FALSE(r.accepted);
    EXPECT_FALSE(r.rejection_reason.empty());
    expectSameState(before, snapshot(tracker));
  }
  EXPECT_EQ(tracker.acceptedUpdates(), 1u);
  EXPECT_EQ(tracker.rejectedUpdates(), invalid.size());
  EXPECT_EQ(tracker.nextId(), next_id);

  // Recovery: the next valid update continues with the same ids.
  const TrackerUpdateResult r = tracker.update(update(threeRooms(0.1)));
  ASSERT_TRUE(r.accepted);
  EXPECT_TRUE(r.new_ids.empty());
  EXPECT_EQ(r.matched_ids, (std::vector<uint64_t>{1, 2, 3}));
  EXPECT_EQ(tracker.tracks().at(2).age, 2u);
}

TEST(RegionTracker, PartialUpdateIsRejectedThenForcedAfterLimit) {
  TrackerConfig config;
  config.max_consecutive_rejections = 2;
  RegionTracker tracker(config);
  ASSERT_TRUE(tracker.update(update(threeRooms())).accepted);
  const auto before = snapshot(tracker);

  // Only one of three rooms: two thirds of the area vanished.
  const RegionUpdate partial = update({region(0, rect(0, 0, 4, 4))});
  for (int i = 0; i < 2; ++i) {
    const TrackerUpdateResult r = tracker.update(partial);
    EXPECT_FALSE(r.accepted);
    EXPECT_NE(r.rejection_reason.find("area"), std::string::npos);
    expectSameState(before, snapshot(tracker));
  }

  // A complete update in between resets the rejection counter.
  ASSERT_TRUE(tracker.update(update(threeRooms())).accepted);
  EXPECT_FALSE(tracker.update(partial).accepted);
  EXPECT_FALSE(tracker.update(partial).accepted);

  // The change persists: accept it (the map really changed).
  const TrackerUpdateResult forced = tracker.update(partial);
  ASSERT_TRUE(forced.accepted);
  EXPECT_TRUE(forced.forced);
  EXPECT_EQ(canonicalOf(forced, 0), 1u);
  EXPECT_EQ(forced.missing_ids, (std::vector<uint64_t>{2, 3}));
}

TEST(RegionTracker, OverlappingRegionsAreRejected) {
  RegionTracker tracker;
  const TrackerUpdateResult r = tracker.update(
      update({region(0, rect(0, 0, 4, 4)), region(1, rect(1, 1, 5, 5))}));
  EXPECT_FALSE(r.accepted);
  EXPECT_NE(r.rejection_reason.find("overlap"), std::string::npos);
  EXPECT_TRUE(tracker.tracks().empty());
}

TEST(RegionTracker, DegenerateRegionsAreDroppedFromUpdateAndAdjacency) {
  RegionTracker tracker;
  auto rooms = threeRooms();
  rooms[1].adjacent_local_ids = {0, 2, 3};
  rooms.push_back(region(3, rect(8, 4, 8.2, 4.2), {1}));  // 0.04 m^2
  const TrackerUpdateResult r = tracker.update(update(rooms));
  ASSERT_TRUE(r.accepted);
  EXPECT_EQ(r.dropped_local_ids, (std::vector<int>{3}));
  EXPECT_EQ(tracker.tracks().size(), 3u);
  EXPECT_EQ(tracker.tracks().at(2).adjacent_ids,
            (std::vector<uint64_t>{1, 3}));
}

TEST(RegionTracker, ExpiredIdsAreNeverReused) {
  TrackerConfig config;
  config.max_missed_updates = 0;
  RegionTracker tracker(config);
  ASSERT_TRUE(tracker.update(update(threeRooms())).accepted);

  auto partial = threeRooms();
  partial.pop_back();
  partial[1].adjacent_local_ids = {0};
  TrackerUpdateResult r = tracker.update(update(partial));
  ASSERT_TRUE(r.accepted);
  EXPECT_EQ(r.removed_ids, (std::vector<uint64_t>{3}));

  // A region shows up exactly where the expired one was.
  r = tracker.update(update(threeRooms()));
  ASSERT_TRUE(r.accepted);
  const uint64_t id = canonicalOf(r, 2);
  EXPECT_EQ(id, 4u);
  EXPECT_NE(id, 3u);

  std::set<uint64_t> seen;
  for (const auto &[tid, t] : tracker.tracks()) {
    EXPECT_TRUE(seen.insert(tid).second);
    EXPECT_LT(tid, tracker.nextId());
  }
}

TEST(RegionTracker, DebugReportListsIdsScoresAndEvents) {
  RegionTracker tracker;
  ASSERT_TRUE(tracker.update(update({region(0, rect(0, 0, 8, 4))})).accepted);
  const TrackerUpdateResult r =
      tracker.update(update({region(4, rect(0, 0, 5, 4), {9}),
                             region(9, rect(5, 0, 8, 4), {4})}));
  const std::string report = formatUpdateReport(r, true);
  EXPECT_NE(report.find("4->1"), std::string::npos);
  EXPECT_NE(report.find("9->2"), std::string::npos);
  EXPECT_NE(report.find("SPLIT"), std::string::npos);
  EXPECT_NE(report.find("iou="), std::string::npos);
  EXPECT_NE(report.find("ASSIGNED"), std::string::npos);

  const std::string rejected =
      formatUpdateReport(tracker.update(update({}, "map")), true);
  EXPECT_NE(rejected.find("REJECTED"), std::string::npos);
}
