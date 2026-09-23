# A free-space region produced by Incremental DuDe and tracked over time.
# All coordinates are in the header.frame_id of the enclosing Region2DArray.

uint8 STATUS_ACTIVE=1   # observed in the latest accepted decomposition
uint8 STATUS_MISSING=2  # not observed recently; geometry is the last observed one

# Canonical, persistent region id. Allocated monotonically; never reused.
uint64 id
uint8 status

geometry_msgs/Point centroid     # area centroid (z = 0); may lie outside non-convex polygons
geometry_msgs/Point[] polygon    # boundary, counter-clockwise, not closed (z = 0)
float64 area                     # m^2
uint64[] adjacent_ids            # canonical ids of adjacent published regions

# Lifecycle, counted in accepted decomposition updates.
uint64 first_seen_update
uint64 last_seen_update
uint32 age
uint32 hits
uint32 missed_updates            # consecutive updates without a match
int32 source_region_index        # frame-local index in the latest decomposition, -1 if not observed

# Lineage from the latest accepted update only (empty otherwise).
uint64[] split_from              # track(s) this region split off from
uint64[] merged_ids              # tracks absorbed into this region
