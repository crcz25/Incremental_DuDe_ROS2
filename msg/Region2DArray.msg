# Stable, tracked free-space regions of the occupancy map.
# Published once per accepted decomposition update.

std_msgs/Header header    # frame_id and stamp of the source nav_msgs/OccupancyGrid
uint64 update_index       # index of the accepted decomposition update (1-based)
Region2D[] regions        # ordered by canonical id
RegionEvent[] events      # split/merge/lifecycle events of this update
