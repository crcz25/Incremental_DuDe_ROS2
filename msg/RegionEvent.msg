# A tracking lifecycle event produced by one accepted decomposition update.

uint8 CREATED=0    # id: new region; related_ids: split parent, if any
uint8 SPLIT=1      # id: parent region; related_ids: regions that split off it
uint8 MERGED=2     # id: surviving region; related_ids: absorbed regions
uint8 MISSING=3    # id: region no longer observed
uint8 RECOVERED=4  # id: missing region observed again
uint8 REMOVED=5    # id: region retired; reason: expired | merged | superseded

uint8 type
uint64 id
uint64[] related_ids
string reason
