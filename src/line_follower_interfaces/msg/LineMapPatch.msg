# /lf/map/patch (map_node -> planning_node): the window of the 2400 x 2400 line map
# (line_map.py, 5 mm cells, cell (i, j) at y = (i - 1200) * 0.005, x = (j - 1200) * 0.005 in odom)
# changed by one frame (rows = cols = 0 when nothing changed), or with full = true the whole
# written extent (sent once a second so a late planning node can catch up).
std_msgs/Header header
int32 i0                      # top-left cell (row = y index, col = x index)
int32 j0
uint16 rows
uint16 cols
uint8[] grid                  # rows*cols bytes each, row-major; empty = layer absent
uint8[] seen
uint8[] conf
uint8[] frozen
uint8[] restored
bool full
