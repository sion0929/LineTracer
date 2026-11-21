# /perception/line  (line_detector -> path_planner, mission_manager)
# header.stamp = camera CAPTURE stamp, never the callback receive time (design §5.3-1)
std_msgs/Header header
bool line_found
LineSegment[] segments
Junction[] junctions
float32 observed_range_min   # [m] nearest valid ground observation
float32 observed_range_max   # [m] farthest valid ground observation
bool projection_valid        # false where flat-ground projection is untrusted (slopes, design §6.3)
float32 quality              # [0, 1]
bool finish_marker_candidate # T-marker candidate; mission_manager makes the finish decision (design §9.3)

bool image_tracking_valid
float32 image_offset # normalized pixel offset, positive right
float32 image_heading # near-minus-far pixel centre, positive left heading
