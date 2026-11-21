# /path/local  (path_planner -> motion_controller). header.frame_id = robot base frame
std_msgs/Header header
PathPoint[] points                # never contains SOURCE_UNKNOWN
geometry_msgs/Point target_point
float32 target_heading            # [rad] in header.frame_id
float32 speed_limit               # [m/s]
DriveMode mode
builtin_interfaces/Time valid_until

bool image_tracking # image steering only; no invented metric points
float32 image_offset
float32 image_heading
