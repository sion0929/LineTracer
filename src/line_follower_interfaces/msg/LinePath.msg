# /lf/path (planning_node -> control_node): local path, one per map patch
std_msgs/Header header        # frame_id = odom
geometry_msgs/Point[] points  # nearest first, ~2 cm apart (z = 0)
string end_state              # 'open' | 't' | 'len'
bool has_t_point              # finish T accepted
geometry_msgs/Point t_point
