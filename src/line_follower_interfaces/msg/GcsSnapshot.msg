# Dev snapshot, aggregated by mission_manager only when enabled. Geometry is in odom.
std_msgs/Header header
MissionState mission
bool pose_valid
geometry_msgs/Pose robot_pose
ObservedGraph observed_graph
bool path_valid
LocalPath selected_path
PathCandidates candidates
PathStatus path_status
SafetyStatus safety_status
# Transitions since the previous snapshot, so short states are not lost at 2 Hz.
MissionState[] mission_events
PathStatus[] mode_events
SafetyStatus[] safety_events
PerceptionStatus perception_status
string[] stale_sources
