# /localization/estimate  (path_planner PoseEstimator -> analysis). header.frame_id = odom
std_msgs/Header header
geometry_msgs/Pose pose
float32 pitch                 # [rad] from IMU, used for slope handling
float64[9] covariance_xy_yaw
bool healthy
