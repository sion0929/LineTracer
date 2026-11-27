# /lf/pose (perception_node -> map / planning / control): dead-reckoned pose, on every /odom
std_msgs/Header header
float64 x                     # odom [m]
float64 y
float64 yaw                   # [rad]
float32 travel                # distance driven [m]
float32 pitch_f               # filtered IMU pitch, nose up > 0 [rad]
float32 elev                  # wheel height above the start floor [m]
bool rocking                  # body rocking on flat ground (camera range cut)
uint8 slope                   # PerceptionFrame.SLOPE_*
