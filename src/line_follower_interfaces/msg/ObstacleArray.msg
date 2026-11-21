# /perception/obstacles  (obstacle_detector -> path_planner, extension only; never run in competition)
# header.stamp = camera capture stamp
std_msgs/Header header
Obstacle[] obstacles
bool healthy
