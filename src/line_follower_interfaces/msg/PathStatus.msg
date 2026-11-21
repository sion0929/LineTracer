# /path/status  (path_planner -> safety_guard, analysis)
uint8 EVALUATOR_RULE_BASED=0
uint8 EVALUATOR_MLP_ASSISTED=1

std_msgs/Header header
bool valid
DriveMode mode
float32 uncertainty
float32 verified_remaining_distance   # [m]
uint8 evaluator
bool mlp_healthy
string detail
