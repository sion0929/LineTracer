# /perception/status  (line_detector -> safety_guard, mission_manager, analysis)
uint8 BACKEND_OPENCV_BASELINE=0
uint8 BACKEND_FAST_SCNN=1

std_msgs/Header header
builtin_interfaces/Time last_capture_stamp
float32 capture_to_publish_ms
float32 inference_ms
float32 postprocess_ms
uint8 backend
bool fallback_active
bool healthy
string detail
