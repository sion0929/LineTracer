# /mission/state  (mission_manager -> all nodes, dev GCS). State machine: design §9.1
uint8 STARTUP_WAIT=0
uint8 CHECKING=1
uint8 READY=2
uint8 RUNNING=3
uint8 STOPPING=4
uint8 FINISHED=5
uint8 FAULT=6
uint8 E_STOP=7
uint8 STOPPED=8

std_msgs/Header header
uint8 state
bool drive_enabled
string reason
string run_id
float32 elapsed_since_start   # [s] wall (steady) time since mission_manager start
