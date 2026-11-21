# Path point data contract (design §5.2, §7.2). frame_id lives in the enclosing message header.
uint8 SOURCE_OBSERVED=0
uint8 SOURCE_MEMORY_UPDATED=1
uint8 SOURCE_UNKNOWN=2   # never sent to the controller as a confirmed path

float64 x
float64 y
uint8 source
uint32 segment_id
float32 confidence
float32 uncertainty
builtin_interfaces/Time capture_stamp
builtin_interfaces/Time valid_until
