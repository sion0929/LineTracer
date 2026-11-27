# /lf/perception/frame (perception_node -> map_node), one per processed camera frame
# (docs/architecture/NODE_SPLIT.md). The map node applies it with the same LineMap calls the
# single follower_node makes: freeze() first (when freeze), then update() (when use_for_map).
std_msgs/Header header        # stamp = capture time, frame_id = odom
bool has_pose
float64 x                     # pose at capture (odom) [m, m, rad]
float64 y
float64 yaw
float32 pitch                 # camera pitch used for the projection [rad]
float32 imu_pitch             # filtered IMU body pitch, nose up > 0 [rad]
float32 elev                  # wheel height above the start floor [m]
float32 travel                # odometry distance [m]
uint8 SLOPE_FLAT=0
uint8 SLOPE_CLIMB=1
uint8 SLOPE_DECK=2
uint8 SLOPE_DESCENT=3
uint8 slope
bool use_for_map              # call LineMap.update with the masks below
bool freeze                   # call LineMap.freeze(x, y, freeze_radius) first
float32 freeze_radius         # [m]
# BEV grid (vision.py): row 0 = x_far (ahead), last row = x_near; col 0 = +y_half (left)
uint16 rows
uint16 cols
float32 res
float32 x_near
float32 x_far
float32 y_half
# masks: np.packbits(mask > 0) of a 0/255 mask (ceil(rows*cols/8) bytes), or the raw uint8
# mask (rows*cols bytes) when it has other values; empty = absent
uint8[] line
uint8[] valid
uint8[] far                   # far-band valid mask (lower confidence), or empty
uint8[] add_only              # cells that may only add unseen line, or empty
