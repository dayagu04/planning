std_msgs/Header header # ros header
MsgHeader msg_header
MsgMeta msg_meta
uint64 isp_timestamp
uint8 lanes_size
LaneData[] lanes
uint8 lane_line_size
LaneLine[] lane_line
uint8 lane_ground_markings_size
LaneGroundMarking[] lane_ground_markings
uint8 stop_line_size
LaneLine[] stop_line
uint8 inhibit_line_size
LaneLine[] inhibit_line
uint8 speed_info_size
SpeedInfo[] speed_info
CommonCameraPerceptionInputTimestamp camera_perception_input_timestamp
