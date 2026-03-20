std_msgs/Header header # ros header
MsgHeader msg_header
MsgMeta msg_meta
uint64 isp_timestamp
uint8 parking_slots_size
CameraSlotType[] parking_slots
uint8 vision_slot_limiters_size
VisionSlotLimiter[] vision_slot_limiters
uint8 vision_slot_specificatios_size
VisionSlotSpecification[] vision_slot_specificatios
float32[16] ego_motion
uint8 is_after_reset
uint64[4] images_timestamp
