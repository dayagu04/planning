std_msgs/Header header # ros header
MsgHeader msg_header
MsgMeta msg_meta
uint64 position_age
uint8 positions_size
PostionData[] positions
uint8 relative_pos_size
RelativePostion[] relative_pos
uint8 absolute_pos_size
AbsolutePostion[] absolute_pos
uint8 position_warning_size
PositionWarning[] position_warning
uint8 position_geofence_size
PositionGeofence[] position_geofence
uint8 fail_safe_size
PostionFailSafe[] fail_safe