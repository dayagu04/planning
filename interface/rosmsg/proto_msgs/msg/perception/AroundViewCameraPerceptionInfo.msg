std_msgs/Header header
common_Header msg_header
uint64 id # 消息id
#common_Header header # 头信息
uint64 avm_time_stamp # avm拼接图生成时的时间戳
uint64 left_camera_exposure_time_stamp # 左环视相机的曝光时间戳
uint64 front_camera_exposure_time_stamp # 前环视相机的曝光时间戳
uint64 right_camera_exposure_time_stamp # 右环视相机的曝光时间戳
uint64 rear_camera_exposure_time_stamp # 后环视相机的曝光时间戳
ParkingSpace[] parking_spaces # 视觉车位线信息   <最大8个>
ParkingLimiter[] parking_limiters # 视觉限位器信息
ParkingColumn[] parking_columns # 视觉立柱接地线信息
ParkingWall[] parking_walls # 视觉墙面接地线信息
ParkingDeceler[] parking_decelers # 视觉减速带信息
ParkingLane[] parking_lanes # 视觉车道线信息
ParkingArrow[] parking_arrows # 视觉地面箭头信息
ParkingLeftExtend[] parking_left_extends     
ParkingRightExtend[] parking_right_extends  
uint32 parking_space_num
uint32 parking_limiter_num
uint32 parking_deceler_num
uint32 parking_column_num
uint32 parking_wall_num
uint32 parking_left_extend_num
uint32 parking_right_extend_num
uint32 parking_arrow_num
uint32 parking_lane_num