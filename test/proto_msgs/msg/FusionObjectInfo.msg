float32 long_position   # float32  障碍物纵向距离
float32 lat_position    # float32  障碍物横向距离
float32 heading_angle   # float32  障碍物航向角
float32 speed          # float32  障碍物绝对速度
float32 long_velocity   # float32  障碍物纵向绝对速度
float32 lat_velocity     # float32  障碍物横向绝对速度
float32 acceleration    # float32  障碍物绝对加速度
float32 long_acceleration  # float32  障碍物纵向加速度
float32 lat_acceleration    # float32  障碍物横向加速度
float32 curvature         # float32  曲率
uint32 type    # uint8  障碍物类型
float32 width              # float32  障碍物宽
float32 height             # float32  障碍物高
float32 length             # float32  障碍物长
uint32 nearest_side      # uint8
uint32 turn_indicator    # uint8
uint32 brake_light_indicator   # uint8
uint32 hazard_light_indicator  # uint8
uint32 motion_pattern_current    # uint8  障碍物当前运动状态
uint32 motion_pattern_history   # uint8  障碍物历史运动状态
float32 distance_to_left_near_lane_marking  # float32  到左边线距离
float32 distance_to_right_near_lane_marking  # float32  到右边线距离
float32 acceleration_std_dev   # float32 加速度的标准差
uint32 cmbb_primary_confidence        # uint8  自动紧急制动(CMBB)第一置信度等级
uint32 cmbb_secondary_confidence    # uint8  自动紧急制动(CMBB)第二置信度等级
uint32 fcw_confidence                        # uint8 前向碰撞预警系统(FCW)置信度等级
uint32 fusion_source   # uint8  融合障碍物的数据来源
uint32[3] sensor_source  # uint8[3] 传感器来源
float32 heading_angle_std_dev     # float32  航向角标准差
uint32 id           # uint8  序号
uint32 track_id     # uint8  融合障碍物id
uint32 track_age   # uint16 融合障碍物年龄
float32 lat_position_std_dev     # float32  横向位置标准差
uint32 left_near_lane_marking_confidence   # uint8
float32 long_position_std_dev    # float32  纵向位置标准差
uint32 motion_model   # uint8
uint32 position_confidence  # uint8 位置置信度等级
uint32 right_near_lane_marking_confidence  # uint8
float32 speed_std_dev      # float32  速度方差
uint32 tja_confidence    # uint8  交通拥堵辅助(TJA)置信度
uint32 track_status    # uint8  跟踪状态
uint32 traffic_scenario   # uint8  not used
uint32 vision_id   # uint8 相机障碍物编号
uint32 radar_id    # uint8  雷达障碍物编号
float32 obstacle_probability   # float32  是障碍物的概率
uint32 target_selection_type   # uint8  障碍物所在方位状态，和车道有关，ACC使用
uint32 cut_type   # uint8  障碍物切入切出状态，ACC使用
uint32 feature_target_cutflag  # uint8，ACC使用
float32 dst_lat_from_mid_of_laneself #自车离车道线距离
uint32 target_selection_ccr   # uint8  车辆障碍物所在方位状态，和车道有关，AEB使用
uint32 target_selection_vru   # uint8  行人障碍物所在方位状态，和车道有关，AEB使用