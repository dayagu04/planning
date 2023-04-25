uint64 timestamp                   # 道路建模时间戳（预测）
float32 offset_lat                  # 横向距离
float32 angle                        # 角度
float32 curvature                    # 曲率 
float32 curvature_rate_1             # 第一段曲率变化率
float32 curvature_rate_2            # 第二段曲率变化率
float32 curvature_rate_3            # 第三段曲率变化率
float32 segment_1_length             # 第一段长度
float32 segment_2_length             # 第二段长度
float32 segment_3_length            # 第三段长度
float32 distance_with_high_quality  # 高质量区域
float32 distance_with_low_quality   # 低质量区域
float32 lane_width                  # 车道宽
bool vaild                        # 输出车道线是否有效
int32[] obj_use_for_update_angel_dir           # 障碍物是否用于AngelDir Update
int32[] obj_use_for_update_position            # 障碍物是否用于Position Updateb
int32[] obj_valid_for_extrpn  
int32[] obj_valid_for_update_position 
bool veh_self_on_slip_road_left           # 自车是否在左辅道
bool veh_self_on_slip_road_right          # 自车是否在右辅道