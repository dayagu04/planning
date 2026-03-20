std_msgs/Header ros_header
common_Header header # 头信息
float32 vehicle_speed # 本车实际车速         (米/秒)
bool vehicle_speed_available # 本车实际车速有效性   (true:有效/false:无效)
float32 vehicle_speed_display # 本车仪表车速         (米/秒)
bool vehicle_speed_display_available # 本车仪表车速有效性   (true:有效/false:无效)
uint32 vehicle_speed_direction # 本车车速方向         (0:前进/1:后退)
bool vehicle_speed_direction_available # 本车车速方向有效性   (true:有效/false:无效)
float32 fl_wheel_speed # 前左轮速             (米/秒)
bool fl_wheel_speed_available # 前左轮速有效性       (true:有效/false:无效)
float32 fr_wheel_speed # 前右轮速             (米/秒)
bool fr_wheel_speed_available # 前右轮速有效性       (true:有效/false:无效)
float32 rl_wheel_speed # 后左轮速             (米/秒)
bool rl_wheel_speed_available # 后左轮速有效性       (true:有效/false:无效)
float32 rr_wheel_speed # 后右轮速             (米/秒)
bool rr_wheel_speed_available # 后右轮速有效性       (true:有效/false:无效)
uint32 fl_wheel_speed_pulse # 前左轮速脉冲
bool fl_wheel_speed_pulse_available # 前左轮速脉冲有效性   (true:有效/false:无效)
uint32 fr_wheel_speed_pulse # 前右轮速脉冲
bool fr_wheel_speed_pulse_available # 前右轮速脉冲有效性   (true:有效/false:无效)
uint32 rl_wheel_speed_pulse # 后左轮速脉冲
bool rl_wheel_speed_pulse_available # 后左轮速脉冲有效性   (true:有效/false:无效)
uint32 rr_wheel_speed_pulse # 后右轮速脉冲
bool rr_wheel_speed_pulse_available # 后右轮速脉冲有效性   (true:有效/false:无效)
uint32 fl_wheel_speed_direction # 前左轮速方向         (0:前进/1:后退)
bool fl_wheel_speed_direction_available # 前左轮速方向有效性   (true:有效/false:无效)
uint32 fr_wheel_speed_direction # 前右轮速方向         (0:前进/1:后退)
bool fr_wheel_speed_direction_available # 前右轮速方向有效性   (true:有效/false:无效)
uint32 rl_wheel_speed_direction # 后左轮速方向         (0:前进/1:后退)
bool rl_wheel_speed_direction_available # 后左轮速方向有效性   (true:有效/false:无效)
uint32 rr_wheel_speed_direction # 后右轮速方向         (0:前进/1:后退)
bool rr_wheel_speed_direction_available # 后右轮速方向有效性   (true:有效/false:无效)
float32 steering_wheel_angle 
bool steering_wheel_angle_available # 本车方向盘转角有效性 (true:有效/false:无效)
float32 steering_wheel_angle_speed 
bool steering_wheel_angle_speed_available # 本车方向盘转速有效性 (true:有效/false:无效)
float32 yaw_rate 
bool yaw_rate_available # 本车横摆角速度有效性 (true:有效/false:无效)
float32 long_acceleration 
bool long_acceleration_available # 本车纵向加速度有效性 (true:有效/false:无效)
float32 lat_acceleration 
bool lat_acceleration_available # 本车横向加速度有效性     (true:有效/false:无效)
float32 accelerator_pedal_pos # 实际加速踏板开度百分比   [0-100]
bool accelerator_pedal_pos_available # 实际加速踏板开度有效性   (true:有效/false:无效
float32 brake_pedal_pos 
bool brake_pedal_pos_available # 实际制动踏板开度有效性   (true:有效/false:无效)
bool brake_pedal_pressed # 制动踏板是否被驾驶员踩下状态         (true:被踩下/false:未踩下)
bool brake_pedal_pressed_available # 制动踏板是否被驾驶员踩下状态有效性    (true:有效/false:无效)
uint32 epb_state 
bool epb_state_available # EPB系统状态有效性        (true:有效/false:无效)
uint32 auto_hold_state # 自动驻车系统状态         (0:Not Active/1:Active)
bool auto_hold_state_available # 自动驻车系统状态有效性   (true:有效/false:无效)
uint32 shift_lever_state 
bool shift_lever_state_available # 驾驶员操作的换档杆状态有效性     (true:有效/false:无效)
uint32 gear_lever_state # 变速箱实际挡位           <数值表示档位数值，0表示Invalid>
bool gear_lever_state_available # 变速箱实际挡位有效性     (true:有效/false:无效)
uint32 turn_switch_state # 转向开关状态           (0: OFF 1: Left 2: Right)
bool turn_switch_state_available # 转向开关状态有效性     (true:有效/false:无效)
bool left_turn_light_state 
bool left_turn_light_state_available # 左转向灯状态有效性       (true:有效/false:无效)
bool right_turn_light_state 
bool right_turn_light_state_available # 右转向灯状态有效性       (true:有效/false:无效)
bool hazard_light_state 
bool hazard_light_state_available # 双闪状态有效性           (true:有效/false:无效)
float32 driver_hand_torque 
bool driver_hand_torque_available # 驾驶员手力矩有效性       (true:有效/false:无效)
bool driver_hands_off_state # 驾驶员脱手状态           (true:HandsOff/false:HandsOn)
bool driver_hands_off_state_available # 驾驶员脱手状态有效性     (true:有效/false:无效)
bool door_lock_state # 门锁状态 (false:unlock  true:lock)
bool door_lock_state_available # 门锁状态有效性(true:有效/false:无效)
bool fl_door_state # 前左车门状态             (true:On/false:Off)
bool fl_door_state_available # 前左车门状态有效性       (true:有效/false:无效)
bool fr_door_state # 前右车门状态             (true:On/false:Off)
bool fr_door_state_available # 前右车门状态有效性       (true:有效/false:无效)
bool rl_door_state # 后左车门状态             (true:On/false:Off)
bool rl_door_state_available # 后左车门状态有效性       (true:有效/false:无效)
bool rr_door_state # 后右车门状态             (true:On/false:Off)
bool rr_door_state_available # 后右车门状态有效性       (true:有效/false:无效)
bool hood_state # 前舱盖状态               (true:On/false:Off)
bool hood_state_available # 前舱盖状态有效性         (true:有效/false:无效)
bool trunk_door_state # 后备箱状态               (true:On/false:Off)
bool trunk_door_state_available # 后备箱状态有效性         (true:有效/false:无效)
bool fl_seat_belt_state # 前左安全带状态           (true:On/false:Off)
bool fl_seat_belt_state_available # 前左安全带状态有效性     (true:有效/false:无效)
bool fr_seat_belt_state # 前右安全带状态           (true:On/false:Off)
bool fr_seat_belt_state_available # 前右安全带状态有效性     (true:有效/false:无效)
bool rl_seat_belt_state # 后左安全带状态           (true:On/false:Off)
bool rl_seat_belt_state_available # 后左安全带状态有效性     (true:有效/false:无效)
bool rm_seat_belt_state # 后中安全带状态           (true:On/false:Off)
bool rm_seat_belt_state_available # 后中安全带状态有效性     (true:有效/false:无效)
bool rr_seat_belt_state # 后右安全带状态           (true:On/false:Off)
bool rr_seat_belt_state_available # 后右安全带状态有效性     (true:有效/false:无效)
uint32 wiper_state 
bool wiper_state_available # 雨刮状态有效性           (true:有效/false:无效)
bool front_fog_light_state # 前雾灯状态               (true:On/false:Off)
bool front_fog_light_state_available # 前雾灯状态有效性         (true:有效/false:无效)
bool rear_fog_light_state # 后雾灯状态               (true:On/false:Off)
bool rear_fog_light_state_available # 后雾灯状态有效性         (true:有效/false:无效)
bool auto_light_state # 自动灯光控制状态         (true:On/false:Off)
bool auto_light_state_available # 自动灯光控制状态有效性    (true:有效/false:无效)
bool low_beam_state # 近光灯状态               (true:On/false:Off)
bool low_beam_state_available # 近光灯状态有效性         (true:有效/false:无效)
bool high_beam_state # 远光灯状态               (true:On/false:Off)
bool high_beam_state_available # 远光灯状态有效性         (true:有效/false:无效)
float32 left_solar_sensor_value # 左光照传感器值           (瓦/平方米  W/m^2)
bool left_solar_sensor_value_available # 左光照传感器值有效性     (true:有效/false:无效)
float32 right_solar_sensor_value # 右光照传感器值           (瓦/平方米  W/m^2)
bool right_solar_sensor_value_available # 右光照传感器值有效性     (true:有效/false:无效)
bool esp_active # 车身稳定性控制功能激活状态           (true:Active/false:Inactive)
bool esp_active_available # 车身稳定性控制功能激活状态有效性      (true:有效/false:无效)
bool abs_active # 车轮防抱死控制功能激活状态           (true:Active/false:Inactive)
bool abs_active_available # 车轮防抱死控制功能激活状态有效性      (true:有效/false:无效)
bool tcs_active # 牵引力控制功能激活状态               (true:Active/false:Inactive)
bool tcs_active_available # 牵引力控制功能激活状态有效性         (true:有效/false:无效)
float32 power_train_current_torque # 动力系统当前输出扭矩                 (牛米 Nm)
bool power_train_current_torque_available # 动力系统当前输出扭矩有效性           (true:有效/false:无效)
float32 power_train_allow_torque_max # 动力系统当前允许请求的最大扭矩        (牛米 Nm)
bool power_train_allow_torque_max_available # 动力系统当前允许请求的最大扭矩有效性  (true:有效/false:无效)
float32 power_train_allow_torque_min # 动力系统当前允许请求的最小扭矩        (牛米 Nm)
bool power_train_allow_torque_min_available # 动力系统当前允许请求的最小扭矩有效性  (true:有效/false:无效)
bool power_train_override_flag # 动力系统超控标志位                   (true:Override/false:Not Override)
bool power_train_override_flag_available # 动力系统超控标志位有效性             (true:有效/false:无效)
uint32 pilot_long_control_actuator_status 
bool pilot_long_control_actuator_status_available # 行车纵向控制执行器状态有效性         (true:有效/false:无效)
uint32 pilot_lat_control_actuator_status 
bool pilot_lat_control_actuator_status_available # 行车横向控制执行器状态有效性         (true:有效/false:无效)
uint32 aeb_actuator_status 
bool aeb_actuator_status_available # AEB执行器状态有效性                 (true:有效/false:无效)
uint32 parking_long_control_actuator_status 
bool parking_long_control_actuator_status_available # 泊车纵向控制执行器状态有效性         (true:有效/false:无效)
uint32 parking_lat_control_actuator_status 
bool parking_lat_control_actuator_status_available # 泊车横向控制执行器状态有效性         (true:有效/false:无效)
float32 year # 年
float32 month # 月
float32 day # 日
float32 hour # 时
float32 minute # 分
float32 second # 秒
float32 total_odometer # 总里程       (公里)
float32 ext_temperature # 外部温度 (℃)
bool ext_temperature_available # 外部温度有效性 (true:有效/false:无效)
float32 steering_wheel_angle_bias
float32 yaw_rate_offset