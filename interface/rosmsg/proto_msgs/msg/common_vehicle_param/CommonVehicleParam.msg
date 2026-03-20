string project # 项目id
string car_type # 应用车型

float64 vehicle_weight_curb # 整备质量         (kg千克)
float64 vehicle_weight_gross # 总质量           (kg千克)
float64 vehicle_length # 车辆总长度       (米)
float64 vehicle_width # 车辆总宽度       (米)
float64 vehicle_height # 车辆总高度       (米)
float64 wheel_base # 轴距             (米)
float64 front_wheel_distance # 前轮距           (米)
float64 rear_wheel_distance # 后轮距           (米)
float64 lon_distance_from_front_edge_to_rear_axis # 车辆前保到后轴中心的纵向距离     (米)
float64 lon_distance_from_back_edge_to_rear_axis # 车辆后保到后轴中心的纵向距离     (米)
float64 lat_distance_from_left_edge_to_rear_axis # 车辆左端到后轴中心的横向距离     (米)
float64 lat_distance_from_right_edge_to_rear_axis # 车辆右端到后轴中心的横向距离     (米)
float64 lat_distance_from_front_bumper_to_front_axle # 车辆前保到前轴中心的纵向距离     (米)

float64 turn_radius_min # 最小转弯半径             (米)
float64 steer_angle_max # 方向盘最大转向角         (rad)
float64 steer_angle_rate_max # 方向盘最大转向角速度     (rad/s)
float64 front_wheel_max_angle # 前轮最大转角             (rad)
float64 steer_ratio # 转向器比率

float64 acceleration_max # 最大加速度       (m/s^2)
float64 deceleration_max # 最大减速度       (m/s^2)
float64 front_wheel_rolling_radius # 前车轮滚动半径   (米)
float64 rear_wheel_rolling_radius # 后车轮滚动半径   (米)
float64 abs_max_speed_when_stopped # 车辆静止时,abs测量的最大速度     (m/s)
float64 brake_deadzone # 制动死区         (米)
float64 throttle_deadzone # 节气门死区       (米)
float64 rolling_resistance_coeff # 滚动阻力系数
float64 air_drag_coeff # 空气阻力系数
float64 windward_area # 迎风面积         (平方米m^2)
float64 equivalent_mass_coeff # 等效质量系数

float64 front_shrink_distance # 泊车前边收缩距离         (米)
float64 front_side_shrink_distance # 泊车前方侧边收缩距离     (米)
float64 rear_shrink_distance # 泊车后边收缩距离         (米)
float64 rear_side_shrink_distance # 泊车后方侧边收缩距离     (米)