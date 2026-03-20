int64 track_id # 该点处lane_id
Point2d car_point # 参考点在自车坐标系下坐标     (米)
Point3d enu_point # 参考点在ENU坐标系下坐标      (米)
Point3d local_point # 参考点在局部坐标系下坐标     (米)
float32 curvature # 参考点处曲率
float32 car_heading # 参考点处自车坐标系下航向角   (弧度rad)
float32 enu_heading # 参考点处ENU坐标系下航向角    (弧度rad)
float32 local_heading # 参考点处LOCAL坐标系下航向角  (弧度rad)
float32 distance_to_left_road_border # 参考点到道路左边缘距离       (米)
float32 distance_to_right_road_border # 参考点到道路右边缘距离       (米)
float32 distance_to_left_lane_border # 参考点到车道左车道线距离     (米)
float32 distance_to_right_lane_border # 参考点到车道右车道线距离     (米)
float32 lane_width # 参考点处车道宽度             (米)
float32 speed_limit_max # 参考点处最高限速             (米/秒)
float32 speed_limit_min # 参考点处最低限速             (米/秒)
int32 left_road_border_type # 参考点处左道路边缘类型
int32 right_road_border_type # 参考点处右道路边缘类型
int32 left_lane_border_type # 参考点处左车道线类型
int32 right_lane_border_type # 参考点处右车道线类型
bool is_in_intersection # 参考点是否在路口             (true:路口 / false:不在路口)
int32 lane_type # 参考点处车道类型
float64 s # 参考点距离起点的累计距离