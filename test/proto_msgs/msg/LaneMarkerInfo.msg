uint32 track_id    # uint8
float32 confidence  # uint8 置信度 0.0-1.0
uint32 measuring_status   # uint8 
uint32 type_class    # uint8  车道线类型
uint32 color       # uint8  颜色
uint32 quality     # uint8
uint32 crossing    # uint8
uint32 width_marking    # uint8
float32 view_range_start     # 起点
float32 view_range_end       # 终点
float32 coeff_a0   # 车道线方程常数项 y = a3 * x^3 + a2 * x^2 + a1 * x^1 + a0;
float32 coeff_a1   # 车道线方程一次项
float32 coeff_a2   # 车道线方程二次项
float32 coeff_a3   # 车道线方程三次项
float32 tlc 
int32 pos_type   #车道线位置