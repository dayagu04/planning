uint64 timestamp    # 时间戳
uint8 radar_type # 角雷达安装位置
uint32 obj_report_cnt
uint32 num_obstacles  # 雷达检测障碍物数量（无法给出障碍物信息时赋0）
Radar_OBJ_TYPE_T[] obj_type  #角雷达检测到的障碍物信息
uint32 num_roads    # 雷达检测道路线数量（无法给出道路信息时赋0）
Radar_ROAD_TYPE_T[] road_type  #角雷达检测到的金属护栏信息