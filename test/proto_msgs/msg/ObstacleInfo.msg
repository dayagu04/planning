uint32 type                              # 障碍物类型
uint32 sensor_type                       # 检测到该障碍物的传感器类型
int32 id                                  # 障碍 id
float32 conf                            # 障碍物置信度
int32 life_time                           # 生命周期，单位:ms
int32 age                                 # 障碍物存在的帧数
int32 is_fusion                           # 如果 if_fusion为  true,表示该障碍物为融合得来，否则不是
ObstacleImageSpaceInfo image_space_info   # 图像空间对应信息
ObstacleWorldSpaceInfo world_space_info   # 物理空间对应信息
ObstacleAuxiliaries aux                   # 附加信息