uint32 id                          # 障碍物id
int32 type                    # 障碍物类型 enum ObjectType
Shape3d shape                      # 障碍物尺寸
Point2f relative_position          # 障碍物近侧距离自车后轴中心的距离,(米)
Point2f relative_velocity          # 障碍物相对自车坐标系相对速度,(米/秒)
Point2f relative_acceleration      # 障碍物相对自车坐标系相对加速度,(米/秒^2)
Point2f relative_center_position   # 障碍物几何中心距离自车后轴中心的距离,(米)
float32 relative_heading_angle       # 障碍物与自车的相对朝向角,(弧度rad)
float32 relative_heading_angle_rate  # 障碍物与自车的相对朝向角变化率,(弧度rad/秒)
Point2f position                  # 障碍物距自车近侧在绝对坐标系中的位置,(米)
Point2f velocity                  # 障碍物在绝对坐标系中的速度,(米/秒)
Point2f acceleration              # 障碍物在绝对坐标系中的加速度,(米/秒^2)
Point2f center_position           # 障碍物几何中心在绝对坐标系的位置,(米)
float32 heading_angle               # 障碍物在绝对坐标系中的朝向角,(弧度rad)
float32 heading_angle_rate          # 障碍物在绝对坐标系中的朝向角变化率,(弧度rad/秒)