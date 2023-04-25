uint32 Id              # 识别车位ID号
Pos2DType[] CornerPoints     # 单个车位角点信息[4]
float32 LineA 	                # 车位入口A斜率
float32 LineB	                # 车位入口B斜率
int8 ParkingFusionSlotType  # enum ParkingFusionSlotType Type
bool Update_flag          # 车位更新标志位 //新增0131