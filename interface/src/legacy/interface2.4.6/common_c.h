// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_LEGACY_INTERFACE2_4_6_COMMON_H_INCLUDE_
#define _IFLYAUTO_LEGACY_INTERFACE2_4_6_COMMON_H_INCLUDE_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#ifndef _STRUCT_ALIGNED_
#define _STRUCT_ALIGNED_ __attribute__((aligned(4)))
#endif  // _STRUCT_ALIGNED_

#ifndef _ENUM_PACKED_
#define _ENUM_PACKED_ __attribute__((packed))
#endif  // _ENUM_PACKED_

#define PROTO_STR_LEN 64
#define INPUT_HISTORY_TIMESTAMP_MAX_NUM 16

#ifdef __cplusplus
namespace iflyauto {
  namespace interface_2_4_6 {
#endif  // __cplusplus

#pragma pack(4)

// 通用2维点
typedef struct {
  int32 x;
  int32 y;
} _STRUCT_ALIGNED_ Point2si;

typedef struct {
  float32 x;
  float32 y;
} _STRUCT_ALIGNED_ Point2f;

typedef struct {
  float64 x;
  float64 y;
} _STRUCT_ALIGNED_ Point2d;

// 通用3维点
typedef struct {
  int32 x;
  int32 y;
  int32 z;
} _STRUCT_ALIGNED_ Point3si;

typedef struct {
  float32 x;
  float32 y;
  float32 z;
} _STRUCT_ALIGNED_ Point3f;

typedef struct {
  float64 x;
  float64 y;
  float64 z;
} _STRUCT_ALIGNED_ Point3d;

// 立方体三维信息
typedef struct {
  float32 length;  // 长  (米)
  float32 width;   // 宽  (米)
  float32 height;  // 高  (米)
} _STRUCT_ALIGNED_ Shape3f;

typedef struct {
  float64 length;  // 长  (米)
  float64 width;   // 宽  (米)
  float64 height;  // 高  (米)
} _STRUCT_ALIGNED_ Shape3d;

// ENU信息
typedef struct {
  float64 x;  // East from the origin         (米)
  float64 y;  // North from the origin        (米)
  float64 z;  // Up from the WGS-84 ellipsoid (米)
} _STRUCT_ALIGNED_ PointENU;

// 地图瓦片标识id信息
typedef struct {
  /** id号
   *  备注：parking_map地图中只用16位
   **/
  uint32 id;
} _STRUCT_ALIGNED_ TileId;

// 城市标识id信息
typedef struct {
  /** id号
   *  备注：parking_map地图中只用16位
   **/
  uint32 id;
} _STRUCT_ALIGNED_ UrId;

// LLH信息
typedef struct {
  float64 lon;     // Longitude in degrees, ranging from -180 to 180  (度)
  float64 lat;     // Latitude in degrees, ranging from -90 to 90     (度)
  float64 height;  // WGS-84 ellipsoid height in meters               (米)
} _STRUCT_ALIGNED_ PointLLH;

// 旋转四元数
typedef struct {
  float64 w;  // 四元数的实部，标量部分
  float64 x;  // 四元数的虚部 i 分量
  float64 y;  // 四元数的虚部 j 分量
  float64 z;  // 四元数的虚部 k 分量
} _STRUCT_ALIGNED_ Quaternion;

// 时间戳统计信息
typedef enum {
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_PANORAMA_VIEW_CAMERA_NODE = 0,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_AROUND_VIEW_CAMERA_NODE,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_RADAR_FM_PERCEPTION,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_RADAR_FL_PERCEPTION,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_RADAR_FR_PERCEPTION,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_RADAR_RL_PERCEPTION,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_RADAR_RR_PERCEPTION,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_IMU,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_GNSS,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_USS_WAVE,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_VIHECLE_SERVICES,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_COM_SERVICE,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_HMI_SERVICE_MCU_INNER,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_HMI_SERVICE_SOC_INNER,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_SD_MAP_SERVICE,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_BSW_RESERVE1,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_BSW_RESERVE2,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_PANORAMA_VIEW_CAMERA_PERCEPTION_OBJ = 20,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_PANORAMA_VIEW_CAMERA_PERCEPTION_OCCUPANCY_OBJ,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_PANORAMA_VIEW_CAMERA_PERCEPTION_LANELINE,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_PANORAMA_VIEW_CAMERA_PERCEPTION_RESERVE1,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_PANORAMA_VIEW_CAMERA_PERCEPTION_RESERVE2,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_AROUND_VIEW_PERCEPTION,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_AROUND_VIEW_PERCEPTION_RESERVE1,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_AROUND_VIEW_PERCEPTION_RESERVE2,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_USS_PERCEPTION,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_USS_PERCEPTION_RESERVE1,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_OBSTACLE_FUSION = 40,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_MEGA,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_STATIC_FUSION,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_FUSION_RESERVE1,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_LOCALIZATION,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_LOCALIZATION_RESERVE1,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_PLANNING = 50,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_PLANNING_HMI,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_PREDICTION,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_CONTROL,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_STATE_MACHINE = 60,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_HMI_OUT ,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_HMI_RESERVE1,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_HMI_RESERVE2,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_EHR = 70,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_MAP,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_SYS_RESERVE1 = 100,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_SYS_RESERVE2,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_SYS_RESERVE3,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_SYS_RESERVE4,
  INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_SYS_RESERVE5,
} _ENUM_PACKED_ InputHistorySourceType;

typedef struct {
  // 输入数据类型
  InputHistorySourceType input_type;  // 数据来源模块
  uint64 seq;                                  // 接收数据的seq，用于离线排查整个链路的数据情况
} _STRUCT_ALIGNED_ InputHistoryInfo;

// 时间戳信息集合
typedef struct {
  uint64 front_isp_timestamp;                           // 前视相机的ISP出图时间戳    (微秒)
  uint64 rear_isp_timestamp;                            // 后视相机的ISP出图时间戳    (微秒)
  uint64 fl_isp_timestamp;                              // 左前相机的ISP出图时间戳    (微秒)
  uint64 rl_isp_timestamp;                              // 左后相机的ISP出图时间戳    (微秒)
  uint64 fr_isp_timestamp;                              // 右前相机的ISP出图时间戳    (微秒)
  uint64 rr_isp_timestamp;                              // 右后相机的ISP出图时间戳    (微秒)
  uint64 vehicle_service_timestamp;                     // 车身信号的时间戳           (微秒)
  uint64 pbox_imu_timestamp;                            // pbox_imu信号的时间戳       (微秒)
} _STRUCT_ALIGNED_ CommonCameraPerceptionInputTimestamp;

// 信息头结构
typedef struct {
  uint64 timestamp;  // 仅表示本次消息发送时刻的时间戳，不可表示某项数据产生时间戳     (微秒)
  uint64 start_timestamp;  // 仅表示模块本轮运算开始的时间     (微秒)
  uint8 version[PROTO_STR_LEN];  // 模块版本号
  uint8 input_list_size;
  /** 用于模块记录自己本轮收到数据的情况，便于在数据包中分析和查找关联数据
   *  举例：障碍物融合模块本轮运算收到相机障碍物、雷达障碍物、vs、定位，记录下收到的这四个数据的seq
   **/
  InputHistoryInfo input_list[INPUT_HISTORY_TIMESTAMP_MAX_NUM];
  /** 信息发送序号
   *  备注：从0开始，每次发topic计数+1，mcu产生的消息在mcu处计数，转发者不计数只透传
   **/
  uint64 seq;
} _STRUCT_ALIGNED_ Header;

// 通用障碍物类型
typedef enum {
  OBJECT_TYPE_UNKNOWN = 0,                // 未知障碍物
  OBJECT_TYPE_UNKNOWN_MOVABLE = 1,        // 未知可移动障碍物
  OBJECT_TYPE_UNKNOWN_IMMOVABLE = 2,      // 未知不可移动障碍物
  OBJECT_TYPE_COUPE = 3,                  // 轿车
  OBJECT_TYPE_MINIBUS = 4,                // 面包车
  OBJECT_TYPE_VAN = 5,                    // 厢式轿车
  OBJECT_TYPE_BUS = 6,                    // 大型客车
  OBJECT_TYPE_TRUCK = 7,                  // 卡车
  OBJECT_TYPE_TRAILER = 8,                // 拖车
  OBJECT_TYPE_BICYCLE = 9,                // 自行车
  OBJECT_TYPE_MOTORCYCLE = 10,            // 摩托车
  OBJECT_TYPE_TRICYCLE = 11,              // 三轮车
  OBJECT_TYPE_PEDESTRIAN = 12,            // 行人
  OBJECT_TYPE_ANIMAL = 13,                // 动物
  OBJECT_TYPE_TRAFFIC_CONE = 14,          // 交通锥
  OBJECT_TYPE_TRAFFIC_BARREL = 15,        // 隔离墩
  OBJECT_TYPE_TRAFFIC_TEM_SIGN = 16,      // 临时指示牌
  OBJECT_TYPE_FENCE = 17,                 // 栅栏
  OBJECT_TYPE_CYCLE_RIDING = 18,          // 人骑着自行车
  OBJECT_TYPE_MOTORCYCLE_RIDING = 19,     // 人骑着摩托车
  OBJECT_TYPE_TRICYCLE_RIDING = 20,       // 人骑着三轮车
  OBJECT_TYPE_WATER_SAFETY_BARRIER = 21,  // 水马
  OBJECT_TYPE_CTASH_BARREL = 22,          // 防撞桶
  //以上类型能给出
  //一下HMI预留，后续待感知给出
  OBJECT_TYPE_WARNING_TRIANGLE = 23,      // 三角牌
  OBJECT_TYPE_PARKING_GATE = 24,          // 停车杆
  OBJECT_TYPE_POLE = 25,                  // 杆
  OBJECT_TYPE_SPEED_BUMP = 26,            // 减速带
  OBJECT_TYPE_PARKING_SPOT_LOCK = 27,     // 地锁
  OBJECT_TYPE_SQUARE_COLUMN = 28,         // 方柱
  OBJECT_TYPE_CYLINDER = 29,              // 圆柱
  OBJECT_TYPE_SUV = 30,                   // SUV车辆
} _ENUM_PACKED_ ObjectType;

// 障碍物灯光状态
typedef enum {
  OBJECT_LIGHT_TYPE_UNKNOWN = 0,         // 无
  OBJECT_LIGHT_TYPE_TURN_LEFT = 1,       // 左转
  OBJECT_LIGHT_TYPE_TURN_RIGHT = 2,      // 右转
  OBJECT_LIGHT_TYPE_DOUBLE_FLASH = 3,    // 双闪
  OBJECT_LIGHT_TYPE_BRAKE = 4,           // 刹车灯
  OBJECT_LIGHT_TYPE_REVERSE = 5,         // 倒车灯
} _ENUM_PACKED_ ObjectLightType;

// 障碍物运动状态
typedef enum {
  OBJECT_MOTION_TYPE_UNKNOWN = 0,  // 未知
  OBJECT_MOTION_TYPE_STATIC = 1,   // 静止
  OBJECT_MOTION_TYPE_STOPED = 2,   // 停止
  OBJECT_MOTION_TYPE_MOVING = 3,   // 同向
  OBJECT_MOTION_TYPE_ONCOME = 4,   // 对向
  OBJECT_MOTION_TYPE_CROSS = 5,    // 横穿
} _ENUM_PACKED_ ObjectMotionType;

// 通用车位类型
typedef enum {
  PARKING_SLOT_TYPE_VERTICAL = 0,    // 垂直车位
  PARKING_SLOT_TYPE_HORIZONTAL = 1,  // 平行车位
  PARKING_SLOT_TYPE_SLANTING = 2,    // 斜车位
  PARKING_SLOT_TYPE_INVALID = 3,     // 不合法车位
} _ENUM_PACKED_ ParkingSlotType;

typedef enum {
  FUSION_SLOT_POSITION_TYPE_LEFT = 0,
  FUSION_SLOT_POSITION_TYPE_RIGHT = 1,
} _ENUM_PACKED_ ParkingSlotPositionType;

// 通用传感器类型
typedef enum {
  SENSOR_TYPE_UNKNOWN = 0,                 // 未知
  SENSOR_TYPE_FRONT_CAMERA = 1,            // 前视相机
  SENSOR_TYPE_REAR_CAMERA = 2,             // 后视相机
  SENSOR_TYPE_LEFT_FRONT_CAMERA = 3,       // 左前侧视相机
  SENSOR_TYPE_LEFT_REAR_CAMERA = 4,        // 左后侧视相机
  SENSOR_TYPE_RIHGT_FRONT_CAMERA = 5,      // 右前侧视相机
  SENSOR_TYPE_RIGHT_REAR_CAMERA = 6,       // 右后侧视相机
  SENSOR_TYPE_FRONT_SURROUND_CAMERA = 7,   // 前环视相机
  SENSOR_TYPE_REAR_SURROUND_CAMERA = 8,    // 后环视相机
  SENSOR_TYPE_LEFT_SURROUND_CAMERA = 9,    // 左环视相机
  SENSOR_TYPE_RIGHT_SURROUND_CAMERA = 10,  // 右环视相机
  SENSOR_TYPE_FRONT_RADAR = 11,            // 正前雷达
  SENSOR_TYPE_LEFT_FRONT_RADAR = 12,       // 左前雷达
  SENSOR_TYPE_RIGHT_FRONT_RADAR = 13,      // 右前雷达
  SENSOR_TYPE_LEFT_REAR_RADAR = 14,        // 左后雷达
  SENSOR_TYPE_RIGHT_REAR_RADAR = 15,       // 右后雷达
} _ENUM_PACKED_ SensorType;

// 通用更新标志类型
typedef enum {
  UPDATE_FLAG_INVALID = 0,    // 非法的
  UPDATE_FLAG_NEW = 1,        // 新创建的(前一帧中不存在，当前帧新出现的)
  UPDATE_FLAG_MEAS = 2,       // 测量的(前一帧中存在，当前帧也测量到)
  UPDATE_FLAG_PREDICTED = 3,  // 预测的(前一帧中存在，当前帧消失了，使用预测方式跟踪一段时间)
  UPDATE_FLAG_MERGED = 4,     // 聚类的，或融合的
} _ENUM_PACKED_ UpdateFlag;

// 通用障碍物结构
typedef struct {
  uint32 id;                          // 障碍物id
  ObjectType type;                   // 障碍物类型
  ObjectLightType light_type;        // 障碍物灯光状态
  Shape3d shape;                     // 障碍物尺寸
  Point3f relative_position;         // 障碍物近侧距离自车后轴中心的距离     (米)
  Point3f relative_velocity;         // 障碍物相对自车坐标系相对速度         (米/秒)
  Point3f relative_acceleration;     // 障碍物相对自车坐标系相对加速度 (米/秒^2)
  Point3f relative_center_position;  // 障碍物几何中心距离自车后轴中心的距离 (米)

  /* 障碍物与自车的相对朝向角
     单位：弧度rad
     备注：沿自车坐标系x轴正向逆时针旋转为正（0~Π），顺时针为负（0~-Π）
  */
  float32 relative_heading_angle;
  float32 relative_heading_angle_rate;  // 障碍物与自车的相对朝向角变化率
                                        // (弧度rad/秒)
  Point3f position;                     // 障碍物距自车近侧在绝对坐标系中的位置 (米)
  Point3f velocity;                     // 障碍物在绝对坐标系中的速度           (米/秒)
  Point3f acceleration;                 // 障碍物在绝对坐标系中的加速度         (米/秒^2)
  Point3f center_position;              // 障碍物几何中心在绝对坐标系的位置     (米)
  float32 heading_angle;                // 障碍物在绝对坐标系中的朝向角         (弧度rad)
  float32 heading_angle_rate;           // 障碍物在绝对坐标系中的朝向角变化率 (弧度rad/秒)
} _STRUCT_ALIGNED_ Obstacle;

// 规划轨迹类型
typedef enum {
  TRAJECTORY_TYPE_TARGET_REFERENCE = 0,   // 目标参考值跟踪
  TRAJECTORY_TYPE_TRAJECTORY_POINTS = 1,  // 规划轨迹点
} _ENUM_PACKED_ TrajectoryType;

// 规划期望方向盘转角档位
typedef enum {
  LATERAL_MANEUVER_GEAR_SLOW = 0,      // 缓慢
  LATERAL_MANEUVER_GEAR_NORMAL = 1,    // 正常
  LATERAL_MANEUVER_GEAR_FAST = 2,      // 快速
  LATERAL_MANEUVER_GEAR_ACCIDENT = 3,  // 原地快速
} _ENUM_PACKED_ LateralManeuverGear;

// 转向灯请求值
typedef enum {
  TURN_SIGNAL_TYPE_NONE = 0,             // 无请求
  TURN_SIGNAL_TYPE_LEFT = 1,             // 请求左转向
  TURN_SIGNAL_TYPE_RIGHT = 2,            // 请求右转向
  TURN_SIGNAL_TYPE_EMERGENCY_FLASH = 3,  // 双闪
} _ENUM_PACKED_ TurnSignalType;

// 灯光信号请求值
typedef enum {
  LIGHT_SIGNAL_TYPE_NONE = 0,       // 无请求
  LIGHT_SIGNAL_TYPE_HIGH_BEAM = 1,  // 远光灯
  LIGHT_SIGNAL_TYPE_LOW_BEAM = 2,   // 近光灯
} _ENUM_PACKED_ LightSignalType;

// 喇叭请求值
typedef enum {
  HORN_SIGNAL_TYPE_NONE = 0,     // 无请求
  HORN_SIGNAL_TYPE_LONG = 1,     // 长鸣
  HORN_SIGNAL_TYPE_SHORT = 2,    // 短鸣
  HORN_SIGNAL_TYPE_NARMAL = 3,   // 正常
  HORN_SIGNAL_TYPE_SEGMENT = 4,  // 分段鸣
} _ENUM_PACKED_ HornSignalType;

// 挡位请求信息
typedef enum {
  GEAR_COMMAND_VALUE_NONE = 0,     // 无挡位信息
  GEAR_COMMAND_VALUE_PARKING = 1,  // 驻车
  GEAR_COMMAND_VALUE_REVERSE = 2,  // 倒车
  GEAR_COMMAND_VALUE_NEUTRAL = 3,  // 空挡
  GEAR_COMMAND_VALUE_DRIVE = 4,    // 前进挡
  GEAR_COMMAND_VALUE_LOW = 5,      // 低速挡
} _ENUM_PACKED_ GearCommandValue;

// 车道线颜色类型
typedef enum {
  LaneLineColor_COLOR_UNKNOWN = 0,                /* 未知 */
  LaneLineColor_COLOR_WHITE = 1,                  /* 白色 */
  LaneLineColor_COLOR_YELLOW = 2,                 /* 黄色 */
  LaneLineColor_COLOR_BLUE = 3,                   /* 蓝色 */
  LaneLineColor_COLOR_GREEN = 4,                  /* 绿色 */
  LaneLineColor_COLOR_GRAY = 5,                   /* 灰色 */
  LaneLineColor_COLOR_ORANGE = 6,                 /* 橘色 */
  LaneLineColor_COLOR_LEFT_GRAY_RIGHT_YELLOW = 7, /* 左侧车道线灰色，右侧车道线黄色 */
  LaneLineColor_COLOR_LEFT_YELLOW_RIGHT_WHITE = 8,/* 左侧车道线黄色，右侧车道线白色 */
  LaneLineColor_COLOR_RED = 9,                    /* 红色 */
  LaneLineColor_COLOR_RAINBOW = 10,               /* 彩色 */
} _ENUM_PACKED_ LaneLineColor;

// 车道线虚实类型
typedef enum {
  LaneBoundaryType_MARKING_UNKNOWN = 0,                 /* 未知线型 */
  LaneBoundaryType_MARKING_DASHED = 1,                  /* 虚线 */
  LaneBoundaryType_MARKING_SOLID = 2,                   /* 实线 */
  LaneBoundaryType_MARKING_SHORT_DASHED = 3,            /* 短虚线 */
  LaneBoundaryType_MARKING_DOUBLE_DASHED = 4,           /* 双虚线 */
  LaneBoundaryType_MARKING_DOUBLE_SOLID = 5,            /* 双实线 */
  LaneBoundaryType_MARKING_LEFT_DASHED_RIGHT_SOLID = 6, /* 左虚右实线 */
  LaneBoundaryType_MARKING_LEFT_SOLID_RIGHT_DASHED = 7, /* 左实右虚线 */
  LaneBoundaryType_MARKING_DECELERATION = 8,            /* 减速线 */
  LaneBoundaryType_MARKING_VIRTUAL = 9,                 /* 虚拟线 */
  LaneBoundaryType_MARKING_WAITLINE = 10,               /* 待转线 */
} _ENUM_PACKED_ LaneBoundaryType;

// 车道转向类型
typedef enum {
  LaneDrivableDirection_DIRECTION_UNKNOWN = 0,                // 未知
  LaneDrivableDirection_DIRECTION_STRAIGHT = 1,               // 直行
  LaneDrivableDirection_DIRECTION_STRAIGHT_RIGHT = 2,         // 直行加右转
  LaneDrivableDirection_DIRECTION_RIGHT = 3,                  // 右转
  LaneDrivableDirection_DIRECTION_STRAIGHT_LEFT = 4,          // 直行加左转
  LaneDrivableDirection_DIRECTION_LEFT = 5,                   // 左转
  LaneDrivableDirection_DIRECTION_UTURN_LEFT = 6,             // 调头加左转
  LaneDrivableDirection_DIRECTION_UTURN_RIGHT = 7,            // 调头加右转
  LaneDrivableDirection_DIRECTION_STRAIGHT_LEFT_RIGHT = 8,    // 直行加左转加右转
  LaneDrivableDirection_DIRECTION_STRAIGHT_UTURN_LEFT = 9,    // 直行加左调头
  LaneDrivableDirection_DIRECTION_STRAIGHT_UTURN_RIGHT = 10,  // 直行加右调头
  LaneDrivableDirection_DIRECTION_STRAIGHT_OFF_ROUTE = 11,    // 直行加左转加左掉头
  LaneDrivableDirection_DIRECTION_LEFT_UTURN = 12,            // 左调头
  LaneDrivableDirection_DIRECTION_RIGHT_UTURN = 13,           // 右调头
  LaneDrivableDirection_DIRECTION_NO_UTURN = 14,              // 禁止调头
  LaneDrivableDirection_DIRECTION_LEFT_RIGHT = 15,            // 左转加右转
  LaneDrivableDirection_DIRECTION_LEFT_MERGE = 16,            // 左侧汇流
  LaneDrivableDirection_DIRECTION_RIGHT_MERGE = 17,           // 右侧汇流
  LaneDrivableDirection_DIRECTION_CROSS_LINE = 18,            // 人行横道
} _ENUM_PACKED_ LaneDrivableDirection;

// 车道分合流方向类型
typedef enum {
  LaneOrientation_ORIENTATION_UNKNOWN = 0,
  LaneOrientation_ORIENTATION_LEFT = 1,
  LaneOrientation_ORIENTATION_RIGHT = 2,
} _ENUM_PACKED_ LaneOrientation;

// 车道分合流信息
typedef struct {
  float32 distance;             // 自车后轴中心到该分合流点起始位置距离     (米)
  float32 length;               // 该分合流长度                           (米)
  boolean is_split;             // 该参考线是否为分流                     (true:yes / false:no)
  boolean is_continue;          // 该参考线是否占路权                     (true:yes / false:no)
  LaneOrientation orientation;  // 车道分合流方向
  Point3d point;                // 分合流点
} _STRUCT_ALIGNED_ LaneMergeSplitPointData;

// 接地线类型
typedef enum {
  GROUND_LINE_TYPE_UNKNOWN,  // 未知类型接地线
  GROUND_LINE_TYPE_WALL,     // 墙面接地线
  GROUND_LINE_TYPE_COLUMN,   // 立柱接地线
  GROUND_LINE_TYPE_FENCE,    // 围栏接地线
  GROUND_LINE_TYPE_STEP,     // 楼梯接地线
  GROUND_LINE_TYPE_CURB,     // 路沿接地线
  GROUND_LINE_TYPE_SPECIAL,  // 特殊建筑接地线
} _ENUM_PACKED_ GroundLineType;

// UTC时间数据结构
typedef struct {
  uint32 year;           // 年
  uint32 month;          // 月
  uint32 day;            // 日
  uint32 hour;           // 时
  uint32 minute;         // 分
  uint32 second;         // 秒
  uint32 milli_second;   // 毫秒
  uint32 time_accuracy;  // 时间精度     [0-999]
} _STRUCT_ALIGNED_ UtcTime;

// 车道线类型
typedef enum {
  LINE_TYPE_UNKNOWN = 0,              // 未知线型
  LINE_TYPE_LANELINE = 1,             // 车道线
  LINE_TYPE_BORDERLINE = 2,           // 边沿线，物理隔离障碍物统称
  LINE_TYPE_CENTER = 3,               // 双向道路的道路中心线
  LANE_LINE_TYPE_GUARDRAIL = 4,       // 护栏
  LANE_LINE_TYPE_CONCRETEBARRIER = 5, // 水泥护栏
  LANE_LINE_TYPE_FENCE = 6,           // 栅栏
  LANE_LINE_TYPE_WALL = 7,            // 墙
  LANE_LINE_TYPE_CANOPY = 8,          // 绿植
  LANE_LINE_TYPE_CONE = 9,            // 锥形桶
  LANE_LINE_TYPE_STOPLINE = 10,       // 停止线
  LANE_LINE_TYPE_INHIBITLINE = 11,    // 禁止线
} _ENUM_PACKED_ LineType;

// 车道类型
typedef enum {
  LANETYPE_UNKNOWN = 0,                   // 未知类型
  LANETYPE_NORMAL = 1,                    // 常规车道
  LANETYPE_VIRTUAL = 2,                   // 虚拟车道
  LANETYPE_PARKING = 3,                   // 车位线
  LANETYPE_ACCELERATE = 4,                // 加速车道
  LANETYPE_DECELERATE = 5,                // 减速车道
  LANETYPE_BUS = 6,                       // 公交车道
  LANETYPE_EMERGENCY = 7,                 // 应急车道
  LANETYPE_ACCELERATE_DECELERATE = 8,     // 加减速车道
  LANETYPE_LEFT_TURN_WAITTING_AREA = 9,   // 左转弯待转区
  LANETYPE_NON_MOTOR = 10,                // 非机动车道
  LANETYPE_HOV = 11,                      // HOV
  LANETYPE_SLOW = 12,                     // 慢车道
  LANETYPE_SHOULDER = 13,                 // 路肩
  LANETYPE_DRIVABLE_SHOULDER = 14,        // 可行驶路肩
  LANETYPE_CONTROL = 15,                  // 管制车道
  LANETYPE_EMERGENCY_PARKING_STRIP = 16,  // 紧急停车带
  LANETYPE_TIDE = 17,                     // 潮汐车道
  LANETYPE_DIRECTION_CHANGE = 18,         // 可变车道
  LANETYPE_DRIVABLE_PARKING_ROAD = 19,    // 可行驶停车道
  LANETYPE_TRUCK = 20,                    // 货车专用道
  LANETYPE_TIME_LIMIT_BUS = 21,           // 限时公交车道
  LANETYPE_PASSING_BAY = 22,              // 错车道
  LANETYPE_REVERSAL_LEFT_TURN = 23,       // 借道左转车道
  LANETYPE_TAXI = 24,                     // 出租车车道
  LANETYPE_DIRECTION_LOCKED = 25,         // 定向车道
  LANETYPE_VEHICLE_BICYCLE_MIX = 26,      // 机非混合车道
  LANETYPE_MOTOR = 27,                    // 摩托车道
  LANETYPE_U_TURN = 28,                   // 掉头车道
  LANETYPE_TOLL = 29,                     // 收费站车道
  LANETYPE_CHECK_POINT = 30,              // 检查站车道
  LANETYPE_DANGEROUS_ARTICLE = 31,        // 危险品专用车道
  LANETYPE_FORBIDDEN_DRIVE = 32,          // 非行驶区域
  LANETYPE_THOUGH_LANE_ZONE = 33,         // 借道区
  LANETYPE_STREET_RAILWAY = 34,           // 有轨电车车道
  LANETYPE_BUS_BAY = 35,                  // 公交港湾车道
  LANETYPE_SPECIAL_CAR = 36,              // 特殊车辆专用车道
  LANETYPE_PEDESTRIANS = 37,              // 人行道
  LANETYPE_ETC = 38,                      // ETC车道
  LANETYPE_RAMP = 39,                     // 斜坡
  LANETYPE_RIGHT_TURN_WAITTING_AREA = 40, // 右转弯待转区
  LANETYPE_BRANCH = 41,                   // 岔道
} _ENUM_PACKED_ LaneType;

// 车道通行方向信息
typedef struct {
  float64 begin;                      // 起点             (米)
  float64 end;                        // 终点             (米)
  LaneDrivableDirection lane_mark;    // 车道转向类型
} _STRUCT_ALIGNED_ LaneMarkMsg;

// 车道类型信息
typedef struct {
  float64 begin;                      // 起点             (米)
  float64 end;                        // 终点             (米)
  LaneType type;                      // 车道类型
} _STRUCT_ALIGNED_ LaneTypeMsg;

// 通用车位数据来源
typedef enum {
  SLOT_SOURCE_TYPE_INVALID = 0,            // 无效的
  SLOT_SOURCE_TYPE_ONLY_CAMERA = 1,        // 仅相机
  SLOT_SOURCE_TYPE_ONLY_USS = 2,           // 仅超声波雷达
  SLOT_SOURCE_TYPE_CAMERA_USS = 3,         // 相机和超声波雷达融合
} _ENUM_PACKED_ SlotSourceType;

#pragma pack()
#ifdef __cplusplus
  }  // namespace interface_2_4_6
}  // namespace iflyauto
#endif  // __cplusplus

#endif  //_IFLYAUTO_LEGACY_INTERFACE2_4_6_COMMON_H_INCLUDE_
