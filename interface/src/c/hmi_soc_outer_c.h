// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2024/03/29

#ifndef _IFLYAUTO_HMI_SOC_OUTER_H_
#define _IFLYAUTO_HMI_SOC_OUTER_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"
#include "hmi_inner_c.h"
#include "planning_hmi_c.h"
#include "planning_plan_c.h"
#include "fusion_parking_slot_c.h"
#include "vehicle_service_c.h"
#include "fusion_objects_c.h"
#include "func_state_machine_c.h"
#include "ifly_parking_map_c.h"
#include "fusion_road_c.h"
#include "camera_perception_lane_lines_c.h"
#include "camera_perception_tsr_c.h"
#include "camera_perception_groundline_c.h"
#include "camera_perception_deceler_c.h"
#include "ifly_localization_c.h"

#define HMI_TOPO_LINE_MAX_NUM 4              // 2条可变车道的车道线数量
#define HMI_GROUND_MARKING_POINTS_NUM 4      // 每个地面标识4个点
#define HMI_SENSOR_INFO_MAX_NUM 15           // 传感器数量
#define HMI_HPP_HIS_TRAJ_POINT_MAX_NUM 60    // hpp历史轨迹点
#define HMI_PA_DIRECTION_AVAILABLE_MAX_NUM 2 // 一键贴边的可选方向个数
#define HMI_FREE_SPACE_VOXEL_SIZE 40000      // 可行使区域的栅格大小 200 * 200

#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)

typedef struct {
  uint8_t point_size;                             // 规划轨迹点数量
  Point2f point[PLANNING_TRAJ_POINTS_MAX_NUM];    // 规划轨迹点信息
} _STRUCT_ALIGNED_ TrajectoryPointSet;

// 转向灯状态
typedef enum {
  TURNLIGHTSTATE_OFF = 0,
  TURNLIGHTSTATE_LEFT = 1,           // 左转
  TURNLIGHTSTATE_RIGHT = 2,          // 右转
  TURNLIGHTSTATE_LEFT_AND_RIGHT = 3, // 双闪
} _ENUM_PACKED_ TurnLightState;

// 通用信息提醒
typedef enum {
  COMMON_NOTIFY_NO_REQ = 0,
  COMMON_NOTIFY_SHARP_TURN = 1,                           // 急转弯提醒
  COMMON_NOTIFY_BAD_WEATHER = 2,                          // 天气提醒
  COMMON_NOTIFY_SENSOR_POSITION_DEVIATION_TOO_LARGE = 3,  // 传感器位置偏差过大，功能无法激活
  COMMON_NOTIFY_SENSOR_SEVERE_DEVIATION = 4,              // 传感器偏差严重，请联系售后
  COMMON_NOTIFY_SENSOR_REPLACED = 5,                      // 传感器被更换，请联系售后
  COMMON_NOTIFY_CONFIG_FILE_TAMPERED = 6,                 // 配置文件被篡改，请联系售后
  COMMON_NOTIFY_CALIBRATION_FILE_ERROR = 7,               // 标定文件错误，功能无法激活
  COMMON_NOTIFY_EMERGENCY_STOP_REQUEST = 8,               // 安全停车提醒
  COMMON_NOTIFY_FOLLOW_TRAFFIC_SPEED = 9,                 // 按设定速度巡航
  COMMON_NOTIFY_RED_LIGHT_STOP = 10,                      // 闯红灯风险提醒
  COMMON_NOTIFY_GREEN_LIGHT_START = 11,                   // 绿灯未起步提醒
  COMMON_NOTIFY_LEAD_VEHICLE_STARTING = 12,               // 前车起步提醒
} _ENUM_PACKED_ CommonNotifyReq;

// 功能按钮状态
typedef enum {
  FUNC_BUTTON_NONE = 0,        // 不显示
  FUNC_BUTTON_UNAVAILABLE = 1, // 置灰
  FUNC_BUTTON_AVAILABLE = 2,   // 可点击
  FUNC_BUTTON_ACTIVE = 3,      // 激活状态
} _ENUM_PACKED_ FuncButtonState;

// 通用的接管请求原因
typedef enum {
  ADAS_TAKEOVER_NONE = 0,
  ADAS_TAKEOVER_GENERAL = 1,                      // 通用提示
  ADAS_TAKEOVER_EMERGENCY_DECELERATION = 2,       // 急减速，可能刹不住接管提醒，包含红灯停止线
  ADAS_TAKEOVER_HANDS_OFF_OR_DISTRACTION = 3,     // 脱手或分心接管
  ADAS_TAKEOVER_ACC_BRAKE_ONLY = 4,               // ACC激活BREAK ONLY接管
  ADAS_TAKEOVER_MERGE_SPLIT_NO_CONDITION = 5,     // 合流或分流无变道条件或变道失败接管
  ADAS_TAKEOVER_CONE_ZONE = 6,                    // 施工区域接管
  ADAS_TAKEOVER_TOLL_STATION = 7,                 // 收费站接管
  ADAS_TAKEOVER_NONE_ODD_AREA = 8,                // ODD外区域接管
  ADAS_TAKEOVER_TURN_LANE = 9,                    // 左右转车道接管
  ADAS_TAKEOVER_ROUNDABOUT = 10,                  // 环岛接管
  ADAS_TAKEOVER_SCC_DEGRADE_ACC = 11,             // SCC被动降级ACC或被动退出接管
  ADAS_TAKEOVER_NOA_DEGRADE_ACC = 12,             // NOA被动降级ACC或被动退出接管
  ADAS_TAKEOVER_LANE_CHAGE_FAILED = 13,           // 变道失败请求接管，如变道过程中空间不满足
} _ENUM_PACKED_ ADASTakeoverReason;


typedef enum {
  ADAS_CRUISE_ACCELERATE_STS_NONE = 0,
  ADAS_CRUISE_ACCELERATE_STS_ACCELERATING = 1, // 显示加速光波
  ADAS_CRUISE_ACCELERATE_STS_DECELERATING = 2, // 显示减速光波
} _ENUM_PACKED_ ADASCruiseAccelerateSts;

// 公共显示
typedef struct {
  CommonNotifyReq common_notify_req;                     // 通用的提醒
  float32 vehicle_speed_display;                         // 表显车速 KM/H
  boolean brake_pedal_pressed;                           // 刹车是否踩下
  ShiftLeverStateEnum shift_lever_state;                 // 挡位信息
  TurnLightState turn_light_state;                       // 转向灯信息
  TrajectoryPointSet tp_set;                             // 轨迹点
  IFLYPosition position;                                 // 自车位置
  RearViewMirrorCommand rear_view_mirror_signal_command; // 后视镜请求
  boolean adas_takeover_req;                             // 通用的接管请求
  ADASTakeoverReason adas_takeover_reason;               // 接管请求原因
  ADASCruiseAccelerateSts CruiseAccelerateSts;           // 加减速光波
  AvoidObstacleDirection avoiddirect;                    // 躲避方向，当前正在躲避的障碍物(纵向有重叠且正在躲避)
  LaneBorrowDirection borrow_direction;                  // 借道方向
} _STRUCT_ALIGNED_ HmiCommon;

// NOA状态,和FunctionalState中APA状态对应,修改时注意两边一致
typedef enum {
  NOA_OFF = 0,                    // 关闭
  NOA_PASSIVE = 1,                // 抑制
  NOA_STANDBY = 2,
  NOA_ACTIVE = 3,
  NOA_STAND_ACTIVE = 4,
  NOA_STAND_WAIT = 5,
  NOA_OVERRIDE_LATERAL = 6,       // 横向超控
  NOA_OVERRIDE_LONGITUDINAL = 7,  // 纵向超控
  NOA_OVERRIDE = 8,               // 纵向 + 横向
  NOA_SECURE = 9,
  NOA_DEGRADED = 10,
  NOA_FAILURE = 11,
} _ENUM_PACKED_ NoaStatus;

// 接管请求等级
typedef enum {
  TAKEOVER_NO_REQUEST,
  TAKEOVER_REQ_LEVEL_MILD, 
  TAKEOVER_REQ_LEVEL_MIDDLE,
  TAKEOVER_REQ_LEVEL_URGENT,
} _ENUM_PACKED_ TakeoverReqLevel;

typedef enum {
  NOA_NOTIFY_NO_REQ,
  NOA_NOTIFY_START_SUCCESS = 1,               // 功能开启成功
  NOA_NOTIFY_VEH_SPEED_HIGH = 2,              // 功能开启失败（车速过快）
  NOA_NOTIFY_TURN_LIGHT_ON = 3,               // 功能开启失败（转向灯抑制）
  NOA_NOTIFY_ROAD_NOT_SUPPORT = 4,            // 功能开启失败（车道抑制）
  NOA_NOTIFY_VEH_NOT_READY = 5,               // 功能开启失败-车辆未READY
  NOA_NOTIFY_CONDITION_NOT_SATISFIED = 6,     // 功能开启失败-其他ODD不满足导致激活失败
  NOA_NOTIFY_SYSTEM_FAILURE = 7,              // 功能开启失败-故障时激活
  NOA_NOTIFY_VEH_NEAR_TO_TOLL_STATION = 8,    // 功能开启失败-车辆处于收费站附近
  NOA_NOTIFY_LATERAL_EVASION = 9,             // 横向躲闪场景
  NOA_NOTIFY_EMERGENCY_SLOWDOWN = 10,         // 紧急减速场景
  NOA_NOTIFY_LANE_MERGE = 11,                 // 汇流
  NOA_NOTIFY_LANE_SPLIT_RIGHT = 12,           // 向右分流
  NOA_NOTIFY_LANE_SPLIT_LEFT = 13,            // 向左分流
  NOA_NOTIFY_LANE_NARROW = 14,                // 窄路
  NOA_NOTIFY_FRONT_VEH_LEAVE = 15,            // 跟车驶离提醒
  NOA_NOTIFY_RIGHT_LANE_CHANGE_PREPARE = 16,  // 向右变道准备
  NOA_NOTIFY_LEFT_LANE_CHANGE_PREPARE = 17,   // 向左变道准备
  NOA_NOTIFY_AVOID_FRONT_VEH = 18,            // 故障车避让变道
  NOA_NOTIFY_AVOID_OBSTACLE = 19,             // 障碍物避让变道
  NOA_NOTIFY_RIGHT_LANE_CHANGE_HOLD = 20,     // 右变道抑制 虚变实
  NOA_NOTIFY_LEFT_LANE_CHANGE_HOLD = 21,      // 左变道抑制 虚变实
  NOA_NOTIFY_LANE_CHANGE_BACK = 22,           // 变道返回
  NOA_NOTIFY_LANE_CHANGE_CANCEL = 23,         // 变道取消（人为取消）
  NOA_NOTIFY_LANE_CHANGE_WAIT_TIMEOUT = 24,   // 变道取消（等待超时取消）
  NOA_NOTIFY_LANE_CHANGE_SUCCESS = 25,        // 变道成功
  NOA_NOTIFY_ARRIVE_TOLL_STAITON = 26,        // 即将到达收费站
  NOA_NOTIFY_HANDS_OFF_LEVEL1 = 27,           // 脱手一级提醒
  NOA_NOTIFY_HANDS_OFF_LEVEL2 = 28,           // 脱手二级提醒
  NOA_NOTIFY_HANDS_OFF_LEVEL3 = 29,           // 脱手三级提醒
  NOA_NOTIFY_UNAVAILABLE = 30,                // 功能禁用
  NOA_NOTIFY_ACCELERATOR_OVERRIDE_LEVEL1 = 31,// 干预时长一级提醒
  NOA_NOTIFY_ACCELERATOR_OVERRIDE_LEVEL2 = 32,// 干预时长二级提醒
  NOA_NOTIFY_ACCELERATOR_OVERRIDE_LEVEL3 = 33,// 干预时长三级提醒
  NOA_NOTIFY_DOWNGRADE_TO_SCC = 34,           // NOA降级SCC (TODO:待确认删除)
  NOA_NOTIFY_UPGRADE_FROM_SCC = 35,           // SCC升级NOA
  NOA_NOTIFY_EMERGENCY_TAKEOVER = 36,         // 自动退出-较紧急场景
  NOA_NOTIFY_DEACTIVE = 37,                   // 主动接管退出
  NOA_NOTIFY_ARRIVE_DESTINATION = 38,         // 功能即将退出-即将到达目的地
  NOA_NOTIFY_DOWNGRADE_TO_ACC = 39,           // NOA降级ACC (TODO:待确认删除)
  NOA_NOTIFY_LANE_CHANGE_FAIL = 40,           // 变道失败
  NOA_NOTIFY_RIGHT_LANE_CHANGE_FOR_NAVIGATION = 41,    // 导航右变道
  NOA_NOTIFY_START_RECOMMEND = 42,                     // 推荐开启（上电后首次）
  NOA_NOTIFY_RIGHT_LANE_CHANGE_ONGING = 43,            // 自动右变道
  NOA_NOTIFY_LEFT_LANE_CHANGE_ONGING = 44,             // 自动左变道
  NOA_NOTIFY_OVERTAKING_RIGHT_LANE_CHANGE_ONGING = 45, // 超车右变道
  NOA_NOTIFY_OVERTAKING_LEFT_LANE_CHANGE_ONGING = 46,  // 超车左变道
  NOA_NOTIFY_ENTER_RAMP = 47,                          // 即将进入匝道
  NOA_NOTIFY_RIGHT_LANE_CHANGE_FAST_VEH_HOLD = 48,     // 右变道抑制-临车快速接近
  NOA_NOTIFY_LEFT_LANE_CHANGE_FAST_VEH_HOLD = 49,      // 左变道抑制-临车快速接近
  NOA_NOTIFY_NOT_SATISFY_ODD_EXIT = 50,     // 自动退出-不满足功能ODD
  NOA_NOTIFY_DIS_TO_TOLL_STAITON_200M = 51, // 200M后到达收费站
  NOA_NOTIFY_DIS_TO_TOLL_STAITON_150M = 52, // 150M后到达收费站
  NOA_NOTIFY_DIS_TO_TOLL_STAITON_100M = 53, // 100M后到达收费站
  NOA_NOTIFY_DIS_TO_TOLL_STAITON_75M = 54,  // 75M后到达收费站
  NOA_NOTIFY_DIS_TO_TOLL_STAITON_500M = 55, // 500M后到达收费站
  NOA_NOTIFY_DIS_TO_TOLL_STAITON_25M = 56,  // 25M后到达收费站
  NOA_NOTIFY_DIS_TO_DESTINATION_200M = 57,  // 200M后到达目的地
  NOA_NOTIFY_DIS_TO_DESTINATION_150M = 58,  // 150M后到达目的地
  NOA_NOTIFY_DIS_TO_DESTINATION_100M = 59,  // 100M后到达目的地
  NOA_NOTIFY_DIS_TO_DESTINATION_75M = 60,   // 75M后到达目的地
  NOA_NOTIFY_DIS_TO_DESTINATION_50M = 61,   // 50M后到达目的地
  NOA_NOTIFY_DIS_TO_DESTINATION_25M = 62,   // 25M后到达目的地
  NOA_NOTIFY_UPGRADE_FROM_ACC = 63,         // ACC升级NOA
  NOA_NOTIFY_AWAY_FROM_LARGE_VEH = 64,      // 避让大车
  NOA_NOTIFY_LANE_MERGE_FAIL = 65,          // 汇流失败
  NOA_NOTIFY_LEFT_LANE_CHANGE_FOR_NAVIGATION = 66, // 导航左变道
  NOA_NOTIFY_ENTER_RAMP_FAIL = 67,         // 即将下匝道失败，请接管
  NOA_NOTIFY_ENTER_RAMP_TIMEOUT = 68,      // 道路状况较差，下匝道超时，请接管
  NOA_NOTIFY_DIS_TO_RAMP_1000M = 69,       // 1km后进入匝道
  NOA_NOTIFY_DIS_TO_RAMP_200M = 70,        // 200m后进入匝道
  NOA_NOTIFY_LONG_SOLID_LINE_AHEAD = 71,   // 前方即将有长实线（长实线前500m）
  NOA_NOTIFY_TAKE_OVER_OF_TURN_RIGHT = 72, // 前方路口右转，请接管
  NOA_NOTIFY_TAKE_OVER_OF_TURN_LEFT = 73,  // 前方路口左转，请接管
  NOA_NOTIFY_ENABLE_EFFICIENT_MODE = 74,   // 开启高效通行模式
  NOA_NOTIFY_DISABLE_EFFICIENT_MODE = 75,  // 关闭高效通行模式
  NOA_NOTIFY_ENTER_RAMP_CONGESTION = 76,   // 匝道拥堵
  NOA_NOTIFY_AUTO_EXIT = 77,               // 自动退出（接管请求提醒、后溜退出）
  NOA_NOTIFY_ENABLE_SAFE_PASSAGE = 78,     // 安全通行（增大车距）
  NOA_NOTIFY_DISABLE_SAFE_PASSAGE = 79,    // 安全通行（减小车距）
  NOA_NOTIFY_TAKE_OVER_REQUEST = 80,       // 请立即接管
  NOA_NOTIFY_BORROW_WAY = 81,              // 借道避让
  NOA_NOTIFY_OVERSPEED_EXIT = 82,          // 超速退出
  NOA_NOTIFY_HAND_OFF_PUNISHMENT = 83,     // 脱手惩罚
  NOA_NOTIFY_FRONT_VEH_LEAVE_PLEASE_RESUME = 84, // 驶离提醒（跟停5-10min）
  NOA_NOTIFY_RECOMMENDED_LEFT_LANE_CHANGE = 85,  // 推荐左变道
  NOA_NOTIFY_RECOMMENDED_RIGHT_LANE_CHANGE = 86, // 推荐右变道
  NOA_NOTIFY_ADJUST_SPEED_BASE_ENVIRONMENT = 87, // 根据当前环境调整目标巡航车速
  NOA_NOTIFY_OVERTAKING_RIGHT_LANE_CHANGE_WAIT = 88, // 超车右变道
  NOA_NOTIFY_OVERTAKING_LEFT_LANE_CHANGE_WAIT = 89,  // 超车左变道
  NOA_NOTIFY_EXTREME_LANE_CHANGE = 90,               // 极限变道中
  NOA_NOTIFY_LATERAL_TAKEOVER = 91,                  // 干预方向盘
  NOA_NOTIFY_POSITIONING_QUALITY_POOR = 92,           // 定位质量不佳
  NOA_NOTIFY_NETWORK_SIGNAL_WEAK = 93,                // 网络信号不佳
  NOA_NOTIFY_NAVIGATION_APPROACHING_DESTINATION = 94, // 接近导航终点
  NOA_NOTIFY_TURN_SIGNAL_NOT_OFF = 95,       // 未关闭转向灯激活失败
  NOA_NOTIFY_ROAD_LANE_TOO_NARROW = 96,      // 车道过窄
  NOA_NOTIFY_WEATHER_ADVERSE = 97,           // 恶劣天气
  NOA_NOTIFY_TIRE_PRESSURE_ABNORMAL = 98,    // 胎压异常
  NOA_NOTIFY_LIDAR_OBSTRUCTED = 99,          // 激光雷达遮挡
  NOA_NOTIFY_CAMERA_FRONT_OBSTRUCTED = 100,   // 前视摄像头遮挡模糊
  NOA_NOTIFY_CAMERA_CALIBRATION_NEEDED = 101, // 相机未校准
  NOA_NOTIFY_STEERING_SYSTEM_FAILURE = 102,   // 转向系统故障
  NOA_NOTIFY_CHASSIS_SYSTEM_ABNORMAL = 103,   // 底盘系统异常
  NOA_NOTIFY_PARKING_BRAKE_NOT_RELEASED = 104, // 电子手刹未释放激活失败
  NOA_NOTIFY_VEHICLE_NOT_CENTERED = 105,      // 车辆未居中
  NOA_NOTIFY_VEHICLE_MODE_UNSUPPORTED = 106,  // 车辆模式不满足
  NOA_NOTIFY_HAZARD_LIGHTS_ON = 107,          // 未关闭危险警告灯激活失败
  NOA_NOTIFY_CAMERA_SURROUND_ABNORMAL = 108,  // 周视相机异常
  NOA_NOTIFY_RADAR_FRONT_ABNORMAL = 109,      // 前雷达异常
  NOA_NOTIFY_RADAR_CORNER_ABNORMAL = 110,     // 角雷达异常
  NOA_NOTIFY_HAND_OFF_DETECTION_ERROR = 111,  // 脱手检测异常
  NOA_NOTIFY_SWITCH_NOT_ACTIVATED = 112,      // 开关未打开激活失败
  NOA_NOTIFY_SECURE_STOP = 113,               // 安全停车
  NOA_NOTIFY_LAT_OVERRIDE = 114,              // 横向override
  NOA_NOTIFY_LAT_OVERRIDE_RECOVER = 115,      // 横向override恢复
  NOA_NOTIFY_Lane_Change_ADVICE_LEFT = 116,   // 推荐左变道
  NOA_NOTIFY_LANE_CHANGE_ADVICE_RIGHT = 117,  // 推荐右变道
  NOA_NOTIFY_LANE_CHANGE_ADVICE_CANCEL = 118, // 取消推荐变道
  NOA_NOTIFY_LANE_MERGE_COMPLETE = 119,       // 汇流完成
  NOA_NOTIFY_RESERVED_1 = 120,
  NOA_NOTIFY_ENTER_RAMP_COMPLETE = 121,       // 进匝道成功
  NOA_NOTIFY_CONE_DETECTED = 122,             // 检测到锥桶
  NOA_NOTIFY_SCC_UPGRADE_INHIBIT = 123,       // lcc升级抑制
  NOA_NOTIFY_EPB_ACTIVATE = 124,              // 未释放EPB，功能开启失败
  NOA_NOTIFY_ESC_DESABLE = 125,               // ESC关闭，功能开启失败
  NOA_NOTIFY_EXTREME_POWER_SAVER = 126,       // 极致节能模式下，巡航功能受限，功能开启失败
} _ENUM_PACKED_ NoaNotifyReq;

typedef enum {
  NOA_NOTIFY_NONE = 0,  
  NOA_RESTRAIN_MAIN_SWITCH_OFF = 1,             // 功能开启失败-软开关关闭
  NOA_RESTRAIN_VEH_SPEED_HIGH = 2,               // 功能开启失败-车速过快
  NOA_RESTRAIN_WIPER_MAX_GEAR = 3,              // 功能开启失败-雨刮最大档位
  NOA_RESTRAIN_TIRE_PRESSURE_ABNORMAL = 4,            //功能开启失败-胎压异常
  NOA_RESTRAIN_SEAT_BELT_NOT_FASTENED = 5,      // 功能开启失败-安全带未系
  NOA_RESTRAIN_BRAKE_WAS_PRESSED = 6,           // 功能开启失败-未松开刹车
  NOA_RESTRAIN_GEAR_NOT_IN_FORWAR = 7,          // 功能开启失败-档位非D档
  NOA_RESTRAIN_EPB_NOT_RELEASE = 8,          // 功能开启失败-EPB未释放
  NOA_RESTRAIN_ESC_DESABLE = 9,                // 功能开启失败-ESC关闭
  NOA_RESTRAIN_COLLISION_OCCURRED = 10, //功能开启失败-发生碰撞
  NOA_RESTRAIN_THROTTLE_EXCESSIVE = 11, //功能开启失败-油门过大
  NOA_RESTRAIN_DOOR_NOT_CLOSE = 12,              // 功能开启失败-车门未关
  NOA_RESTRAIN_TRUNK_NOT_CLOSE = 13,             // 功能开启失败-后备箱未关
  NOA_RESTRAIN_HOOD_NOT_CLOSE = 14,              // 功能开启失败-前车盖未关
  NOA_RESTRAIN_VEH_ROLL = 15,              // 功能开启失败-车辆后溜
  NOA_RESTRAIN_VEH_NOT_READY = 16,                // 功能开启失败-车辆未READY
  NOA_RESTRAIN_VEH_CHARGE = 17,               // 功能开启失败-车辆充电中
  NOA_RESTRAIN_LONG_SAFETY_FCT_ACTIVE = 18,      // 功能开启失败-纵向主动安全功能激活，FCW、AEB等
  NOA_RESTRAIN_OTHER_FCT_ACTIVE = 19,                // 功能开启失败-其他功能激活，ABS、TCS、泊车等
  NOA_RESTRAIN_SLOPE_LARGE = 20,       //功能开启失败-坡度过大
  NOA_RESTRAIN_LONG_ACCEL_LARGE = 21,    //纵向舒适性不满足
  NOA_RESTRAIN_EXTREME_POWER_SAVER = 22,           // 功能开启失败-模式受限
  NOA_RESTRAIN_CAMERA_BLOCK = 23,   // 功能开启失败-相机遮挡
  NOA_RESTRAIN_CONDITION_NOT_SATISFIED = 24,    // 功能开启失败-其他ODD不满足，光照雨量、其他场景等
  NOA_RESTRAIN_LONG_SYSTEM_FAILURE = 25,               // 功能开启失败-纵向相关系统故障
  NOA_RESTRAIN_LATERAL_SYSTEM_FAILURE = 26,               // 功能开启失败-横向相关系统故障
  NOA_RESTRAIN_LANE_CONDITION_NOT_SATISFIED = 27,           // 功能开启失败-车道条件不满足，宽度、车道线缺失等
  NOA_RESTRAIN_VEH_POSE_NOT_SATISFIED = 28,    //功能开启失败-车身距离、夹角等不满足
  NOA_RESTRAIN_LANE_CURVATURE_LARGE = 29,    //功能开启失败-弯道曲率过大
  NOA_RESTRAIN_LANE_STEER_CONDITION_NOT_SATISFIED = 30,    //功能开启失败-方向盘转角或转速过大
  NOA_RESTRAIN_LATER_ACCEL_LARGE = 31,     //功能开启失败-横向舒适性不满足
  NOA_RESTRAIN_PUNISH_ACTIVE = 32,     //功能开启失败-惩罚模式激活
  NOA_RESTRAIN_TURN_LIGHT_ON = 33,     //功能开启失败-转向灯激活
  NOA_RESTRAIN_EMERGENCY_LIGHT_ON = 34,     //功能开启失败-双闪激活
  NOA_RESTRAIN_LAT_SAFETY_FCT_ACTIVE = 35,      // 功能开启失败-横向主动安全功能激活，LKA、ELK等
  NOA_RESTRAIN_DRIVER_HANDS_OFF = 36,  // 功能开启失败-驾驶员脱手
  NOA_RESTRAIN_LOCALIZATION_ERROR = 37,  // 功能开启失败-定位错误
  NOA_RESTRAIN_AREA_CONDITION_NOT_SATISFIED  = 38,  // 功能开启失败-不在地理围栏内
  NOA_RESTRAIN_AREA_LEFT_DISTANCE_NOT_SATISFIED  = 39,  // 功能开启失败-地理围栏内剩余距离不足
  NOA_RESTRAIN_NAVAGITION_NOT_ACTIVE = 40,  // 功能开启失败-导航未开启

  NOA_TEXT_NOA_START_SUCCESS = 50,                // NOA开启成功,拨杆激活
  NOA_TEXT_SHARP_SLOWNDOWN = 51,                // 减速体感不舒适
  NOA_TEXT_FRONT_VEH_EXIST_WAIT_START = 52,       //前车未驶离-等待起步（0-10min）
  NOA_TEXT_FRONT_VEH_LEAVE_FOLLOW_START = 53,             // 驶离提醒-跟随起步（0-5min）
  NOA_TEXT_FRONT_VEH_LEAVE_PLEASE_RESUME = 54, // 驶离提醒-驾驶员起步（跟停5-10min）
  NOA_TEXT_ENABLE_EFFICIENT_MODE = 55,       // 开启高效通行模式，车距调整为1档
  NOA_TEXT_DISABLE_EFFICIENT_MODE = 56,      // 关闭高效通行模式，车距回复为X档
  NOA_TEXT_ENABLE_SAFE_PASSAGE = 57,         // 安全通行（增大车距）
  NOA_TEXT_DISABLE_SAFE_PASSAGE = 58,        // 安全通行（减小车距）
  NOA_TEXT_ADJUST_SPEED_BASE_ENVIRONMENT_INCREASE = 59, // 根据当前环境调整增加巡航车速
  NOA_TEXT_ADJUST_SPEED_BASE_ROAD_LIMIT_INCREASE  = 60,  // 根据道路限速增加目标巡航车速
  NOA_TEXT_ADJUST_SPEED_DUE_TO_OVERRIDE = 61,  // 巡航车速自动调整-油门override30s
  NOA_TEXT_ACCELERATOR_OVERRIDE_LEVEL1 = 62,    // Override一级告警
  NOA_TEXT_ACCELERATOR_OVERRIDE_LEVEL2 = 63,   // Override二级告警
  NOA_TEXT_SHARP_TURN = 64,                  // 通过急弯
  NOA_TEXT_ATTENTION_BAD_WEATHER = 65,                  // 注意恶劣天气
  NOA_TEXT_ACC_UPGRADE_NOA = 66,     //ACC升级NOA
  NOA_TEXT_LCC_MANUAL_UPGRADE_NOA = 67,     //LCC主动升级NOA
  NOA_TEXT_LCC_UPGRADE_NOA = 68,     //LCC被动升级NOA
  NOA_TEXT_LANE_NARROW = 69,                // 窄路场景
  NOA_TEXT_INTERSECTION_STRAIGHT  = 70,      // 路口直行
  NOA_TEXT_AWAY_FROM_LARGE_VEH = 71,        // 避让大车
  NOA_TEXT_DRIVER_TURN_LANE_CHANGE_NO_CONDITION = 72,  //拨杆变道条件不满足，如车速
  NOA_TEXT_LANE_CHANGE_PREPARE_LEFT = 73,  // 向左变道准备
  NOA_TEXT_LANE_CHANGE_PREPARE_RIGHT = 74,  // 向右变道准备
  NOA_TEXT_LANE_CHANGE_FAST_VEH_HOLD_LEFT = 75,   // 向左变道抑制-临车快速接近
  NOA_TEXT_LANE_CHANGE_FAST_VEH_HOLD_RIGHT = 76,   // 向右变道抑制-临车快速接近
  NOA_TEXT_LANE_CHANGE_SOLID_LINE_HOLD_LEFT = 77, // 向左变道抑制-虚变实
  NOA_TEXT_LANE_CHANGE_SOLID_LINE_RIGHT = 78, // 向右变道抑制-虚变实
  NOA_TEXT_LANE_CHANGE_NO_LANES_AVAILABLE_LEFT = 79,     // 向左变道抑制-无可用车道
  NOA_TEXT_LANE_CHANGE_NO_LANES_AVAILABLE_RIGHT = 80,     // 向右变道抑制-无可用车道
  NOA_TEXT_LANE_CHANGE_ONGING_LEFT = 81,         // 正在向左变道
  NOA_TEXT_LANE_CHANGE_ONGING_RIGHT = 82,         // 正在向右变道
  NOA_TEXT_LANE_CHANGE_BACK = 83,           // 变道返回-系统自动返回
  NOA_TEXT_LANE_CHANGE_CANCEL_MANUAL = 84,         // 变道取消（人为取消）
  NOA_TEXT_LANE_CHANGE_CANCEL_TIMEOUT = 85,         //变道取消（等待超时取消）
  NOA_TEXT_LANE_CHANGE_SUCCESS = 86,        // 变道成功
  NOA_TEXT_RED_LIGHT_STOP = 87,              // 红灯停车
  NOA_TEXT_GREEN_LIGHT_GO = 88,              // 绿灯起步
  NOA_TEXT_ATTENTION_LIGHTS = 89,            // 请注意观察红绿灯
  NOA_TEXT_TURN_IN_RIGHT_TURN_LANE = 90,     // 右转专用道右转
  NOA_TEXT_SHIFT_LEVER_TO_SPECIAL_LANE = 91, // 拨杆变道进入特殊车道
  NOA_TEXT_DETOUR_IN_CURRENT_LANE = 92,      // 本车道绕行
  NOA_TEXT_DETOUR_VIA_DASHED_LINE_LANE = 93, // 借虚线车道绕行
  NOA_TEXT_COMPACT_SOLID_LINE_TO_AVOID = 94, // 压实线避让障碍物
  NOA_TEXT_DETOUR_FAILURE = 95,            // 绕行失败
  NOA_TEXT_BRAKE_TO_STOP_AND_AVOID = 96,    // 无变道空间，刹停避让
  NOA_TEXT_UNABLE_TO_COMPLETE_DETOUR = 97,   // 刹停等待后无法完成绕行
  NOA_TEXT_START_RECOMMEND_NOA = 98, //进入NOA可用区域，推送给用户
  NOA_TEXT_LANE_CHANGE_REQUEST_CONFIRM_LEFT  = 99,  //请求驾驶员向左确认变道
  NOA_TEXT_LANE_CHANGE_REQUEST_CONFIRM_RIGHT  = 100,  //请求驾驶员向右确认变道
  NOA_TEXT_LANE_ATTENTION_CONE_AREA = 101,    //注意锥桶      
  NOA_TEXT_LANE_ENTER_RAMP_CONGESTION = 102,   // 匝道拥堵，无法汇入
  NOA_TEXT_LANE_GO_MAIN_CONGESTION = 103,   // 主路拥堵，无法汇入
  NOA_TEXT_EXTREME_LANE_CHANGE = 104,               // 极限变道中
  NOA_TEXT_EXTREME_NOA_SWITCH_LCC_BUTTON_DISABLE = 105,  //NOA与LCC切换开关暂时无法点击
  NOA_TEXT_APPROACH_DESTINATION_500M = 106,     //接近目的地500m
  NOA_TEXT_APPROACH_DESTINATION_250M = 107,     //接近目的地250m
  NOA_TEXT_APPROACH_DESTINATION = 108,  //接近目的地xxm，NOA即将退出   结合距离目的地的距离信号
  NOA_TEXT_APPROACH_TOLL_STATION_500M = 109,     //接近收费站500m
  NOA_TEXT_APPROACH_TOLL_STATION_250M = 110,     //接近收费站250m
  NOA_TEXT_APPROACH_TOLL_STATION = 111,  //接近收费站xxm，NOA即将退出   结合距离收费站的距离信号
  NOA_TEXT_ACC_OVERRIDE_ROCOVER_NOA = 112, //NOA横向超越恢复
  NOA_TEXT_ADJUST_SPEED_BASE_ENVIRONMENT_DECREASE = 113, //根据当前环境调整减小巡航车速
  NOA_TEXT_ADJUST_SPEED_BASE_ROAD_LIMIT_DECREASE = 114, //根据道路限速减小目标巡航车速
  NOA_TEXT_ABOUT_TO_QUIT_TAKEOVER_WARNING = 115, //NOA即将退出，请接管

  NOA_QUIT_MAIN_SWITCH_OFF = 120,             // 功能退出-软开关关闭
  NOA_QUIT_VEH_SPEED_HIGH = 121,               // 功能退出-车速过快
  NOA_QUIT_WIPER_MAX_GEAR = 122,              // 功能退出-雨刮最大档位
  NOA_QUIT_TIRE_PRESSURE_ABNORMAL = 123,            //功能退出-胎压异常
  NOA_QUIT_SEAT_BELT_NOT_FASTENED = 124,      // 功能退出-安全带未系
  NOA_QUIT_BRAKE_WAS_PRESSED = 125,           // 功能退出-踩下刹车
  NOA_QUIT_FCT_SWITCH_CANCEL = 126,          // 功能退出-拨杆主动退出
  NOA_QUIT_GEAR_NOT_IN_FORWAR = 127,          // 功能退出-档位非D档
  NOA_QUIT_EPB_NOT_RELEASE = 128,          // 功能退出-EPB未释放
  NOA_QUIT_ESC_DESABLE = 129,                // 功能退出-ESC关闭
  NOA_QUIT_COLLISION_OCCURRED = 130,  //功能退出-发生碰撞
  NOA_QUIT_STANDWAIT_TIME_OUT = 131, //功能退出-Standwait超时
  NOA_QUIT_OVERRIDE_TIME_OUT = 132, //功能退出-Oeverride超时
  NOA_QUIT_DOOR_NOT_CLOSE = 133,              // 功能退出-车门未关
  NOA_QUIT_TRUNK_NOT_CLOSE = 134,             // 功能退出-后备箱未关
  NOA_QUIT_HOOD_NOT_CLOSE = 135,              // 功能退出-前车盖未关
  NOA_QUIT_VEH_ROLL = 136,              // 功能退出-车辆后溜
  NOA_QUIT_VEH_NOT_READY = 137,                // 功能退出-车辆未READY
  NOA_QUIT_VEH_CHARGE = 138,               // 功能退出-车辆充电中
  NOA_QUIT_LONG_SAFETY_FCT_ACTIVE = 139,      // 功能退出-纵向主动安全功能激活，AEB等
  NOA_QUIT_OTHER_FCT_ACTIVE = 140,                // 功能退出-其他功能激活，ABS、TCS等
  NOA_QUIT_SLOPE_LARGE = 141,       //功能退出-坡度过大
  NOA_QUIT_EXTREME_POWER_SAVER = 142,           // 功能退出-模式受限
  NOA_QUIT_CAMERA_BLOCK = 143,   // 功能退出-相机遮挡
  NOA_QUIT_CONDITION_NOT_SATISFIED = 144,    // 功能退出-通用提醒
  NOA_QUIT_LONG_SYSTEM_FAILURE = 145,               // 功能退出-纵向相关系统故障
  NOA_QUIT_LATERAL_SYSTEM_FAILURE = 146,               // 功能退出-横向相关系统故障
  NOA_QUIT_LATERAL_INTERSECTION = 147,               // 功能退出-路口场景
  NOA_QUIT_LANE_ENTER_LEFT_TRUN_LANE = 148,            // 功能退出-识别到左转车道
  NOA_QUIT_LANE_ENTER_RIGHT_TRUN_LANE = 149,           // 功能退出-识别到右转车道
  NOA_QUIT_ROUNDABOUT = 150,                 // 功能退出-识别到环岛
  NOA_QUIT_CURVE_RATE_LARGE  = 151,          //功能退出-弯道过大
  NOA_QUIT_BAD_WEATHER = 152, //功能退出-恶劣天气
} _ENUM_PACKED_ NoaNotify;

// NOA功能显示
typedef struct {
  // noa 状态
  NoaStatus noa_status;
  // noa是否为激活状态
  boolean noa_activate_resp;  // (true: on / false: off)
  boolean noa_driver_denied;  // 激活失败
  // 接管请求等级
  TakeoverReqLevel noa_takeover_req_lv;
  uint32 distance_to_destination; // 距离目的地(m)
  uint32 distance_to_station;     // 收费站距离(m)
  uint32 distance_to_ramp;        // 距离匝道距离(m)
  uint32 distance_to_tunnel;      // 距离隧道距离(m)
  uint32 distance_to_split;       // 单位是米 离前方分流的距离
  uint32 distance_to_merge;       // 单位是米 离前方汇流的距离
  NoaNotifyReq noa_notify_req;    // 座舱交互（E0Y使用）
  NoaNotify noa_notify;           // 新的noa提醒(量产使用)
  LaneChangeStyle lane_change_style; // 变道风格设置反馈
  boolean noa_cruise_dclc_resp;      // 变道确认提醒开关反馈
} _STRUCT_ALIGNED_ HmiNoaInfo;

// 车道线信息
typedef struct {
  uint32 line_id; // 车道线
  PosType line_index;           // 车道线索引 
  LineType type;                // 车道线类型
  uint8 line_segments_size;                                                     // 车道线分段数量
  LineSegment line_segments[CAMERA_PERCEPTION_LINE_SEGMENTS_MAX_NUM];           // 车道线每段信息
  uint8 marking_segments_size;                                                  // 车道线线型分段数
  MarkingSegment marking_segments[CAMERA_PERCEPTION_MARKING_SEGMENTS_MAX_NUM];  // 线型信息
  uint8 color_segments_size;                                                    // 车道线颜色分段数
  ColorSegment color_segments[CAMERA_PERCEPTION_COLOR_SEGMENTS_MAX_NUM];        // 颜色信息
  int8 car_points_size;                                                        // 车道线点集大小 
  Point2f car_points[FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM];                // 自车坐标系车道线点集  (米)
} _STRUCT_ALIGNED_ HmiLineInfo;

// 障碍物信息
typedef struct {
  float32 target_long_position;    // 障碍物纵向距离       (米)
  float32 target_lat_position;     // 障碍物横向距离       (米)
  float32 heading_angle;           // 障碍物朝向（rad）
  int32 target_track_id;           // 障碍物ID
  ObjectType target_type;          // 障碍物类型
  ObjectMotionType motion_pattern; // 运动状态
  ObjectLightType light_status;    // 灯光状态
  Shape3d shape;                   // 尺寸，长、宽、高
  Point2f velocity;                // 速度
  int32 lane_id;                   // 所属车道ID, 0代表自车道，1代表右侧相临车道，-1代表左侧相邻车道,依次类推
} _STRUCT_ALIGNED_ HmiObjInfo;

// APA车位信息
typedef struct {
  uint32 apa_slot_id;               // 车位id
  ParkingSlotType apa_slot_type;    // 车位类型
  Point2f apa_slot_corner_points[FUSION_PARKING_SLOT_CORNER_POINT_NUM];  // 角点的坐标      (米)    <固定4个>
  boolean allow_parking;               // 是否允许泊入
  boolean planning_success;            // 是否规划成功 
  boolean is_narrow_slot;              // 是否窄车位标志
  uint32 apa_slot_num;                 // 车位序号
  uint8 limiters_size;                 // 此库位限位器个数
  ParkingFusionLimiter limiters[FUSION_PARKING_SLOT_LIMITER_MAX_NUM]; // 单个库位最多2个限位器,每个限位器两个点
} _STRUCT_ALIGNED_ HmiApaSlotInfo;

// 当前泊车状态,和FunctionalState中APA状态对应,修改时注意两边一致
typedef enum {
  APA_OFF = 0,
  APA_PASSIVE = 1,
  APA_STANDBY = 2,
  APA_IN_SEARCHING = 3,
  APA_GUIDANCE = 4,
  APA_SUSPEND = 5,
  APA_COMPLETED =6,
  APA_OUT_SEARCHING = 7,
  APA_ABORT = 8,
  APA_PRE_ACTIVE = 9,
  APA_FAILURE = 10,
} _ENUM_PACKED_ ApaStatus;

// 座舱交互提示
typedef enum {
  APA_NOTIFY_NO_REQ = 0,
  APA_NOTIFY_FAILURE = 1,                              // 泊车功能故障
  // 启动泊车前
  APA_NOTIFY_HMI_VERSION_UNMATCHED = 2,                // 版本不匹配
  APA_NOTIFY_VEH_NOT_READY = 3,                        // 车辆未READY
  APA_NOTIFY_VEH_SPEED_HIGH = 4,                       // 请降低车速
  APA_NOTIFY_UNAVAILABLE_ADAS_ON = 5,                  // 智驾功能中，APA暂不可用
  APA_NOTIFY_SYSTEM_UNAVAILABLE = 6,                   // 不满足APA激活条件
  // 智驾功能A流程：APA功能开启-泊入开始前
  // A流程：进入APA泊车，开始找车位
  APA_NOTIFY_SLOW_DOWN_SEARCHING = 7,                  // 请降低车速以搜索车位
  APA_NOTIFY_SLOPE_STEEP = 8,                          // 环境不支持泊车功能正常运行（坡度不满足）
  APA_NOTIFY_CLOSE_DOOR_SEARCHING = 9,                 // 车门打开时
  APA_NOTIFY_CLOSE_HATCH_SEARCHING = 10,               // 车前盖打开时
  APA_NOTIFY_CLOSE_TRUNK_SEARCHING = 11,               // 后备箱打开时
  APA_NOTIFY_FASTEN_SEATBELT_SEARCHING = 12,           // 未系安全带时
  APA_NOTIFY_SEARCHING = 13,                           // 正常识别车位过程:小飞正在帮你寻找车位，请继续往前行驶
  APA_NOTIFY_PARKING_SLOT_FOUND = 14,                  // 找到一个或多个车位，且车速＞0时
  APA_NOTIFY_SEARCHING_QUIT_HIGH_SPD = 15,             // A流程退出到主界面（速度超过30kph）
  APA_NOTIFY_SEARCHING_QUIT_ENVIRONMENT = 16,          // 环境不支持，自动泊车已退出(坡度超15%，光照雨量不满足条件)
  APA_NOTIFY_SEARCHING_QUIT_OVERTIME = 17,             // A流程退出到主界面（超时未点击”开始泊车“按钮）
  APA_NOTIFY_SEARCHING_QUIT_USER = 18,                 // A流程退出到主界面（软开关点击退出，回到上一级界面（即退出智驾APP））
  APA_NOTIFY_SYSTEM_FAILURE_IN_SEARCHING = 19,         // 发生异常，自动泊车已退出
  APA_NOTIFY_PLAN_FAILURE_IN_SEARCHING = 48,           // 路径规划不成功，返回A过程
  // A流程：选车位/泊车准备
  APA_NOTIFY_BREAK_THEN_START = 20,                    // 开始按钮置灰时（未踩刹车或P档或拉起EPB）:踩刹车后点击开始泊入（界面文案）
  APA_NOTIFY_UNAVAILABLE_M_GEAR = 21,                  // 开始按钮置灰时（挂m挡）:请更换挡位（界面文案）
  APA_NOTIFY_BREAK_AND_START = 22,                     // 开始按钮为激活时:踩住刹车点击开始（界面文案）
  APA_NOTIFY_SLIPING_WARNING = 23,                     // 溜车预警
  APA_NOTIFY_ABORT_HANDSHAKE = 24,                     // 松开刹车的同时点击开始按钮（握手失败）时
  // 智驾功能B流程：APA控车泊入开始-APA泊入成功/失败
  // 泊入B流程
  APA_NOTIFY_RELEASE_BREAK_START = 25,                 // 点击泊车开始按钮时未松刹车
  APA_NOTIFY_RESERVE1 = 26,                            // 预留
  APA_NOTIFY_PARKING_AND_WATCH_OUT = 27,               // 泊车进行时      
  APA_NOTIFY_COMPLETE = 28,                            // 泊车完成时，跳转完成界面后10s跳转至主界面
  APA_NOTIFY_PARKING_CONTINUE = 29,                    // 泊车继续时
  APA_NOTIFY_GEAR_SHIFT_UNRESPONSIVE = 30,             // 系统控车泊入时，驾驶员换挡，不执行
  APA_NOTIFY_PARKING_SUSPENSIVE_NO_BREAK = 31,         // 暂停时，用户未踩住刹车时，泊车继续按钮置灰
  APA_NOTIFY_ERRATIC_DYNAMIC_OBSTRUCTIVE = 32,         // 泊入过程中识别到路径上障碍物:等待障碍物移除(动态)
  APA_NOTIFY_BREAK_SUSPENSIVE = 33,                    // 系统控车泊入时，泊车暂停（驾驶员踩刹车）（预留）
  APA_NOTIFY_DOOR_OPEN_SUSPENSIVE = 34,                // 系统控车泊入时，泊车暂停（打开车门）（预留）
  APA_NOTIFY_HATCH_OPEN_SUSPENSIVE = 35,               // 系统控车泊入时，泊车暂停（打开车前盖）（预留）
  APA_NOTIFY_TRUNK_OPEN_SUSPENSIVE = 36,               // 系统控车泊入时，泊车暂停（打开车后盖）（预留）
  APA_NOTIFY_SEATBELT_SUSPENSIVE = 37,                 // 系统控车泊入时，泊车暂停（驾驶员松开安全带）（预留）
  APA_NOTIFY_ACC_SUSPENSIVE = 38,                      // 系统控车泊入时，泊车暂停（驾驶员踩油门）（预留）
  APA_NOTIFY_RELEASE_BREAK_CONTINUE = 39,              // 系统控车泊入时，泊车暂停（驾驶员踩刹车），继续泊车按钮置灰或者点击无效
  APA_NOTIFY_STATIC_OBSTRUCTIVE = 40,                  // 系统控车泊入时，泊车暂停（与不可自主移动障碍物距离＜30cm），继续按钮置灰或者点击无效
  APA_NOTIFY_SUSPENSIVE_RELEASE_ACC = 41,              // 系统控车泊入时，泊车暂停（驾驶员踩加速踏板），继续按钮置灰或者点击无效
  APA_NOTIFY_SUSPENSIVE_CLOSE_DOOR = 42,               // 系统控车泊入时，泊车暂停（车门打开），继续按钮置灰或者点击无效
  APA_NOTIFY_SUSPENSIVE_CLOSE_HATCH = 43,              // 系统控车泊入时，泊车暂停（前盖打开），继续按钮置灰或者点击无效
  APA_NOTIFY_SUSPENSIVE_CLOSE_TRUNK = 44,              // 灰系统控车泊入时，泊车暂停（后备箱打开），继续按钮置灰或者点击无效
  APA_NOTIFY_SUSPENSIVE_FASTEN_SEATBELT = 45,          // 系统控车泊入时，泊车暂停（安全带解开），继续按钮置灰或者点击无效
  APA_NOTIFY_QUIT_TAKE_OVER = 46,                      // 泊车已退出，请接管车辆（弹窗）
  APA_NOTIFY_QUIT = 47,                                // 泊车已退出
  // 48在A流程
  // 泊出准备
  APA_NOTIFY_PARKING_OUT_BREAK_AND_START = 49,          // 开始按钮为激活时:踩住刹车点击开始（界面文案）
  //泊出B流程
  APA_NOTIFY_PARKING_OUT_RELEASE_BREAK_AND_START = 50,  // 点击泊出开始按钮时未松刹车，松开刹车，开始自动泊出（界面文案）
  APA_NOTIFY_PARKING_BACK_OUT_AND_WATCH_OUT = 51,       // 车尾泊出中，请注意周围环境（界面文案）
  APA_NOTIFY_PARKING_OUT_COMPLETE = 52,                 // 泊出已完成
  APA_NOTIFY_PARKING_OUT_RELEASE_BREAK_CONTINUE = 53,   // 泊出暂停恢复时，驾驶员踩住了刹车
  APA_NOTIFY_PLAN_COMPUTING_IN_SEARCHING = 54,          // 开始按钮置灰:路径规划中
  APA_NOTIFY_ACTIVE_SUCCESS = 55,                       // apa激活成功
  APA_NOTIFY_FOLDING_REARVIEW_MIRRORS = 56,             // 正在收起后视镜
  APA_NOTIFY_SELECT_TARGET_PARKING_ON_IMAGE = 57,       // 请在影像界面选择目标车位
  APA_NOTIFY_SLOT_UNAVAILABLE_PLANNING_FAIL = 58,       // 目标车位不可停（规划失败）
  APA_NOTIFY_MAIN_SWITCH_OFF = 59,                      // 设置项开关未打开，功能无法开启
  APA_NOTIFY_CAMERA_MALFUNCTION_INHIBIT = 60,           // 摄像头故障，功能无法开启
  APA_NOTIFY_RADAR_MALFUNCTION_INHIBIT = 61,            // 雷达故障，功能无法开启
  APA_NOTIFY_SYSTEM_FAILURE_INHIBIT = 62,               // 系统故障，功能无法开启
  APA_NOTIFY_DEPENDENT_SYSTEM_FAILURE_INHIBIT = 63,     // 关联件系统故障，功能无法开启
  APA_NOTIFY_CAMERA_OBSTRUCTION_INHIBIT = 64,           // 摄像头遮挡，功能无法开启
  APA_NOTIFY_SPEED_OVER_LIMIT_INHIBIT = 65,             // 车速超限，功能无法开启
  APA_NOTIFY_TIRE_PRESSURE_ABNORMAL_INHIBIT = 66,        // 胎压异常，功能无法开启
  APA_NOTIFY_CAMERA_MALFUNCTION_EXIT = 67,              // 摄像头故障，泊车退出
  APA_NOTIFY_RADAR_MALFUNCTION_EXIT = 68,               // 雷达故障，泊车退出
  APA_NOTIFY_SYSTEM_FAILURE_EXIT = 69,                  // 系统故障，泊车退出
  APA_NOTIFY_DEPENDENT_SYSTEM_FAILURE_EXIT = 70,        // 关联件系统故障，泊车退出
  APA_NOTIFY_CAMERA_OBSTRUCTION_EXIT = 71,              // 摄像头遮挡，泊车退出
  APA_NOTIFY_SPEED_OVERSHOOT = 72,                      // 速度超调，泊车退出
  APA_NOTIFY_ESP_OFF = 73,                              // ESP关闭，功能终止
  APA_NOTIFY_EPB_ACTIVE = 74,                            // EPB拉起，功能终止
  APA_NOTIFY_TRAJECTORY_OVERRUN = 75,                    // 轨迹超限,泊车退出
  APA_NOTIFY_OTHER_FUNCTION_ACTIVATION = 76,             // 其他功能激活或关联功能激活,泊车退出
  APA_NOTIFY_NUMBER_OF_INTERRUPTIONS_OVERLIMIT = 77,    // 中断次数超限,泊车退出
  APA_NOTIFY_INTERRUPTION_TIME_OUT = 78,                // 中断时间超时,泊车退出
  APA_NOTIFY_PARKING_TIME_OUT = 79,                     // 泊车超时,泊车退出
  APA_NOTIFY_PARKING_SHIFT_OVERLIMIT = 80,               // 泊车超步,泊车退出
  APA_NOTIFY_DRIVER_INTERVENTION = 81,                   // 驾驶员干预,泊车退出
  APA_NOTIFY_INSUFFICIENT_PARKING_SPACE = 82,            // 泊车空间不足,泊车退出
  APA_NOTIFY_TIRE_PRESSURE_ABNORMAL_EXIT = 83,           // 胎压异常,泊车退出
  APA_NOTIFY_ENVIRONMENT_ABNORMAL_EXIT = 84,             // 环境异常,泊车退出
  APA_NOTIFY_STOP_TO_PARK_OUT = 85,                      // 如需泊出请停车
  APA_NOTIFY_CHARGING = 86,                              // 激活异常:充电中
  APA_NOTIFY_SEARCH_TIMED_OUT = 87,                      // 搜车位过程超时(预留)
  APA_NOTIFY_CAREFUL_SLOWS_DOWN = 88,                    // 车辆减速，请注意周围环境(预留)
  APA_NOTIFY_PARKING_IN_START = 89,                      // 泊入开始，请注意周围环境
  APA_NOTIFY_PARKING_OUT_START = 90,                     // 泊出开始，请注意周围环境
  APA_NOTIFY_OVERSPEED_EXIT = 91,                        // 车速超限，泊车退出
  APA_NOTIFY_PARKING_OUT_AVAILABLE = 92,                 // 泊出功能可用
  APA_NOTIFY_RELEASE_ACC_PEDAL = 93,                     // 搜车位时，车辆静止，踩油门异常暂停
  APA_NOTIFY_CHARGING_AND_PARKING_EXIT = 94,             // 充电中，泊车退出
  APA_NOTIFY_VEHICLE_COLLISION = 95,                     // 激活前提示：发生碰撞，泊车无法使用
  APA_NOTIFY_PARKING_SPACE_NARROW = 96,                  // 窄车位提醒，遥控泊车可用
  APA_NOTIFY_SIDE_MIRROR_FOLDED = 97,                    // 外后视镜未展开
  APA_NOTIFY_DRIVING_MODE_RESTRICTION = 98,              // 驾驶模式限制
  APA_NOTIFY_CUSTOM_PARKING_SPEED = 99,                  // 在自定义泊车界面且车速不为0
  APA_NOTIFY_SIDE_MIRROR_FOLD_SUSPEND = 100,             // 外后视镜折叠导致暂停
  APA_NOTIFY_VEHICLE_COLLISION_EXIT = 101,               // 发生碰撞导致退出
  APA_NOTIFY_DRIVING_MODE_EXIT = 102,                    // 驾驶模式导致退出
  APA_NOTIFY_ADAS_ACTIVE_EXIT = 103,                     // ADAS功能激活，泊车退出
  APA_NOTIFY_DRIVER_INTERVENTION_EXIT = 104,             // 系统控车泊入时，泊车退出（驾驶员踩加速踏板）
  APA_NOTIFY_CHARGING_AND_PARKING_SEARCHING = 105,       // 充电中，泊车暂不可用
  APA_NOTIFY_VEH_NOT_READY_REQUEST_SEARCHING = 106,      // 搜车位过程中动力未READY 
  APA_NOTIFY_ERRATIC_STATIC_OBSTRUCTIVE = 107,           // 泊入过程中识别到路径上障碍物:等待障碍物移除(静态)
  APA_NOTIFY_SLOT_UNAVAILABLE_OBSTACLE_OCCUPANCY = 108,  // 目标车位不可停（障碍物占用）
  APA_NOTIFY_NONE_PARKING_OUT_DIRECTION = 109,           // 泊出无可泊方向
  APA_NOTIFY_ULTRASONIC_OBSTRUCTION_EXIT = 110,           // 超声波雷达遮挡
  APA_NOTIFY_RECOMMEND_PARKING_IN        = 112,  // 推荐用户泊入
} _ENUM_PACKED_ ApaNotifyReq;

typedef enum {
  RPA_NO_REQUEST = 0,                         // 无请求
  RPA_IN_ON = 1,                              // RPA垂直入库激活
  RPA_OUT_ON = 2,                             // RPA垂直出库激活
  RPA_BAFA_ON = 3,                            // 直入直出
  HPA_RPA_IN_ON = 4,                          // HPA RPA入库激活
  HPA_RPA_OUT_ON = 5,                         // HPA RPA出库激活
  RPA_RESERVED = 6,                           // 预留
  RPA_WITHOUT_SUPERVISION = 7,                // 无人监控泊车
} _ENUM_PACKED_ RpaSubFuncSts;

typedef enum {
  RPA_QUIT_NO_REQ = 0,                                        // No Request
  RPA_QUIT_TIMING_OVERFLOW = 1,                               // 泊车/暂停超时，泊车系统退出
  RPA_QUIT_MOVE_TIMES_OVERFLOW = 2,                           // 泊车次数超限，泊车系统退出
  RPA_QUIT_RECOVER_INTERRUPT_TIMES_OVERFLOW = 3,              // 暂停次数超限，泊车系统退出
  RPA_QUIT_TRAJECTORY = 4,                                    // 泊车轨迹规划失败，泊车系统退出
  RPA_QUIT_SPACE_LIMIT = 5,                                   // 泊车空间不足，泊车系统退出
  RPA_QUIT_SPEED_HIGH = 6,                                    // 泊车速度太大，泊车系统退出
  RPA_QUIT_GAS_PEDAL = 7,                                     // 油门踏板踩下，泊车系统退出
  RPA_QUIT_ACC = 8,                                           // ACC开启，泊车系统退出
  RPA_QUIT_TJA = 9,                                           // TJA开启，泊车系统退出
  RPA_QUIT_SEAT_BELT_UNBUCKLE = 10,                           // 安全带解开，泊车系统退出
  RPA_QUIT_DRIVER_DOOR = 11,                                  // 车门打开，泊车系统退出
  RPA_QUIT_LUGGAGE_DOOR_OPEN = 12,                            // 后备箱打开，泊车系统退出
  RPA_QUIT_SURROUND_VIEW = 13,                                // 环视关闭，泊车系统退出
  RPA_QUIT_TERMINATE_BUTTON_PRESSED = 14,                     // 取消摁键摁下，泊车系统退出
  RPA_QUIT_GEAR_INTERVENTION = 15,                            // 驾驶员干预挡位，泊车系统退出
  RPA_QUIT_GEAR_INTERRUPT = 16,                               // 挡位不对，泊车系统退出
  RPA_QUIT_EPB_APPLY = 17,                                    // EPB拉起，泊车系统退出
  RPA_QUIT_VEHICLE_BLOCK = 18,                                // 车辆Block, 泊车系统退出
  RPA_QUIT_EXTERNAL_ECU_ACTIVE = 19,                          // ESP动态干预，泊车系统退出
  RPA_QUIT_STEERING_WHEEL_INTERVENTION = 20,                  // 驾驶员干预方向盘，泊车系统退出
  RPA_QUIT_PATH_DEVIATION = 21,                               // 路径偏离退出
  RPA_QUIT_OTHER_REASON = 22,                                 // 其他原因退出
  RPA_QUIT_PHONE_DISCONNECTED = 23,                           // 与手机断开连接，泊车系统退出
  RPA_QUIT_PHONE_LOW_BATTERY = 24,                            // 手机电池电量低，泊车系统退出
  RPA_QUIT_NO_POC_SLOT_AVAILABLE = 25,                        // 无可用泊出车位，泊车系统退出
  RPA_QUIT_UNSAFE_BEHAVIOR = 26,                              // P/EPB没被置起P-lock
  RPA_QUIT_DOOR_LOCK = 27,                                    // 车辆上锁，泊车系统退出(用于RPA)
  RPA_QUIT_REMOTE_DEVICE = 28,                                // 遥控设备原因导致系统退出
  RPA_QUIT_ENGINE_FRONT_DOOR_OPEN = 29,                       // 前引擎盖打开，泊车系统退出
  RPA_QUIT_FUNCTION_SAFETY = 30,                              // 因为功能安全原因导致泊车系统退出
  RPA_QUIT_INTERNAL_COMPONENTS_ABNORMAL = 31,                 // 内部组件异常退出
  RPA_QUIT_BACKWARD_MOVE_LONG = 32,                           // 长时间倒车退出
  RPA_QUIT_CORRESPONDING_REMIND_TIMEOUT = 33,                 // 相应提醒超时退出
  RPA_QUIT_TRAINING_PROCESS_OVERFLOW = 34,                    // 训练过程溢出退出
  RPA_QUIT_PATH_UPLOAD_TIMEOUT = 35,                          // 路径上传超时退出
  RPA_QUIT_STILL_MOVING_IN_RECOVERY_STAGES = 36,              // 恢复阶段仍在移动退出
  RPA_QUIT_LIMITED_SENSORS = 37,                              // 传感器受限退出
  RPA_QUIT_LIMITATION_PER_FUSION_MAP = 38,                    // PER融合地图限制退出
  RPA_QUIT_CHARGE_PLUG_CONNECTED = 39,                        // 充电插头连接退出
  RPA_QUIT_FOLDED_MIRROR = 40,                                // 折叠后视镜退出
  RPA_QUIT_PATH_LONG_OVER = 41,                               // 路径过长退出
  RPA_QUIT_SLOPE_OVER_LIMIT = 42,                             // 坡度超限退出
  RPA_QUIT_PITCH_OVER_LIMIT = 43,                             // 俯仰角超限退出
  RPA_QUIT_NOT_ALLOW_AREA = 44,                               // 不允许区域退出
  RPA_QUIT_LOC_UNAVAILABLE = 45,                              // 定位不可用退出
  RPA_QUIT_FREE_SPACE_LIMIT = 46,                             // 自由空间限制退出
  RPA_QUIT_PASSENGER_DOOR = 47,                               // 乘客门退出
  RPA_QUIT_NO_AVAILABLE_SLOT = 48,                            // 无可用车位退出
  RPA_QUIT_RESPONSE_TIMEOUT = 49,                             // 响应超时退出
  RPA_QUIT_RESERVED_32 = 50,                                  // 预留
  RPA_QUIT_RESERVED_33 = 51,                                  // 预留
  RPA_QUIT_RESERVED_34 = 52,                                  // 预留
  RPA_QUIT_RESERVED_35 = 53,                                  // 预留
  RPA_QUIT_RESERVED_36 = 54,                                  // 预留
  RPA_QUIT_RESERVED_37 = 55,                                  // 预留
  RPA_QUIT_RESERVED_38 = 56,                                  // 预留
  RPA_QUIT_RESERVED_39 = 57,                                  // 预留
  RPA_QUIT_RESERVED_3A = 58,                                  // 预留
  RPA_QUIT_RESERVED_3B = 59,                                  // 预留
  RPA_QUIT_RESERVED_3C = 60,                                  // 预留
  RPA_QUIT_RESERVED_3D = 61,                                  // 预留
  RPA_QUIT_RESERVED_3E = 62,                                  // 预留
  RPA_QUIT_RESERVED_3F = 63,                                  // 预留
  RPA_QUIT_RESERVED_40 = 64,                                  // 预留
  RPA_QUIT_RESERVED_41 = 65,                                  // 预留
  RPA_QUIT_RESERVED_42 = 66,                                  // 预留
  RPA_QUIT_RESERVED_43 = 67,                                  // 预留
  RPA_QUIT_ENGINE_START_FAULT = 68,                           // 发动机启动故障退出
  RPA_QUIT_BLUETOOTH_BRAKE_OT = 69,                           // 蓝牙刹车超时退出
  RPA_QUIT_UNRECOVERABLE_TIMEOUT_INTERRUPT = 70,              // 不可恢复超时中断退出
  RPA_QUIT_SEARCHING_APA_FAILURE = 71,                        // 搜索APA失败退出
  RPA_QUIT_SEARCHING_RELATED_SYSTEM_FAILURE = 72,             // 搜索相关系统失败退出
  RPA_QUIT_OUT_OF_ODD = 73,                                   // 超出ODD退出
  RPA_QUIT_AEB_ACTIVE = 74,                                   // AEB激活退出
  RPA_QUIT_SEARCHING_COLLISION_OCCURS = 75,                   // 搜索过程中发生碰撞退出
  RPA_QUIT_STANDBY_ADAS_ACTIVE = 76,                          // 有其他辅助驾驶功能正在使用,无法开启遥控泊车
  RPA_QUIT_DRIVINGMODE_SWITCH = 77,                           // 驾驶模式切换退出
} _ENUM_PACKED_ RpaQuitNotify;

typedef enum {
  RPA_OPERATE_NO_REQ = 0,                                     // No Request
  RPA_OPERATE_FUNCTION_SELECT = 1,                            // 功能选择请求
  RPA_OPERATE_GEAR_D = 2,                                     // D挡请求
  RPA_OPERATE_SLOW_DOWN = 3,                                  // 减速请求
  RPA_OPERATE_RECOVER_PHONE_OUT_OF_RANGE = 4,                 // 找回手机超出范围请求
  RPA_OPERATE_STOP = 5,                                       // 停止请求
  RPA_OPERATE_PS_ID_SELECTION = 6,                            // 泊车位ID选择请求
  RPA_OPERATE_TURN_LEVER = 7,                                 // 转向灯杆请求(预留)
  RPA_OPERATE_FUNCTION_SELECT_FOR_A_RPA = 8,                  // A/RPA功能选择请求
  RPA_OPERATE_CONNECT_PHONE = 9,                              // 连接手机请求
  RPA_OPERATE_EPB_APPLIED = 10,                               // EPB应用请求(请求P挡和EPB应用)
  RPA_OPERATE_LEAVE_CAR = 11,                                 // 离车请求
  RPA_OPERATE_CLOSE_TRUNK = 12,                               // 关闭后备箱请求
  RPA_OPERATE_CLOSE_ENGINE_FRONT_DOOR = 13,                   // 关闭引擎前门请求
  RPA_OPERATE_CLOSE_DOOR = 14,                                // 关闭所有车门请求
  RPA_OPERATE_BUCKLE_SEAT_BELT = 15,                          // 系安全带请求
  RPA_OPERATE_SURROUND_VIEW = 16,                             // 环视请求
  RPA_OPERATE_PRESS_BRAKE_PEDAL = 17,                         // 踩刹车踏板请求
  RPA_OPERATE_CONFIRM_PRESS_DM_SWITCH = 18,                   // 确认按下DM开关请求
  RPA_OPERATE_RELEASE_BRAKE = 19,                             // 释放刹车请求
  RPA_OPERATE_PROCESS_BAR = 20,                               // 进度条请求
  RPA_OPERATE_FUNCTION_OFF = 21,                              // 功能关闭请求
  RPA_OPERATE_POC_DIRECTION_SELECT = 22,                      // POC方向选择请求
  RPA_OPERATE_REPRESS_PARKING_SWITCH = 23,                    // 重新按下泊车开关请求
  RPA_OPERATE_RESERVED_18 = 24,                               // 预留
  RPA_OPERATE_RESERVED_19 = 25,                               // 预留
  RPA_OPERATE_RESERVED_1A = 26,                               // 预留
  RPA_OPERATE_EPS_ANGLE_OVER_LIMIT = 27,                      // EPS角度超限请求
  RPA_OPERATE_MRA_START_CONFIRM = 28,                         // MRA开始确认请求
  RPA_OPERATE_OPEN_OUTSIDE_REAR_VIEW_MIRROR = 29,             // 打开外后视镜请求
  RPA_OPERATE_FORWARD_MOVE_A_BIT = 30,                        // 向前移动一点请求
  RPA_OPERATE_CONFIRM_PRESS_UNDO_SWITCH = 31,                 // 确认按下撤销开关请求
  RPA_OPERATE_PS_INPUT = 32,                                  // 泊车位输入请求
  RPA_OPERATE_STEERING_WHEEL_HAND_OFF = 33,                   // 方向盘脱手请求
  RPA_OPERATE_UNDO_SELECT = 34,                               // 撤销选择请求
  RPA_OPERATE_REMOVE_CHARGING = 35,                           // 移除充电请求
  RPA_OPERATE_PRESS_BRAKE_AND_RECOVERY_BUTTON = 36,           // 踩刹车并按恢复按钮请求
  RPA_OPERATE_CONFIRM_SAVE_TRAINING_PATH = 37,                // 确认保存训练路径请求
  RPA_OPERATE_STOP_PARKING_SLOT_FIRST_RELEASED = 38,          // 停止泊车位首次释放请求
  RPA_OPERATE_SYSTEM_AUTO_SAVE_PATH_AFTER_TRAINING = 39,      // 训练成功后系统自动保存路径请求
  RPA_OPERATE_PATH_LENGTH_READY_EXCEED_LIMIT = 40,            // 路径长度即将超限请求
  RPA_OPERATE_TRAINING_PATH_IS_MAPPING = 41,                  // 训练路径正在映射请求
  RPA_OPERATE_TARGET_SLOT_OCCUPIED_TRY_OTHER = 42,            // 目标车位被占用，尝试其他车位请求
  RPA_OPERATE_CAUTION_REAR_SPACE = 43,                        // 注意后方空间请求
  RPA_OPERATE_PARKING_RESUME = 44,                            // 泊车恢复请求
  RPA_OPERATE_SPEED_OVER_LIMIT_APA_CANT_ACTIVE = 45,          // 速度超限，APA无法激活请求
  RPA_OPERATE_PATH_SAVING = 46,                               // 路径保存中请求
  RPA_OPERATE_RESERVED_2F = 47,                               // 预留(0x2F)
  RPA_OPERATE_RESERVED_30 = 48,                               // 预留
  RPA_OPERATE_RESERVED_31 = 49,                               // 预留
  RPA_OPERATE_RESERVED_32 = 50,                               // 预留
  RPA_OPERATE_RESERVED_33 = 51,                               // 预留
  RPA_OPERATE_RESERVED_34 = 52,                               // 预留
  RPA_OPERATE_RESERVED_35 = 53,                               // 预留
  RPA_OPERATE_DRIVINGMODE_SWITCH = 54,                        // 驾驶模式开关请求
  RPA_OPERATE_RESERVED_37 = 55,                          // 泊车完成请求
  RPA_OPERATE_RESERVED_38 = 56,                               // 预留
  RPA_OPERATE_RESERVED_39 = 57,                               // 预留
  RPA_OPERATE_RESERVED_3A = 58,                               // 预留
  RPA_OPERATE_RESERVED_3B = 59,                               // 预留
  RPA_OPERATE_RESERVED_3C = 60,                               // 预留
  RPA_OPERATE_RESERVED_3D = 61,                               // 预留
  RPA_OPERATE_PARKING_COMPLETE = 62,                          // 泊车完成请求
  RPA_OPERATE_PRESS_PARKING_SWITCH = 63,                      // 按下泊车开关请求
  RPA_OPERATE_CHECK_GEAR_P_OK = 64,                           // 检查P挡OK
  RPA_OPERATE_CHECK_ALL_DOORS_CLOSED_OK = 65,                 // 检查所有车门关闭OK
  RPA_OPERATE_CHECK_OUTSIDE_MIRROR_CLOSED_OK = 66,            // 检查外后视镜关闭OK
  RPA_OPERATE_CHECK_CHARGING_REMOVED_OK = 67,                 // 检查充电移除OK
  RPA_OPERATE_CHECK_RPA_SYSTEM_READY = 68,                    // 检查RPA系统就绪
  RPA_OPERATE_BAFA_AVAILABLE = 69,                            // BAFA可用
  RPA_OPERATE_PASSING_EXAM_SWT_SET = 70,                      // 通过考试开关设置请求
  RPA_OPERATE_PTREADY = 112,                                  // PT就绪请求
} _ENUM_PACKED_ RpaOperateNotify;

typedef enum {
  RPA_PAUSE_NO_REQ = 0,                                       // No Request
  RPA_PAUSE_RECOVER_PRESS_RESUME_SWITCH = 1,                  // 按下恢复开关恢复
  RPA_PAUSE_RECOVER_PASSENGER_DOOR_OPEN = 2,                  // 乘客门打开恢复
  RPA_PAUSE_RECOVER_FOUND_OBSTACLE_IN_TRAJECTORY = 3,         // 发现轨迹中有障碍物恢复
  RPA_PAUSE_RECOVER_PHONE_OUT_OF_RANGE = 4,                   // 手机超出范围恢复
  RPA_PAUSE_RECOVER_KEY_OUT_OF_RANGE = 5,                     // 钥匙超出范围恢复
  RPA_PAUSE_RECOVER_PEDESTRAIN_IN_TRAJECTORY = 6,             // 轨迹中有行人恢复
  RPA_PAUSE_RECOVER_VEHICLE_STANDSTILL_OVERTIME = 7,          // 车辆静止超时恢复
  RPA_PAUSE_RECOVER_EXTERNAL_MIRROR_OPEN = 8,                 // 外后视镜打开恢复
  RPA_PAUSE_RECOVER_TRUNK_OPEN = 9,                           // 后备箱打开恢复
  RPA_PAUSE_RECOVER_SEAT_BELT_UNBUCKLED = 10,                 // 安全带未系恢复
  RPA_PAUSE_RECOVER_CAMERA_BLOCKED = 11,                      // 摄像头被遮挡恢复
  RPA_PAUSE_RECOVER_HOOD_OPEN = 12,                           // 引擎盖打开恢复
  RPA_PAUSE_RECOVER_REMOTE_BUTTON_RELEASED = 13,              // 遥控按钮释放恢复(Jetour)
  RPA_PAUSE_RECOVER_FOUND_SIDE_OBSTACLE_IN_TRAJECTORY = 14,   // 发现轨迹侧方障碍物恢复(Jetour)
  RPA_PAUSE_RECOVER_REMOTE_CHARGING = 15,                     // 遥控充电恢复(Jetour)
} _ENUM_PACKED_ RpaPauseNotify;

typedef enum {
  RPA_SELF_CHECK_STS_NOT_AVAILABLE = 0,         // 自检不可用
  RPA_SELF_CHECK_AVAILABLE = 1,                 // 自检可用
  RPA_SELF_CHECK_FAILURE = 2,                   // 自检失败
} _ENUM_PACKED_ RpaSelfCheckSts;

typedef enum {
  RPA_SLOT_TYPE_NO_PARK = 0,                     // 无泊车位  
  RPA_SLOT_TYPE_PARALLEL = 1,                    // 平行泊车位
  RPA_SLOT_TYPE_CROSS = 2,                       // 垂直泊车位
  RPA_SLOT_TYPE_DIAGONAL = 3,                    // 斜线泊车位
} _ENUM_PACKED_ RpaSlotType;

typedef enum {
  BAFA_STATUS_OFF = 0,                          // 关闭
  BAFA_STATUS_STANDBY = 1,                      // 待机
  BAFA_STATUS_PREPARING = 2,                    // 准备
  BAFA_STATUS_GUIDANCE_ACTIVE = 3,              // 引导激活
  BAFA_STATUS_COMPLETED = 4,                    // 完成
  BAFA_STATUS_FAILURE = 5,                      // 失败
  BAFA_STATUS_TERMINATE = 6,                    // 终止
  BAFA_STATUS_PAUSE = 7,                        // 暂停
  BAFA_STATUS_RESERVED = 8,                     // 预留
} _ENUM_PACKED_ BafaStatus;

typedef enum {
  APA_POPUP_NONE = 0,                            // 无弹窗
  APA_POPUP_RPA_WITHOUT_SUPERVISON_NOTICE = 1,   // 无监督模式提示弹窗
  APA_POPUP_RPA_MODEL_SELECT_POPUP = 2,          // 模式选择弹窗
} _ENUM_PACKED_ ApaPopup;

typedef struct {
  boolean rpa_available;                                      // rpa开启开关是否可用
  boolean rpa_active_resp;                                    // rpa是否激活
  RpaSubFuncSts rpa_sub_func_sts;                             // rpa 功能运行模式
  boolean rpa_start_button_sts;                               // rpa开始按钮 0: 不显示 1: 显示
  boolean bafa_back_but_sts;                                  // 直入/直出按钮 0: 不显示 1: 显示
  boolean bafa_front_but_sts;                                 // 直入/直出按钮 0: 不显示 1: 显示
  boolean rpa_resume_button_sts;                              // 恢复按钮 0: 不显示 1: 显示
  boolean bafa_fun_sts;                                       // 直入/直出按钮 0: 不显示 1: 显示
  RpaQuitNotify rpa_quit_notify;                              // rpa退出提示
  RpaOperateNotify rpa_operate_notify;                        // rpa操作提示
  RpaPauseNotify rpa_pause_notify;                            // rpa暂停提示
  RpaSelfCheckSts rpa_self_check_sts;                         // rpa自检结果
  RpaSlotType rpa_slot_type;                                  // 选择的泊入/泊出的车位类型
  BafaStatus bafa_status;                                     // rpa 直入直出状态
  ApaPopup apa_popup;                                         // rpa 模式选择弹窗
}_STRUCT_ALIGNED_ HmiRpaInfo;

typedef enum {
  AVM_SCREEN_VIEW_NONE = 0,
  AVM_SCREEN_VIEW_STATE_1 = 1,  // SR
  AVM_SCREEN_VIEW_STATE_2 = 2,  // SR+1/3全景
  AVM_SCREEN_VIEW_STATE_3 = 3,  // SR+2/3全景
} _ENUM_PACKED_ AvmScreenView;

typedef enum {
  None = 0,  
  ANGLE_45_VIEW = 1, //45度视图  
  TOP_ANGLE_VIEW = 2,//俯视图 
  RESERVED_VIEW3 = 3,//预留视图3  
  RESERVED_VIEW4 = 4,//预留视图4  
  RESERVED_VIEW5 = 5,//预留视图5   
  RESERVED_VIEW6 = 6,//预留视图6  
  RESERVED_VIEW7 = 7,//预留视图7   
  RESERVED_VIEW8 = 8,//预留视图8 
  RESERVED_VIEW9 = 9,//预留视图9  
  ESERVED_VIEW10 = 10,//预留视图10
} _ENUM_PACKED_ ParkingViewCtrl;

typedef enum {
  PARKING_ARROW_NONE = 0,         //无箭头
  PARKING_ARROW_FRONT = 1,        //前方箭头
  PARKING_ARROW_BACK = 2,         //后方箭头
  PARKING_ARROW_FRONT_LEFT = 3,   //前左箭头
  PARKING_ARROW_FRONT_RIGHT = 4,  //前右箭头
  PARKING_ARROW_BACK_LEFT = 5,    //后左箭头
  PARKING_ARROW_BACK_RIGHT = 6,   //后右箭头
} _ENUM_PACKED_ ParkingArrow;

// APA功能信息
typedef struct {
  // 当前泊车模式
  ApaWorkMode apa_mode_status;
  // 当前泊车状态
  ApaStatus apa_status;                    // 泊车状态
  uint8 apa_slot_info_size;                // 车位数量
  HmiApaSlotInfo apa_slot_info[FUSION_PARKING_SLOT_MAX_NUM];      // apa界面显示车位线信息
  uint32 select_slot_id;                   // 用户选中的泊入车位ID号   <0为无效>
  ApaNotifyReq apa_notify_req;             // 座舱交互
  float32 apa_notify_remain_distance;      // 距离泊入成功还有x米
  float32 remain_distance_percentage;      // 距离泊入剩余距离百分比
  ApaFreeSlotInfo apa_free_slot_info;      // 自定义泊车
  PreparePlanningState prepare_plan_state; // 预规划状态

  /* 规划生成的泊出方向
   * 按位划分，每位上1表示可泊，0表示不可泊
   * bit 0: 校验位，1有效,0无效
   * 1-8:出库方向
   * bit 1: 垂直前左
   * bit 2: 水平前左
   * bit 3: 前
   * bit 4: 垂直前右
   * bi5 5: 水平前右
   * bit 6: 后
   * bit 7: 垂直后左
   * bit 8: 垂直后右
   * 11-14：入库方向
   * bit 11: 垂直车位车头入库
   * bit 12: 垂直车位车尾入库
   * bit 13: 平行车位，同向入库 (预留)
   * bit 14: 平行车位，掉头入库 (预留)
   */
  uint16 planning_park_dir;
  ApaParkOutDirection select_park_out_dir;   // 用户选择的泊出方向
  ApaParkingDirection select_park_in_dir;    // 用户选择的泊入方向
  uint32 parking_time;                       // 泊车用时，单位: s
  HmiRpaInfo hmi_rap_info;                   // rpa信息
  boolean apa_active_resp;                   // apa激活反馈
  boolean apa_active_denied;                 // 激活失败
  AvmScreenView avm_screen_view;             // AVM界面视图
  FuncButtonState apa_active_button_sts;     // 泊车辅助按钮状态
  FuncButtonState park_in_button_sts;        // 泊入按钮
  FuncButtonState park_out_button_sts;       // 泊出按钮
  FuncButtonState free_park_button_sts;      // 自定义泊车按钮
  FuncButtonState apa_start_button_sts;      // 开始泊入按钮
  FuncButtonState apa_resume_button_sts;     // 暂停后继续泊车按钮
  FuncButtonState rpa_active_button_sts;     // 遥控泊车按钮
  FuncButtonState leave_car_park_button_sts; // 离车泊入按钮
  ApaUserPreference apa_user_preference;     // 泊车用户偏好
  ParkingSpeedSet parking_speed_set;         // 泊车速度设置反馈
  ParkingViewCtrl parking_view_ctrl;         // 泊车视角控制信号
  ParkingArrow parking_arrow;                // 泊车箭头指示
} _STRUCT_ALIGNED_ HmiApaInfo;

// ACC状态,,和FunctionalState中ACC状态对应,修改时注意两边一致
typedef enum {
  ACC_OFF = 0,
  ACC_PASSIVE = 1,
  ACC_STANDBY = 2,
  ACC_ACTIVE = 3,
  ACC_STAND_ACTIVE = 4,
  ACC_STAND_WAIT = 5,
  ACC_OVERRIDE = 6,
  ACC_SECURE = 7,
  ACC_FAILURE = 8,
  ACC_BRAKE_ONLY = 9,
  ACC_COMFORT_EXIT = 10,
} _ENUM_PACKED_ AccStatus;

typedef enum {
  ACC_NOTIFY_NO_REQ = 0,
  ACC_NOTIFY_START_SUCCESS = 1,                // 功能开启成功
  ACC_NOTIFY_VEH_SPEED_HIGH = 2,               // 功能开启失败（车速过快）
  ACC_NOTIFY_TURN_LIGHT_ON = 3,                // 功能开启失败（转向灯抑制）
  ACC_NOTIFY_ROAD_NOT_SUPPORT = 4,             // 功能开启失败（车道抑制）
  ACC_NOTIFY_VEH_NOT_READY = 5,                // 功能开启失败-车辆未ready
  ACC_NOTIFY_CONDITION_NOT_SATISFIED = 6,      // 功能开启失败-其他ODD不满足导致激活失败
  ACC_NOTIFY_SYSTEM_FAILURE = 7,               // 功能开启失败-系统故障
  ACC_NOTIFY_EMERGENCY_SLOWDOWN = 8,           // 紧急减速场景
  ACC_NOTIFY_FRONT_VEH_LEAVE = 9,              // 前车起步
  ACC_NOTIFY_ENABLE_EFFICIENT_MODE = 10,       // 开启高效通行模式，车距调整为1档
  ACC_NOTIFY_DISABLE_EFFICIENT_MODE = 11,      // 关闭高效通行模式，车距回复为X档
  ACC_NOTIFY_HANDS_OFF_LEVEL1 = 12,            // 脱手一级提醒（已删除）
  ACC_NOTIFY_HANDS_OFF_LEVEL2 = 13,            // 脱手二级提醒（已删除）
  ACC_NOTIFY_HANDS_OFF_LEVEL3 = 14,            // 脱手三级提醒（已删除）
  ACC_NOTIFY_UNAVAILABLE = 15,                 // 功能禁用（已删除）
  ACC_NOTIFY_ACCELERATOR_OVERRIDE_LEVEL1 = 16, // 干预时长一级告警
  ACC_NOTIFY_ACCELERATOR_OVERRIDE_LEVEL2 = 17, // 干预时长二级告警
  ACC_NOTIFY_ACCELERATOR_OVERRIDE_LEVEL3 = 18, // 干预时长三级告警
  ACC_NOTIFY_EMERGENCY_TAKEOVER = 19,          // 自动退出-紧急场景
  ACC_NOTIFY_NOT_SATISFY_ODD_EIXT = 20,        // 自动退出-不满足ODD
  ACC_NOTIFY_DEACTIVE = 21,                    // ACC主动退出
  ACC_NOTIFY_DOWNGRADE_FROM_SCC = 22,          // SCC降级ACC
  ACC_NOTIFY_DOWNGRADE_FROM_NOA = 23,          // NOA降级ACC
  ACC_NOTIFY_AUTO_EXIT = 24,                   // 自动退出（接管请求超时、后溜退出）
  ACC_NOTIFY_ENABLE_SAFE_PASSAGE = 25,         // 安全通行（增大车距）
  ACC_NOTIFY_DISABLE_SAFE_PASSAGE = 26,        // 安全通行（减小车距）
  ACC_NOTIFY_MAIN_SWITCH_OFF = 27,             // 功能开启失败（软开关关闭）
  ACC_NOTIFY_SEAT_BELT_NOT_FASTENED = 28,      // 功能开启失败（安全带未系）
  ACC_NOTIFY_DOOR_NOT_CLOSE = 29,              // 功能开启失败（车门未关）
  ACC_NOTIFY_TRUNK_NOT_CLOSE = 30,             // 功能开启失败（后备箱未关）
  ACC_NOTIFY_HOOD_NOT_CLOSE = 31,              // 功能开启失败（前车盖未关）
  ACC_NOTIFY_WIPER_MAX_GEAR = 32,              // 功能开启失败（雨刮最大档位）
  ACC_NOTIFY_GEAR_NOT_IN_FORWAR = 33,          // 功能开启失败（档位非D档）
  ACC_NOTIFY_BRAKE_WAS_PRESSED = 34,           // 功能开启失败（未松开刹车）
  ACC_NOTIFY_SLOWDOWN_AND_TAKEOVER_REQ = 35,   // 急减速且请求接管
  ACC_NOTIFY_FRONT_VEH_LEAVE_PLEASE_RESUME = 36, // 驶离提醒（跟停5-10min）
  ACC_NOTIFY_ADJUST_SPEED_BASE_ENVIRONMENT = 37, // 根据当前环境调整目标巡航车速
  ACC_NOTIFY_ADJUST_SPEED_BASE_ROAD_LIMIT = 38,  // 根据道路限速调整目标巡航车速
  ACC_NOTIFY_ADJUST_SPEED_DUE_TO_OVERRIDE = 39,  // 巡航车速自动调整-油门override30s
  ACC_NOTIFY_ESP_OFF = 40,                       // ESP关闭，功能终止
  ACC_NOTIFY_EPB_ACTIVE = 41,                    // EPB拉起，功能终止
  ACC_NOTIFY_OVERSPEED_EXIT = 42,                // 超速退出
  ACC_NOTIFY_CAMERA_COVER = 43,                  // 功能开启失败（相机遮挡）
  ACC_NOTIFY_FOLLOW_FRONT_VEH_STAGE1 = 44,       // 跟车保持（0-5min）
  ACC_NOTIFY_FOLLOW_FRONT_VEH_STAGE2 = 45,       // 跟车保持（5-10min）
  ACC_NOTIFY_SECURE_STOP = 46,                   // 安全停车（最小风险策略）
  ACC_NOTIFY_FAILURE_EXIT = 47,                  // 故障退出
  ACC_NOTIFY_EPB_ACTIVATE = 48,                  // 未释放EPB，功能开启失败
  ACC_NOTIFY_ESC_DESABLE = 49,                   // ESC关闭，功能开启失败
  ACC_NOTIFY_EXTREME_POWER_SAVER = 50,           // 极致节能模式下，巡航功能受限，功能开启失败
} _ENUM_PACKED_ AccNotifyReq;

typedef enum {
  ACC_NOTIFY_NONE = 0,  
  ACC_RESTRAIN_MAIN_SWITCH_OFF = 1,             // 功能开启失败-软开关关闭
  ACC_RESTRAIN_VEH_SPEED_HIGH = 2,               // 功能开启失败-车速过快
  ACC_RESTRAIN_WIPER_MAX_GEAR = 3,              // 功能开启失败-雨刮最大档位
  ACC_RESTRAIN_TIRE_PRESSURE_ABNORMAL = 4,            //功能开启失败-胎压异常
  ACC_RESTRAIN_SEAT_BELT_NOT_FASTENED = 5,      // 功能开启失败-安全带未系
  ACC_RESTRAIN_BRAKE_WAS_PRESSED = 6,           // 功能开启失败-未松开刹车
  ACC_RESTRAIN_GEAR_NOT_IN_FORWAR = 7,          // 功能开启失败-档位非D档
  ACC_RESTRAIN_EPB_NOT_RELEASE = 8,          // 功能开启失败-EPB未释放
  ACC_RESTRAIN_ESC_DESABLE = 9,                // 功能开启失败-ESC关闭
  ACC_RESTRAIN_COLLISION_OCCURRED = 10, //功能开启失败-发生碰撞
  ACC_RESTRAIN_THROTTLE_EXCESSIVE = 11, //功能开启失败-油门过大
  ACC_RESTRAIN_DOOR_NOT_CLOSE = 12,              // 功能开启失败-车门未关
  ACC_RESTRAIN_TRUNK_NOT_CLOSE = 13,             // 功能开启失败-后备箱未关
  ACC_RESTRAIN_HOOD_NOT_CLOSE = 14,              // 功能开启失败-前车盖未关
  ACC_RESTRAIN_VEH_ROLL = 15,              // 功能开启失败-车辆后溜
  ACC_RESTRAIN_VEH_NOT_READY = 16,                // 功能开启失败-车辆未READY
  ACC_RESTRAIN_VEH_CHARGE = 17,               // 功能开启失败-车辆充电中
  ACC_RESTRAIN_LONG_SAFETY_FCT_ACTIVE = 18,      // 功能开启失败-纵向主动安全功能激活，FCW、AEB等
  ACC_RESTRAIN_OTHER_FCT_ACTIVE = 19,                // 功能开启失败-其他功能激活，ABS、TCS、泊车等
  ACC_RESTRAIN_SLOPE_LARGE = 20,       //功能开启失败-坡度过大
  ACC_RESTRAIN_LONG_ACCEL_LARGE = 21,    //纵向舒适性不满足
  ACC_RESTRAIN_EXTREME_POWER_SAVER = 22,           // 功能开启失败-模式受限
  ACC_RESTRAIN_CAMERA_BLOCK = 23,   // 功能开启失败-相机遮挡
  ACC_RESTRAIN_CONDITION_NOT_SATISFIED = 24,    // 功能开启失败-其他ODD不满足，光照雨量、其他场景等
  ACC_RESTRAIN_LONG_SYSTEM_FAILURE = 25,               // 功能开启失败-纵向相关系统故障

  ACC_TEXT_ACC_START_SUCCESS = 40,                // ACC开启成功,拨杆激活
  ACC_TEXT_SHARP_SLOWNDOWN = 41,                // 减速体感不舒适
  ACC_TEXT_FRONT_VEH_EXIST_WAIT_START = 42,       //前车未驶离-等待起步（0-10min）
  ACC_TEXT_FRONT_VEH_LEAVE_FOLLOW_START = 43,             // 前车驶离提醒-跟随起步（0-5min）
  ACC_TEXT_FRONT_VEH_LEAVE_PLEASE_RESUME = 44,  // 前车驶离提醒-驾驶员起步（跟停5-10min）
  ACC_TEXT_ENABLE_EFFICIENT_MODE = 45,       // 开启高效通行模式，车距调整为1档
  ACC_TEXT_DISABLE_EFFICIENT_MODE =  46,      // 关闭高效通行模式，车距回复为X档
  ACC_TEXT_ENABLE_SAFE_PASSAGE = 47,         // 安全通行（增大车距）
  ACC_TEXT_DISABLE_SAFE_PASSAGE = 48,        // 安全通行（减小车距）
  ACC_TEXT_ADJUST_SPEED_BASE_ENVIRONMENT_INCREASE = 49, // 根据当前环境调整增加巡航车速
  ACC_TEXT_ADJUST_SPEED_BASE_ROAD_LIMIT_INCREASE  = 50,  // 根据道路限速增加目标巡航车速
  ACC_TEXT_ADJUST_SPEED_DUE_TO_OVERRIDE = 51,  // 巡航车速自动调整-油门override30s
  ACC_TEXT_ACCELERATOR_OVERRIDE_LEVEL1 = 52,    // Override一级告警
  ACC_TEXT_ACCELERATOR_OVERRIDE_LEVEL2 = 53,   // Override二级告警
  ACC_TEXT_SHARP_TURN = 54,                  // 通过急弯
  ACC_TEXT_ATTENTION_BAD_WEATHER = 55,                  // 注意恶劣天气
  ACC_TEXT_SCC_DEGRADE_ACC = 56,     //SCC被动降级ACC
  ACC_TEXT_NOA_DEGRADE_ACC = 57,  //NOA被动降级ACC
  ACC_TEXT_MANUAL_NOA_DEGRADE_ACC = 58,  //NOA接管方向主动降级ACC，
  ACC_TEXT_MANUAL_SCC_DEGRADE_ACC = 59,  //SCC接管方向主动降级ACC
  ACC_TEXT_ADJUST_SPEED_BASE_ENVIRONMENT_DECREASE = 60, //根据当前环境调整减小巡航车速
  ACC_TEXT_ADJUST_SPEED_BASE_ROAD_LIMIT_DECREASE = 61,//根据道路限速减小目标巡航车速

 
  ACC_QUIT_MAIN_SWITCH_OFF = 70,             // 功能退出-软开关关闭
  ACC_QUIT_VEH_SPEED_HIGH = 71,               // 功能退出-车速过快
  ACC_QUIT_WIPER_MAX_GEAR = 72,              // 功能退出-雨刮最大档位
  ACC_QUIT_TIRE_PRESSURE_ABNORMAL = 73,            //功能退出-胎压异常
  ACC_QUIT_SEAT_BELT_NOT_FASTENED = 74,      // 功能退出-安全带未系
  ACC_QUIT_BRAKE_WAS_PRESSED = 75,           // 功能退出-踩下刹车
  ACC_QUIT_FCT_SWITCH_CANCEL = 76,          // 功能退出-拨杆主动退出
  ACC_QUIT_GEAR_NOT_IN_FORWAR = 77,          // 功能退出-档位非D档
  ACC_QUIT_EPB_NOT_RELEASE = 78,          // 功能退出-EPB未释放
  ACC_QUIT_ESC_DESABLE = 79,                // 功能退出-ESC关闭
  ACC_QUIT_COLLISION_OCCURRED = 80, //功能退出-发生碰撞
  ACC_QUIT_STANDWAIT_TIME_OUT = 81, //功能退出-Standwait超时
  ACC_QUIT_OVERRIDE_TIME_OUT = 82, //功能退出-Oeverride超时
  ACC_QUIT_DOOR_NOT_CLOSE = 83,              // 功能退出-车门未关
  ACC_QUIT_TRUNK_NOT_CLOSE = 84,             // 功能退出-后备箱未关
  ACC_QUIT_HOOD_NOT_CLOSE = 85,              // 功能退出-前车盖未关
  ACC_QUIT_VEH_ROLL = 86,              // 功能退出-车辆后溜
  ACC_QUIT_VEH_NOT_READY = 87,                // 功能退出-车辆未READY
  ACC_QUIT_VEH_CHARGE = 88,               // 功能退出-车辆充电中
  ACC_QUIT_LONG_SAFETY_FCT_ACTIVE = 89,      // 功能退出-纵向主动安全功能激活，AEB等
  ACC_QUIT_OTHER_FCT_ACTIVE = 90,                // 功能退出-其他功能激活，ABS、TCS等
  ACC_QUIT_SLOPE_LARGE = 91,       //功能退出-坡度过大
  ACC_QUIT_EXTREME_POWER_SAVER = 92,           // 功能退出-模式受限
  ACC_QUIT_CAMERA_BLOCK = 93,   // 功能退出-相机遮挡
  ACC_QUIT_CONDITION_NOT_SATISFIED = 94,    // 功能退出-通用提醒
  ACC_QUIT_LONG_SYSTEM_FAILURE = 95,               // 功能退出-纵向相关系统故障
  ACC_QUIT_BAD_WEATHER = 96,             //功能退出-恶劣天气
} _ENUM_PACKED_ AccNotify;

// 速度偏移设置
typedef struct {
  SpeedOffsetMode offset_mode;
  int32 speed_offset_value;
  int32 speed_offset_percentage;
} _STRUCT_ALIGNED_ SpeedOffsetInfo;

// 分心告警等级
typedef enum {
  DISTRACTION_NO_WARNING,
  DISTRACTION_WARN_LEVEL1_WARNING,
  DISTRACTION_WARN_LEVEL2_WARNING,
  DISTRACTION_WARN_LEVEL3_WARNING,
} _ENUM_PACKED_ DistractionWarn;

typedef enum {
  ACC_SPD_MODE_IND_CURRENT_SPEED_MODE = 0, // 当前车速模式
  ACC_SPD_MODE_IND_SET_ISLI_MODE = 1,          // 道路限速模式
  ACC_SPD_MODE_IND_SET_RESERVED = 2,           // 预留
} _ENUM_PACKED_ ACCCruiseSpdModeInd;

// ACC功能信息
typedef struct {
  AccStatus acc_status;                   // ACC状态机状态
  boolean acc_active_resp;                // acc是否为激活状态 (true:active / false:not active)
  boolean acc_driver_denied;              // 激活失败
  TakeoverReqLevel acc_takeover_req_lv;   // 接管请求
  int32 acc_set_headway;                  // 跟车时距
  float32 acc_set_speed;                  // 巡航速度 (公里/小时)
  boolean intelligent_following;          // 是否处于智慧跟车状态
  AccNotifyReq acc_notify_req;            // 座舱交互提示(E0Y使用)
  AccNotify acc_notify;                   // 新的notify(量产使用)
  DistractionWarn distraction_warning;    // 脱眼告警等级
  SpeedOffsetInfo speed_offset_set;       // 速度偏移设置反馈
  PilotUserPreference pilot_user_preference;   // 行车用户偏好
  ACCCruiseSpdModeInd acc_spd_mode_ind;   // 巡航车速模式
} _STRUCT_ALIGNED_ HmiAccInfo;

// SCC状态,和FunctionalState中SCC状态对应,修改时注意两边一致
typedef enum {
  SCC_OFF = 0,
  SCC_PASSIVE = 1,
  SCC_STANDBY = 2,
  SCC_ACTIVE = 3,
  SCC_STAND_ACTIVE = 4,
  SCC_STAND_WAIT = 5,
  SCC_OVERRIDE_LATERAL = 6,      // 横向超控
  SCC_OVERRIDE_LONGITUDINAL = 7, // 纵向超控
  SCC_OVERRIDE = 8,              // 横向+纵向
  SCC_SECURE = 9,
  SCC_DEGRADED = 10,
  SCC_FAILURE = 11,
} _ENUM_PACKED_ SccStatus;

// 脱手告警等级
typedef enum {
  HANDS_OFF_NO_WARNING,
  HANDS_OFF_LEVEL1_WARNING,
  HANDS_OFF_LEVEL2_WARNING,
  HANDS_OFF_LEVEL3_WARNING,
} _ENUM_PACKED_ SccHandsOffWarn;

// 车道线检测状态
typedef enum {
  NEITHER_SIDE_LANE_MARK_DETECTED,  // 两侧都没检测到
  LEFT_SIDE_LANE_MARK_DETECTED,     // 左侧检测到
  RIGHT_SIDE_LANE_MARK_DETECTED,    // 右侧检测到
  BOTH_SIDE_LANE_MARKS_DETECTED,    // 两侧都被检测到
} _ENUM_PACKED_ SccLineDetectStatus;

// 变道状态
typedef enum {
  NONE_LANE_CHANGE,
  UNAVAILABLE_LANE_CHANGE, // 变道不可用
  CANCEL_LANE_CHANGE,      // 变道取消
  SUCCESS_LANE_CHANGE,     // 变道成功
  FAIL_LANE_CHANGE,        // 变道失败
  REQUEST_LANE_CHANGE,     // 请求状态，需用户确认
  WAITING_LANE_CHANGE,     // 等待状态
  ONGOING_LANE_CHANGE,     // 正在变道中
  SOLID_INHIBITED_LANE_CHANGE,    // 实线抑制
  OBSTACLE_INHIBITED_LANE_CHANGE, // 障碍物抑制
} _ENUM_PACKED_ LC_STATUS;

// 变道方向
typedef enum {
  NONE_LC_DIRECTION,
  LEFT_LANE_CHANGE,   // 左转
  RIGHT_LANE_CHANGE,  // 右转
} _ENUM_PACKED_ LC_DIRECTION;

// 变道原因
typedef enum {
  NONE_LC_REASON,
  DRIVER_COMMAND,                  // 驾驶员指示变道
  PRECEEDING_VEHICLE_SLOW,         // 前面的车辆慢行
  CONSTRUCTION_ZONE,               // 施工区
  OBSTACLE,                        // 障碍物
  DRIVE_OUT_OF_OVERTAKING_LANE,    // 驶出超车车道
  EVASION_CONE,                    // 躲避锥筒
  NAVIGATION,                      // 导航
  FAULT_REMINDER,                  // 故障提醒
  LANE_BLOCK,                      // 车道拥堵
  ENTER_MAINROAD,                  // 入主路
  ENTER_RAMP,                      // 入匝道
  AVOID_RIGHTMOST_RAMP_ENTRANCE,   // 避开最右边的坡道入口
  TRAFFIC_JAM_AHEAD,               // 交通拥堵
  HIGH_EFFICIENCY,                 // 高效通行
  LANE_SPLIT,                      // 分流
  LANE_MERGE,                      // 合流
} _ENUM_PACKED_ LC_REASON;

// 窄路提醒
typedef enum {
  NONE_NARROW_ROAD_TIPS,
  NARROW_ROAD_CAN_PASS,      // 窄路可通过
  NARROW_ROAD_CANNOT_PASS,   // 窄路不可通过
} _ENUM_PACKED_ NarrowRoadTips;

// 变道信息
typedef struct {
  LC_STATUS lc_status;       // 变道状态
  LC_DIRECTION lc_direction; // 变道方向
  LC_REASON lc_reason;       // 变道原因
  int32 obstacle_id;         // 当变道受抑制时，干扰的障碍物ID
  LandingPoint landing_point;// 变道结束位置点
} _STRUCT_ALIGNED_ HmiLaneChange;

// 智慧躲闪的内容
typedef enum {
  NONE_DODGE,               // 未闪躲
  AVOID_OVERSIZED_VEHICLE,  // 躲避大车
  AVOID_OBSTACLE,           // 躲避障碍物
} _ENUM_PACKED_ DodgeType;

// 智慧躲闪信息
typedef struct {
  DodgeType dodge_type;
  int32 object_id;     //躲避的目标物Id
} _STRUCT_ALIGNED_ HmiIntelligentEvasion;

typedef enum {
  SCC_NOTIFY_NO_REQ,
  SCC_NOTIFY_START_SUCCESS = 1,              // 功能开启成功
  SCC_NOTIFY_VEH_SPEED_HIGH = 2,             // 功能开启失败（车速过快）
  SCC_NOTIFY_TURN_LIGHT_ON = 3,              // 功能开启失败（转向灯抑制）
  SCC_NOTIFY_ROAD_NOT_SUPPORT = 4,           // 功能开启失败（车道抑制）
  SCC_NOTIFY_VEH_NOT_READY = 5,              // 功能开启失败-车辆未READY
  SCC_NOTIFY_CONDITION_NOT_SATISFIED = 6,    // 功能开启失败-其他ODD不满足导致激活失败
  SCC_NOTIFY_SYSTEM_FAILURE = 7,             // 功能开启失败-故障时激活
  SCC_NOTIFY_NOT_SATISFY_ODD_EXIT = 8,       // 自动退出-不满足功能ODD
  SCC_NOTIFY_LATERAL_EVASION = 9,            // 横向躲闪场景
  SCC_NOTIFY_EMERGENCY_SLOWDOWN = 10,         // 紧急减速场景
  SCC_NOTIFY_LANE_MERGE = 11,                 // 汇流
  SCC_NOTIFY_LANE_SPLIT_RIGHT = 12,           // 向右分流
  SCC_NOTIFY_LANE_SPLIT_LEFT = 13,            // 向左分流
  SCC_NOTIFY_LANE_NARROW = 14,                // 窄路场景
  SCC_NOTIFY_FRONT_VEH_LEAVE = 15,            // 跟车驶离提醒
  SCC_NOTIFY_RIGHT_LANE_CHANGE_PREPARE = 16,  // 右变道准备（已删除）
  SCC_NOTIFY_LEFT_LANE_CHANGE_PREPARE = 17,   // 左变道准备（已删除）
  SCC_NOTIFY_RIGHT_LANE_CHANGE_FAST_VEH_HOLD = 18,   // 右变道抑制-临车快速接近
  SCC_NOTIFY_LEFT_LANE_CHANGE_FAST_VEH_HOLD = 19,    // 左变道抑制-临车快速接近
  SCC_NOTIFY_RIGHT_LANE_CHANGE_SOLID_LINE_HOLD = 20, // 右变道抑制-虚变实
  SCC_NOTIFY_LEFT_LANE_CHANGE_SOLID_LINE_HOLD = 21,  // 左变道抑制-虚变实
  SCC_NOTIFY_LANE_CHANGE_BACK = 22,           // 变道返回
  SCC_NOTIFY_LANE_CHANGE_CANCEL = 23,         // 变道取消（人为取消）
  SCC_NOTIFY_LANE_CHANGE_WAIT_TIMEOUT = 24,   // 变道取消（等待超时取消）
  SCC_NOTIFY_LANE_CHANGE_SUCCESS = 25,        // 变道成功
  SCC_NOTIFY_LANE_CHANGE_FAIL = 26,           // 变道失败
  SCC_NOTIFY_HANDS_OFF_LEVEL1 = 27,           // 脱手一级报警
  SCC_NOTIFY_HANDS_OFF_LEVEL2 = 28,           // 脱手二级报警
  SCC_NOTIFY_HANDS_OFF_LEVEL3 = 29,           // 脱手三级报警
  SCC_NOTIFY_UNAVAILABLE = 30,                // 功能禁用
  SCC_NOTIFY_ACCELERATOR_OVERRIDE_LEVEL1 = 31,// 超控一级报警
  SCC_NOTIFY_ACCELERATOR_OVERRIDE_LEVEL2 = 32,// 超控二级报警
  SCC_NOTIFY_ACCELERATOR_OVERRIDE_LEVEL3 = 33,// 超控三级报警
  SCC_NOTIFY_DOWNGRADE_TO_ACC = 34,           // SCC降级至ACC
  SCC_NOTIFY_UPGRADE_FROM_ACC = 35,           // ACC升级至SCC
  SCC_NOTIFY_EMERGENCY_TAKEOVER = 36,         // 自动退出-较紧急场景
  SCC_NOTIFY_DEACTIVE = 37,                   // 主动接管退出
  SCC_NOTIFY_AWAY_FROM_LARGE_VEH = 38,        // 避让大车
  SCC_NOTIFY_AVOID_FRONT_VEH = 39,            // 故障车避让变道
  SCC_NOTIFY_AVOID_OBSTACLE = 40,             // 障碍物避让变道
  SCC_NOTIFY_LANE_MERGE_FAIL = 41,            // 汇流失败
  SCC_NOTIFY_ENABLE_EFFICIENT_MODE = 42,      // 开启高效通行模式,车距调整为1档
  SCC_NOTIFY_AUTO_RIGHT_LANE_CHANGE_ONGING = 43,    // 自动右变道
  SCC_NOTIFY_AUTO_LEFT_LANE_CHANGE_ONGING = 44,     // 自动左变道
  SCC_NOTIFY_OVERTAKING_RIGHT_LANE_CHANGE_ONGING = 45,  // 右超车变道
  SCC_NOTIFY_OVERTAKING_LEFT_LANE_CHANGE_ONGING = 46,   // 左超车变道
  SCC_NOTIFY_LANE_ENTER_LEFT_TRUN_LANE = 47,            // 识别到左转车道(注意左转)
  SCC_NOTIFY_INTERSECTION_EXIT = 48,                    // 路口前退出功能
  SCC_NOTIFY_DISABLE_EFFICIENT_MODE = 49,  // 关闭高效通行模式，车距回复为X档
  SCC_NOTIFY_LONG_SOLID_LINE_AHEAD = 50,   // 前方即将有长实线（长实线前500m）
  SCC_NOTIFY_TAKE_OVER_OF_TURN_RIGHT = 51, // 前方路口右转，请接管
  SCC_NOTIFY_TAKE_OVER_OF_TURN_LEFT = 52,  // 前方路口左转，请接管
  SCC_NOTIFY_DOWNGRADE_FROM_NOA = 53,      // NOA降级到SCC
  SCC_NOTIFY_AUTO_EXIT = 54,               // 自动退出（接管请求超时、后溜退出）
  SCC_NOTIFY_LANE_ENTER_RIGHT_TRUN_LANE = 55,            // 识别到右转车道(注意右转)
  SCC_NOTIFY_ENABLE_SAFE_PASSAGE = 56,     // 安全通行（增大车距）
  SCC_NOTIFY_DISABLE_SAFE_PASSAGE = 57,    // 安全通行（减小车距）
  SCC_NOTIFY_TAKE_OVER_REQUEST = 58,       // 请立即接管
  SCC_NOTIFY_BORROW_WAY = 59,              // 借道避让
  SCC_NOTIFY_OVERSPEED_EXIT = 60,          // 超速退出
  SCC_NOTIFY_HAND_OFF_PUNISHMENT = 61,     // 脱手惩罚
  SCC_NOTIFY_FRONT_VEH_LEAVE_PLEASE_RESUME = 62, // 驶离提醒（跟停5-10min）
  SCC_NOTIFY_RECOMMENDED_LEFT_LANE_CHANGE = 63,  // 推荐左变道
  SCC_NOTIFY_RECOMMENDED_RIGHT_LANE_CHANGE = 64, // 推荐右变道
  SCC_NOTIFY_BAD_WEATHER = 65,    // 功能开启失败-环境恶劣
  SCC_NOTIFY_ROAD_CURVATURE_EXCESSIVE = 66,    // 功能开开关未开
  SCC_NOTIFY_MAIN_SWITCH_OFF = 67,             // 功能开启失败（软开关关闭）
  SCC_NOTIFY_SEAT_BELT_NOT_FASTENED = 68,      // 功能开启失败（安全带未系）
  SCC_NOTIFY_DOOR_NOT_CLOSE = 69,              // 功能开启失败（车门未关）
  SCC_NOTIFY_TRUNK_NOT_CLOSE = 70,             // 功能开启失败（后备箱未关）
  SCC_NOTIFY_HOOD_NOT_CLOSE = 71,              // 功能开启失败（前车盖未关）
  SCC_NOTIFY_WIPER_MAX_GEAR = 72,             // 功能开启失败（雨刮最大档位）
  SCC_NOTIFY_GEAR_NOT_IN_FORWAR = 73,          // 功能开启失败（档位非D档）
  SCC_NOTIFY_BRAKE_WAS_PRESSED = 74,          // 功能开启失败（未松开刹车）
  SCC_NOTIFY_INTERSECTION_STRAIGHT  = 75,      // 路口直行
  SCC_NOTIFY_ROUNDABOUT = 76,                 // 识别到环岛
  SCC_NOTIFY_TAKE_OVER_OF_ROUNDABOUT = 77,     // 识别到环岛
  SCC_NOTIFY_TAKE_OVER_OF_LATERAL_EXIT = 78,   // 横向接管，功能没退，请求接管
  SCC_NOTIFY_SHARP_TURN = 79,                  // 通过急弯
  SCC_NOTIFY_NO_LANES_AVAILABLE = 80,         // 无可用车道
  SCC_NOTIFY_DOWNGRADE_TO_ACC_OF_LATERAL = 81, // 横向接管导致降级
  SCC_NOTIFY_DOWNGRADE_TO_ACC_OF_INTERSECTION = 82, // 过路口导致降级
  SCC_NOTIFY_RED_LIGHT_STOP = 83,              // 红灯停车
  SCC_NOTIFY_GREEN_LIGHT_GO = 84,              // 绿灯起步
  SCC_NOTIFY_ATTENTION_LIGHTS = 85,            // 请注意观察红绿灯
  SCC_NOTIFY_TURN_IN_RIGHT_TURN_LANE = 86,     // 右转专用道右转
  SCC_NOTIFY_SHIFT_LEVER_TO_SPECIAL_LANE = 87, // 拨杆变道进入特殊车道
  SCC_NOTIFY_BRAKE_TO_STOP_AND_AVOID = 88,     // 刹停避让
  SCC_NOTIFY_UNABLE_TO_COMPLETE_DETOUR = 89,   // 无法完成绕行
  SCC_NOTIFY_COMPACT_SOLID_LINE_TO_AVOID = 90, // 压实线避让障碍物
  SCC_NOTIFY_DETOUR_IN_CURRENT_LANE = 91,      // 本车道绕行
  SCC_NOTIFY_DETOUR_VIA_DASHED_LINE_LANE = 92, // 借虚线车道绕行
  SCC_NOTIFY_LANE_CHANGE_TO_AVOID = 93,        // 变道避让障碍物
  SCC_NOTIFY_USER_NOT_LOGIN = 94,              // 用户未登陆
  SCC_NOTIFY_VEH_NOT_CENTERED = 95,            // 车辆未居中
  SCC_NOTIFY_VEH_TURNING = 96,                 // 弯道中
  SCC_NOTIFY_TIRE_PRESSURE_LOW = 97,           // 胎压不足
  SCC_NOTIFY_CAMERA_COVER = 98,                // 相机遮挡
  SCC_NOTIFY_SECURE_STOP = 99,                 // 安全退出(最小风险策略)
  SCC_NOTIFY_DISTRACTION_LEVEL1 = 100,         // 一级分心报警
  SCC_NOTIFY_DISTRACTION_LEVEL2 = 101,         // 二级分心报警
  SCC_NOTIFY_DISTRACTION_LEVEL3 = 102,         // 三级分心报警
  SCC_NOTIFY_LAT_OVERRIDE = 103,               // 横向override
  SCC_NOTIFY_LAT_OVERRIDE_RECOVER = 104,       // 横向override恢复
  SCC_NOTIFY_DETOUR_FAILURE = 105,             // 绕行失败
  SCC_NOTIFY_EPB_ACTIVATE = 106,               // 未释放EPB，功能开启失败
  SCC_NOTIFY_ESC_DESABLE = 107,                // ESC关闭，功能开启失败
  SCC_NOTIFY_EXTREME_POWER_SAVER = 108,        // 极致节能模式下，巡航功能受限，功能开启失败
} _ENUM_PACKED_ SccNotifyReq;

typedef enum {
  SCC_NOTIFY_NONE = 0,  
  SCC_RESTRAIN_MAIN_SWITCH_OFF = 1,             // 功能开启失败-软开关关闭
  SCC_RESTRAIN_VEH_SPEED_HIGH = 2,               // 功能开启失败-车速过快
  SCC_RESTRAIN_WIPER_MAX_GEAR = 3,              // 功能开启失败-雨刮最大档位
  SCC_RESTRAIN_TIRE_PRESSURE_ABNORMAL = 4,            //功能开启失败-胎压异常
  SCC_RESTRAIN_SEAT_BELT_NOT_FASTENED = 5,      // 功能开启失败-安全带未系
  SCC_RESTRAIN_BRAKE_WAS_PRESSED = 6,           // 功能开启失败-未松开刹车
  SCC_RESTRAIN_GEAR_NOT_IN_FORWAR = 7,          // 功能开启失败-档位非D档
  SCC_RESTRAIN_EPB_NOT_RELEASE = 8,          // 功能开启失败-EPB未释放
  SCC_RESTRAIN_ESC_DESABLE = 9,                // 功能开启失败-ESC关闭
  SCC_RESTRAIN_COLLISION_OCCURRED = 10, //功能开启失败-发生碰撞
  SCC_RESTRAIN_THROTTLE_EXCESSIVE = 11, //功能开启失败-油门过大
  SCC_RESTRAIN_DOOR_NOT_CLOSE = 12,              // 功能开启失败-车门未关
  SCC_RESTRAIN_TRUNK_NOT_CLOSE = 13,             // 功能开启失败-后备箱未关
  SCC_RESTRAIN_HOOD_NOT_CLOSE = 14,              // 功能开启失败-前车盖未关
  SCC_RESTRAIN_VEH_ROLL = 15,              // 功能开启失败-车辆后溜
  SCC_RESTRAIN_VEH_NOT_READY = 16,                // 功能开启失败-车辆未READY
  SCC_RESTRAIN_VEH_CHARGE = 17,               // 功能开启失败-车辆充电中
  SCC_RESTRAIN_LONG_SAFETY_FCT_ACTIVE = 18,      // 功能开启失败-纵向主动安全功能激活，FCW、AEB等
  SCC_RESTRAIN_OTHER_FCT_ACTIVE = 19,                // 功能开启失败-其他功能激活，ABS、TCS、泊车等
  SCC_RESTRAIN_SLOPE_LARGE = 20,       //功能开启失败-坡度过大
  SCC_RESTRAIN_LONG_ACCEL_LARGE = 21,    //纵向舒适性不满足
  SCC_RESTRAIN_EXTREME_POWER_SAVER = 22,           // 功能开启失败-模式受限
  SCC_RESTRAIN_CAMERA_BLOCK = 23,   // 功能开启失败-相机遮挡
  SCC_RESTRAIN_CONDITION_NOT_SATISFIED = 24,    // 功能开启失败-其他ODD不满足，光照雨量、其他场景等
  SCC_RESTRAIN_LONG_SYSTEM_FAILURE = 25,               // 功能开启失败-纵向相关系统故障
  SCC_RESTRAIN_LATERAL_SYSTEM_FAILURE = 26,               // 功能开启失败-横向相关系统故障
  SCC_RESTRAIN_LANE_CONDITION_NOT_SATISFIED = 27,           // 功能开启失败-车道条件不满足，宽度、车道线缺失等
  SCC_RESTRAIN_VEH_POSE_NOT_SATISFIED = 28,    //功能开启失败-车身距离、夹角等不满足
  SCC_RESTRAIN_LANE_CURVATURE_LARGE = 29,    //功能开启失败-弯道曲率过大
  SCC_RESTRAIN_LANE_STEER_CONDITION_NOT_SATISFIED = 30,    //功能开启失败-方向盘转角或转速过大
  SCC_RESTRAIN_LATER_ACCEL_LARGE = 31,     //功能开启失败-横向舒适性不满足
  SCC_RESTRAIN_PUNISH_ACTIVE = 32,     //功能开启失败-惩罚模式激活
  SCC_RESTRAIN_TURN_LIGHT_ON = 33,     //功能开启失败-转向灯激活
  SCC_RESTRAIN_EMERGENCY_LIGHT_ON = 34,     //功能开启失败-双闪激活
  SCC_RESTRAIN_LAT_SAFETY_FCT_ACTIVE = 35,      // 功能开启失败-横向主动安全功能激活，LKA、ELK等
  SCC_RESTRAIN_DRIVER_HANDS_OFF = 36,   // 功能开启失败-驾驶员脱手

  SCC_TEXT_SCC_START_SUCCESS = 50,                // SCC开启成功,拨杆激活
  SCC_TEXT_SHARP_SLOWNDOWN = 51,                // 减速体感不舒适
  SCC_TEXT_FRONT_VEH_EXIST_WAIT_START = 52,       //前车未驶离-等待起步（0-10min）
  SCC_TEXT_FRONT_VEH_LEAVE_FOLLOW_START = 53,             // 驶离提醒-跟随起步（0-5min）
  SCC_TEXT_FRONT_VEH_LEAVE_PLEASE_RESUME = 54,      // 驶离提醒-驾驶员起步（跟停5-10min）
  SCC_TEXT_ENABLE_EFFICIENT_MODE = 55,       // 开启高效通行模式，车距调整为1档
  SCC_TEXT_DISABLE_EFFICIENT_MODE = 56,      // 关闭高效通行模式，车距回复为X档
  SCC_TEXT_ENABLE_SAFE_PASSAGE = 57,         // 安全通行（增大车距）
  SCC_TEXT_DISABLE_SAFE_PASSAGE = 58,        // 安全通行（减小车距）
  SCC_TEXT_ADJUST_SPEED_BASE_ENVIRONMENT_INCREASE = 59,  // 根据当前环境调整增加巡航车速
  SCC_TEXT_ADJUST_SPEED_BASE_ROAD_LIMIT_INCREASE = 60,  // 根据道路限速增加目标巡航车速
  SCC_TEXT_ADJUST_SPEED_DUE_TO_OVERRIDE = 61,  // 巡航车速自动调整-油门override30s
  SCC_TEXT_ACCELERATOR_OVERRIDE_LEVEL1 = 62,    // Override一级告警
  SCC_TEXT_ACCELERATOR_OVERRIDE_LEVEL2 = 63,   // Override二级告警
  SCC_TEXT_SHARP_TURN = 64,                  // 通过急弯
  SCC_TEXT_ATTENTION_BAD_WEATHER = 65,                  // 注意恶劣天气
  SCC_TEXT_ACC_UPGRADE_SCC = 66,     //ACC升级SCC
  SCC_TEXT_NOA_MANUAL_DEGRADE_SCC = 67,   //NOA手动降级SCC
  SCC_TEXT_NOA_DEGRADE_SCC = 68,   //NOA被动降级SCC
  SCC_TEXT_LANE_NARROW = 69,                // 窄路场景
  SCC_TEXT_INTERSECTION_STRAIGHT  = 70,      // 路口直行
  SCC_TEXT_AWAY_FROM_LARGE_VEH = 71,        // 避让大车
  SCC_TEXT_DRIVER_TURN_LANE_CHANGE_NO_CONDITION = 72,  //拨杆变道条件不满足，如车速
  SCC_TEXT_LANE_CHANGE_PREPARE_LEFT = 73,  // 向左变道准备
  SCC_TEXT_LANE_CHANGE_PREPARE_RIGHT = 74,  // 向右变道准备
  SCC_TEXT_LANE_CHANGE_FAST_VEH_HOLD_LEFT = 75,   // 向左变道抑制-临车快速接近
  SCC_TEXT_LANE_CHANGE_FAST_VEH_HOLD_RIGHT = 76,   // 向右变道抑制-临车快速接近
  SCC_TEXT_LANE_CHANGE_SOLID_LINE_HOLD_LEFT = 77, // 向左变道抑制-虚变实
  SCC_TEXT_LANE_CHANGE_SOLID_LINE_RIGHT = 78, // 向右变道抑制-虚变实
  SCC_TEXT_LANE_CHANGE_NO_LANES_AVAILABLE_LEFT = 79,     // 向左变道抑制-无可用车道
  SCC_TEXT_LANE_CHANGE_NO_LANES_AVAILABLE_RIGHT = 80,     // 向右变道抑制-无可用车道
  SCC_TEXT_LANE_CHANGE_ONGING_LEFT = 81,         // 正在向左变道
  SCC_TEXT_LANE_CHANGE_ONGING_RIGHT = 82,         // 正在向右变道
  SCC_TEXT_LANE_CHANGE_BACK = 83,           // 变道返回-系统自动返回
  SCC_TEXT_LANE_CHANGE_CANCEL_MANUAL = 84,         // 变道取消（人为取消）
  SCC_TEXT_LANE_CHANGE_CANCEL_TIMEOUT = 85,       //变道取消（等待超时取消）
  SCC_TEXT_LANE_CHANGE_SUCCESS = 86,        // 变道成功
  SCC_TEXT_RED_LIGHT_STOP = 87,              // 红灯停车
  SCC_TEXT_GREEN_LIGHT_GO = 88,              // 绿灯起步
  SCC_TEXT_ATTENTION_LIGHTS = 89,            // 请注意观察红绿灯
  SCC_TEXT_TURN_IN_RIGHT_TURN_LANE = 90,     // 右转专用道右转
  SCC_TEXT_SHIFT_LEVER_TO_SPECIAL_LANE = 91, // 拨杆变道进入特殊车道
  SCC_TEXT_DETOUR_IN_CURRENT_LANE = 92,      // 本车道绕行，未压线
  SCC_TEXT_DETOUR_VIA_DASHED_LINE_LANE = 93, // 借虚线车道绕行
  SCC_TEXT_COMPACT_SOLID_LINE_TO_AVOID = 94, // 压实线避让障碍物
  SCC_TEXT_DETOUR_FAILURE = 95,            // 绕行失败
  SCC_TEXT_BRAKE_TO_STOP_AND_AVOID = 96,    // 无变道空间，刹停避让
  SCC_TEXT_UNABLE_TO_COMPLETE_DETOUR = 97,   // 刹停等待后无法完成绕行
  SCC_TEXT_START_RECOMMEND_NOA = 98, // NOA推荐开启
  SCC_TEXT_LANE_CHANGE_REQUEST_CONFIRM_LEFT = 99, //请求驾驶员确认向左变道 
  SCC_TEXT_LANE_CHANGE_REQUEST_CONFIRM_RIGHT = 100, //请求驾驶员确认向左变道
  SCC_TEXT_LANE_ATTENTION_CONE_AREA = 101, //注意锥桶
  SCC_TEXT_LANE_ENTER_RAMP_CONGESTION = 102, // 匝道拥堵，无法汇入
  SCC_TEXT_LANE_GO_MAIN_CONGESTION = 103, // 主路拥堵，无法汇入
  SCC_TEXT_EXTREME_LANE_CHANGE = 104, // 极限变道中
  SCC_TEXT_EXTREME_NOA_SWITCH_LCC_BUTTON_DISABLE = 105,  //NOA与LCC切换开关暂时无法点击
  SCC_TEXT_ACC_OVERRIDE_ROCOVER_SCC = 106, //SCC横向超越恢复
  SCC_TEXT_ADJUST_SPEED_BASE_ENVIRONMENT_DECREASE = 107, //根据当前环境调整减小巡航车速
  SCC_TEXT_ADJUST_SPEED_BASE_ROAD_LIMIT_DECREASE = 108, //根据道路限速减小目标巡航车速

  SCC_QUIT_MAIN_SWITCH_OFF = 120,             // 功能退出-软开关关闭
  SCC_QUIT_VEH_SPEED_HIGH = 121,               // 功能退出-车速过快
  SCC_QUIT_WIPER_MAX_GEAR = 122,              // 功能退出-雨刮最大档位
  SCC_QUIT_TIRE_PRESSURE_ABNORMAL = 123,            //功能退出-胎压异常
  SCC_QUIT_SEAT_BELT_NOT_FASTENED = 124,      // 功能退出-安全带未系
  SCC_QUIT_BRAKE_WAS_PRESSED = 125,           // 功能退出-踩下刹车
  SCC_QUIT_FCT_SWITCH_CANCEL = 126,          // 功能退出-拨杆主动退出
  SCC_QUIT_GEAR_NOT_IN_FORWAR = 127,          // 功能退出-档位非D档
  SCC_QUIT_EPB_NOT_RELEASE = 128,          // 功能退出-EPB未释放
  SCC_QUIT_ESC_DESABLE = 129,                // 功能退出-ESC关闭
  SCC_QUIT_COLLISION_OCCURRED = 130,  //功能退出-发生碰撞
  SCC_QUIT_STANDWAIT_TIME_OUT = 131, //功能退出-Standwait超时
  SCC_QUIT_OVERRIDE_TIME_OUT = 132, //功能退出-Oeverride超时
  SCC_QUIT_DOOR_NOT_CLOSE = 133,              // 功能退出-车门未关
  SCC_QUIT_TRUNK_NOT_CLOSE = 134,             // 功能退出-后备箱未关
  SCC_QUIT_HOOD_NOT_CLOSE = 135,              // 功能退出-前车盖未关
  SCC_QUIT_VEH_ROLL = 136,              // 功能退出-车辆后溜
  SCC_QUIT_VEH_NOT_READY = 137,                // 功能退出-车辆未READY
  SCC_QUIT_VEH_CHARGE = 138,               // 功能退出-车辆充电中
  SCC_QUIT_LONG_SAFETY_FCT_ACTIVE = 139,      // 功能退出-纵向主动安全功能激活，AEB等
  SCC_QUIT_OTHER_FCT_ACTIVE = 140,                // 功能退出-其他功能激活，ABS、TCS等
  SCC_QUIT_SLOPE_LARGE = 141,       //功能退出-坡度过大
  SCC_QUIT_EXTREME_POWER_SAVER = 142,           // 功能退出-模式受限
  SCC_QUIT_CAMERA_BLOCK = 143,   // 功能退出-相机遮挡
  SCC_QUIT_CONDITION_NOT_SATISFIED = 144,    // 功能退出-通用提醒
  SCC_QUIT_LONG_SYSTEM_FAILURE = 145,               // 功能退出-纵向相关系统故障
  SCC_QUIT_LATERAL_SYSTEM_FAILURE = 146,               // 功能退出-横向相关系统故障
  SCC_QUIT_LATERAL_SYSTEM_INTERSECTION = 147,               // 功能退出-路口场景
  SCC_QUIT_LANE_ENTER_LEFT_TRUN_LANE = 148,            // 功能退出-识别到左转车道
  SCC_QUIT_LANE_ENTER_RIGHT_TRUN_LANE = 149,           // 功能退出-识别到右转车道
  SCC_QUIT_ROUNDABOUT = 150,                 // 功能退出-识别到环岛
  SCC_QUIT_CURVE_RATE_LARGE = 151,       //功能退出-弯道过大
  SCC_QUIT_BAD_WEATHER = 152,   //功能退出-恶劣天气
} _ENUM_PACKED_ SccNotify;

// SCC功能信息
typedef struct {
  // SCC状态
  SccStatus scc_status;
  boolean scc_active_resp;       // scc主功能反馈               (true:active / false:not active)
  boolean scc_driver_denied;     // 激活失败
  SccHandsOffWarn scc_hands_off_warning; // 脱手告警等级，提示用户：手脱离方向盘
  TakeoverReqLevel scc_takeover_req_lv;  // 提示用户接管
  SccLineDetectStatus scc_line_detect_status;// 车道线检测状态
  HmiIntelligentEvasion intelligent_evasion; // 智慧躲闪
  HmiLaneChange lane_change;                 // 变道信息
  NarrowRoadTips narrow_road_tips;           // 窄路提示
  SccNotifyReq scc_notify_req;               // 座舱交互提示(E0Y使用)
  SccNotify scc_notify;                      // 新的scc notify(量产使用)
  HandsOffDetection hands_off_detection;     // 脱手检测设置反馈
  boolean traffic_light_stop_go;             // 直行红绿灯启停设置反馈
  boolean obstacle_bypass;                   // 借道绕行设置反馈
} _STRUCT_ALIGNED_ HmiSccInfo;

// adas信息
typedef struct {
  LDWOutputInfoStr ldw_output_info;
  LDPOutputInfoStr ldp_output_info;
  ELKOutputInfoStr elk_output_info;
  TSROutputInfoStr tsr_output_info;
  IHCOutputInfoStr ihc_output_info;
  AMAPOutputInfoStr ama_output_info;
  MEBOutputInfoStr meb_output_info;
  uint8 supp_signs_size;                                                        // 辅助标志牌数量
  CameraPerceptionSuppSign supp_signs[CAMERA_PERCEPTION_SUPP_SIGNS_MAX_NUM];    // 辅助标志牌列表
  uint8 traffic_lights_size;                                                              // 交通信号灯数量
  CameraPerceptionTrafficLight traffic_lights[CAMERA_PERCEPTION_TRAFFIC_LIGHTS_MAX_NUM];  // 交通信号灯列表
  CameraPerceptionTrafficLight current_lane_traffic_light;   //与自车道通行方向相同的信号灯
} _STRUCT_ALIGNED_ HmiAdasInfo;

// HMI输出信息

typedef enum {
  HMI_CALIB_NO_MESSAGE = 0,    // 无消息
  HMI_AVM_RELOAD_SUCCESS = 1,  // AVM加载新标定成功
  HMI_AVM_ERROR = 2,           // AVM模块错误
  HMI_CALIB_SUCCESS = 3,       // 标定成功
  // 下面是标定模块返回的错误码
  HMI_CALIB_BAD_REF_POINTS_INFO = 4,
  HMI_CALIB_WRONG_SENSOR_NUM = 5,
  HMI_CALIB_BAD_REF_POINT_NUM = 6,
  HMI_CALIB_BAD_REF_EXTRINSIC = 7,
  HMI_CALIB_BAD_INTRINSIC = 8,
  HMI_CALIB_BAD_SENSOR_DATA = 9,
  HMI_CALIB_BAD_SENSOR_DATA_NUM = 10,
  HMI_CALIB_PATTERN_NOT_FOUND = 11,
  HMI_CALIB_CALIBRATION_FAILED = 12,
  HMI_CALIB_SINGLE_TURB_VALIDATION_FAILED = 13,
  HMI_CALIB_OVERLAP_TURB_VALIDATION_FAILED = 14,
  HMI_CALIB_REPROJECT_VALIDATION_FAILED = 15,
  HMI_CALIB_POINT_TO_PLANE_VALIDATION_FAILED = 16,
  HMI_CALIB_POINT_TO_PLANE_TURB_FAILED = 17,
  HMI_CALIB_IMU_VALIDATION_FAILED = 18,
  HMI_CALIB_ERROR_BAD_ROLL_RESULT = 19,
  HMI_CALIB_ERROR_BAD_PITCH_RESULT = 20,
  HMI_CALIB_ERROR_BAD_YAW_RESULT = 21,
  HMI_CALIB_ERROR_BAD_X_RESULT = 22,
  HMI_CALIB_ERROR_BAD_Y_RESULT = 23,
  HMI_CALIB_ERROR_BAD_Z_RESULT = 24,
  HMI_CALIB_ERROR_VEHICLE_NOT_STATIONARY = 25,
  HMI_CALIB_TIMEOUT = 26,
  HMI_CALIB_TERMINATE_ABNORMALLY = 27,
  HMI_CALIB_UNKNOWN_ERROR = 28,
  HMI_CALIB_INIT_ERROR = 29,
  HMI_CALIB_IGNORED = 30,
} _ENUM_PACKED_ HmiCalibInfo;

// 传感器状态
typedef enum {
  SENSOR_NORMAL, 
  SENSOR_BLOCKAGE, // 被遮挡
  SENSOR_FAILURE,  // 故障
} _ENUM_PACKED_ SensorState;

// 传感器信息
typedef struct {
  SensorType sensor_type;       // 传感器类型
  SensorState sensor_state;     // 传感器状态
} _STRUCT_ALIGNED_ HmiSensorInfo;

// 语音控车车速反馈
typedef enum {
  NONE_SPEED_SET_RSP = 0,
  SPEED_EXCEEDS_ADJUSTABLE_RANGE = 1,        // 超过可调节速度范围
  SPEED_EXCEEDS_ROAD_LIMIT_RANGE = 2,        // 超过道路限速范围
  SPEED_EXCEEDS_FUNCTIONAL_LIMIT_RANGE = 3,  // 超过功能支持速度范围
} _ENUM_PACKED_ SpeedSetFaileRsp;

// 语音控车车距反馈
typedef enum {
  NONE_INTERVAL_SET_RSP = 0,
  CURRENT_REACHED_MAX_INTERVAL = 1,       // 目前已是最大跟车距离
  CURRENT_REACHED_MIN_INTERVAL = 2,       // 目前已是最小跟车距离
  CONDITION_NOT_ALLOWED_INTERVAL_SET = 3, // 车况/路况不允许此指令生效（如后车过近等） 
} _ENUM_PACKED_ IntervalSetFailRsp;

// 语音泊车偏好设置反馈
typedef enum {
  NONE_PARKING_PREFERENCE_SET_RSP = 0,
  PARKING_PREFERENCE_UNSUPPORTED = 1, // 无法支持偏好设置
  PARKING_PREFERENCE_SET_RSP_RESVRVED_1,
  PARKING_PREFERENCE_SET_RSP_RESVRVED_2,
  PARKING_PREFERENCE_SET_RSP_RESVRVED_3,
  PARKING_PREFERENCE_SET_RSP_RESVRVED_4,
  PARKING_PREFERENCE_SET_RSP_RESVRVED_5,
  PARKING_PREFERENCE_SET_RSP_RESVRVED_6,
  PARKING_PREFERENCE_SET_RSP_RESVRVED_7,
  PARKING_PREFERENCE_SET_RSP_RESVRVED_8,
} _ENUM_PACKED_ ParkingPreferenceSetFailRsp;

// 功能软开关状态反馈
typedef struct {
  boolean acc_switch_response; // acc软开关反馈        (true:on / false:off)
  boolean lcc_switch_response; // scc软开关反馈        (true:on / false:off)
  boolean noa_switch_response; // noa软开关反馈        (true:on / false:off)
  boolean apa_switch_response; // apa软开关反馈        (true:on / false:off)
  boolean rpa_switch_response; // rpa软开关反馈        (true:on / false:off)
  boolean hpp_switch_response; // hpp软开关反馈        (true:on / false:off)
  boolean aeb_switch_response; // aeb软开关反馈        (true:on / false:off)
  boolean meb_switch_response; // meb软开关反馈        (true:on / false:off)
  NotificationMainSwitch tsr_switch_response; // tsr软开关反馈        (true:on / false:off)
  boolean ihc_switch_response; // ihc软开关反馈        (true:on / false:off)
  boolean ldw_switch_response; // ldw软开关反馈        (true:on / false:off)
  SensitivityLevel ldw_level_response; // ldw敏感度等级
  boolean elk_switch_response; // elk软开关反馈        (true:on / false:off)
  boolean bsd_switch_response; // bsd软开关反馈        (true:on / false:off)
  boolean lca_switch_response; // lca软开关反馈        (true:on / false:off)
  boolean dow_switch_response; // dow软开关反馈        (true:on / false:off)
  boolean fcta_switch_response; // fcta软开关反馈       (true:on / false:off)
  boolean fctb_switch_response; // fctb软开关反馈       (true:on / false:off)
  boolean rcta_switch_response; // rcta软开关反馈       (true:on / false:off)
  boolean rctb_switch_response; // rctb软开关反馈       (true:on / false:off)
  boolean rcw_switch_response; // rcw软开关反馈        (true:on / false:off)
  boolean ldp_switch_response; // ldp软开关反馈        (true:on / false:off)
  boolean fcw_switch_response; // fcw软开关反馈        (true:on / false:off)
  boolean rads_switch_response; // rads软开关反馈        (true:on / false:off)
  boolean pa_switch_response; // pa软开关反馈        (true:on / false:off)
  boolean nra_switch_response; // nra软开关反馈        (true:on / false:off)
  SensitivityLevel fcw_level_response; // fcw敏感度等级
  boolean amap_switch_response; // amap软开关
  boolean dai_switch_response; // dai软开关
  boolean mnp_switch_response; // mnp软开关
  boolean dow_secondary_alert_switch_response;   // dow 二级警示语音软开关反馈  (true:on / false:off)
  boolean blue_light_switch_response;            // 小蓝灯软开关反馈  (true:on / false:off)
  boolean function_degrade_switch_response; // 功能降级软开关反馈    (true:on / false:off)

  SpeedSetFaileRsp speed_set_faile_rsp;                       // 速度设置失败反馈
  IntervalSetFailRsp interval_set_fail_rsp;                   // 跟车距离设置失败反馈
  ParkingPreferenceSetFailRsp parking_preference_faile_rsp;   // 泊车偏好设置失败反馈
} _STRUCT_ALIGNED_ HmiSwitchInfo;

typedef enum {
  ROUTE_SAVE_FAILURE_REASON_NONE,           // 没有故障
  ROUTE_SAVE_FAILURE_REASON_NETWORK_FAULT,  // 网络故障
  ROUTE_SAVE_FAILURE_REASON_OTHER,          // 其他原因
} _ENUM_PACKED_ RouteSaveFailureReason;

// HPP状态，和FunctionalState中HPP状态对应,修改时注意两边一致
typedef enum {
  HPP_OFF,                // 功能关闭
  HPP_PASSIVE,            // 功能抑制
  HPP_STANDBY,            // 待激活
  HPP_PROACTIVE_MAPPING,  // 主动建图
  HPP_BACKGROUND_MAPPING, // 静默建图
  HPP_MAP_CHECK,          // 地图校验
  HPP_PRE_ACTIVE,         // 握手阶段
  HPP_CRUISE_ROUTING,     // 固定路线巡航
  HPP_CRUISE_SEARCHING,   // 沿途找车位巡航
  HPP_PARKING_OUT,        // 泊出
  HPP_PARKING_IN,         // 泊入
  HPP_SUSPEND,            // 暂停
  HPP_COMPLETE,           // 完成
  HPP_ABORT,              // 异常退出HPP，缓慢停车并驻车
  HPP_ERROR,              // 系统故障
} _ENUM_PACKED_ HppStatus;

typedef enum {
  HPP_NOTIFY_NO_REQ,                      // 无消息
  // 记忆泊车功能开启            
  HPP_NOTIFY_NOT_READY = 1,            // 记忆泊车未ready 
  HPP_NOTIFY_FUNC_FAILURE = 2,         // 功能故障
  HPP_NOTIFY_EXPERIENCE_MEMORY_PARKING = 3, // 推荐体验记忆泊车（未学习过路线）
  HPP_NOTIFY_USE_MEMORY_PARKING = 4,        // 推荐可使用记忆泊车（已学习过路线）
  HPP_NOTIFY_VEHICLE_OFF_MEMORY_ROUTE = 5,  // 请开到记忆路线上，再使用记忆泊车功能
  HPP_NOTIFY_SPEED_HIGH = 6,                // 请先降低车速
  HPP_NOTIFY_SEATBELT_UNFASTENED = 7,       // 请先系好安全带
  HPP_NOTIFY_VEH_DOOR_OPEN = 8,             // 请关闭车门
  HPP_NOTIFY_POSITIONING_FAILURE = 9,       // 进入地图但定位失败
  HPP_NOTIFY_HPP_UNAVAILABLE = 10,          // 无法开启记忆泊车功能
  HPP_NOTIFY_NO_BACKWARD_DRIVING = 11,      // 挂入R挡且倒行一段距离后，导致无法学习
  HPP_NOTIFY_NEED_NEXT_ENTRY = 12,          // 学习结束后后再次激活
  HPP_NOTIFY_GEAR_MISMATCH_WHEN_HPP_UNAVAILABLE = 13, // 当前不处于D档或P挡
  HPP_NOTIFY_SYSTEM_FAILURE = 14,                     // 系统故障，点击入口时提示不可用
  HPP_NOTIFY_NARROW_ROAD_INHIBIT = 15,                // 所在环境过窄
  HPP_NOTIFY_VEH_ON_SLOPE = 16,                       // 在坡道上
  HPP_NOTIFY_HPP_AVAILABLE_WHEN_MAP_AVAILABLE = 17,   // 满足记忆泊车可用，记忆泊车按钮亮起
  HPP_NOTIFY_HPP_RESUMEABLE = 18,                     // 满足可恢复记忆泊车条件，恢复记忆泊车按钮亮起
  HPP_NOTIFY_HPP_AVAILABLE_WHEN_MAP_UNAVAILABLE = 19, // 该地库未学习过路线，记忆泊车按钮亮起
  HPP_NOTIFY_UPDATE_ROUTE = 20,                       // 该地库已有路线，但未进入该路线区域，点击HPP按钮后，打开更新路线确认弹窗
  HPP_NOTIFY_STEERING_WHEEL_ANG_INHIBIT = 21,         // 方向盘角度过大超出控制范围
  HPP_NOTIFY_LIGHT_INHIBIT = 22,                      // 光照不满足功能激活条件
  HPP_NOTIFY_START_CRUISE = 23,                       // 在HPP路线页点击开始记忆泊车按钮，进入巡航
  HPP_NOTIFY_CAR_HOOD_OPEN = 24,                      // 请关闭车前盖
  HPP_NOTIFY_CAR_TRUNK_OPEN = 25,                     // 请关闭后备箱
  HPP_NOTIFY_CAR_CHARGING_PORT = 26,                  // 请关闭充电口
  HPP_NOTIFY_CRUISE_ROUTE_INSUFFICIENCY = 27,         // 剩余巡航路线≤1m
  HPP_NOTIFY_HPP_UNAVAILABLE_DUE_TO_OTHER = 28,       // 其他原因导致的HPP不可用
  // 记忆泊车路线学习
  HPP_NOTIFY_SAVE_ROUTE = 32,                         // 静默建图成功（无图）
  HPP_NOTIFY_OVERRIDE_ROUTE = 33,                     // 静默建图成功（有图）
  HPP_NOTIFY_ROUTE_LEARNING = 34,                     // 请开往你的车位并泊入
  HPP_NOTIFY_PARKING_SLOT_FOUND = 35,                 // 选择车位后点击开始，小飞将为你自动泊入
  HPP_NOTIFY_PARKING_SLOT_NOT_FOUND = 36,             // 请手动泊入你的车位，泊入完成后请挂P档
  HPP_NOTIFY_AUTO_PARKING_IN_PROGRESS = 37,           // 自动泊入中
  HPP_NOTIFY_MANUAL_PARKING_IN_PROGRESS = 38,         // 手动泊入中
  HPP_NOTIFY_PARKING_GEAR_REQ = 39,                   // 请先挂入P档
  HPP_NOTIFY_DIS_TOO_FAR = 40,                        // 再过100米我就学不会了
  HPP_NOTIFY_SPPED_HIGH_WHEN_LEARNING = 41,           // 请降低车速
  HPP_NOTIFY_COMPLETE_LEARNING_WHEN_MANUAL_PARKING = 42, // 用户手动泊入车位，在P挡下点击“完成学习”
  HPP_NOTIFY_COMPLETE_LEARNING_WHEN_AUTO_PARKING = 43,   // 自动泊入车位，完成学习
  HPP_NOTIFY_PARKING_POSITION_FALSE = 44,                // 建图过程中压限位器
  HPP_NOTIFY_LEARNING_TIMEOUT = 45,                      // 建图超时
  HPP_NOTIFY_ROUTE_SAVING_FAILURE = 46,                  // 转入后台保存，保存失败后
  HPP_NOTIFY_ROUTE_SAVING_SUCCESS = 47,                  // 转入后台保存，保存成功后
  HPP_NOTIFY_ROUTE_SAVING_FAILURE_AND_JUMP_INTERFACE = 48, // 在学习界面，路线保存失败
  HPP_NOTIFY_RETURN_APA = 49,                              // 退出路线学习，界面返回APA
  HPP_NOTIFY_MEMORY_LIMIT_EXCEEDED_RETURN = 50,            // 超过记忆上限，记忆泊车退出
  HPP_NOTIFY_HIGH_SPEED_RETURN = 51,                       // 已超速，记忆泊车已退出
  HPP_NOTIFY_LEARNING_FAILURE_DUE_TO_P_SHIFT = 52,         // 学习路线中（非车位内）挂P档，跳转学习失败界面
  HPP_NOTIFY_LEARNING_FAILURE_DUE_TO_REVERSE = 53,         // 学习中在 R 挡下长距离倒车时（超过20m）
  HPP_NOTIFY_LEARNING_FAILURE_DUE_TO_ROUTE_OVERLAP = 54,   // 学习中在路线重叠（超过20m）
  HPP_NOTIFY_LEARNING_FAILURE_DUE_TO_OTHER = 55,           // 因为其他原因退出
  HPP_NOTIFY_LEARNING_FAILURE_JUMP_DUE_TO_ROUTE_INFO = 56, // 失败原因-路线信息不足
  HPP_NOTIFY_LEARNING_FAILURE_JUMP_DUE_TO_RAMP_INHIBIT = 57, // 失败原因-坡度过大
  HPP_NOTIFY_LEARNING_FAILURE_JUMP_DUE_TO_OTHER = 58,        // 失败原因-其它原因
  HPP_NOTIFY_LEARNING_FAILURE_DUE_TO_ADAS_ACTIVE = 59,       // 失败原因-激活其他智驾功能
  HPP_NOTIFY_LEARNING_FAILURE_DUE_TO_CAR_CHARGING_PORT = 60, // 失败原因-充电口打开
  HPP_NOTIFY_LEARNING_FAILURE_DUE_TO_CAR_DOOR = 61,          // 失败原因-车门打开时
  HPP_NOTIFY_LEARNING_FAILURE_DUE_TO_CAR_HOOD = 62,          // 失败原因-车前盖打开时
  HPP_NOTIFY_LEARNING_FAILURE_DUE_TO_CAR_TRUNK = 63,         // 失败原因-后备箱打开时
  HPP_NOTIFY_LEARNING_FAILURE_DUE_TO_REARVIEW_MIRROR = 64,   // 失败原因-后视镜
  HPP_NOTIFY_LEARNING_UNAVAILABLE_DUE_TO_OTHER = 65,         // 其他原因导致的建图不可用
  // 记忆泊车
  HPP_NOTIFY_LEARNING_FAILURE_DUE_TO_CAMERA_FAIL = 66,       // 失败原因-摄像头故障
  HPP_NOTIFY_LEARNING_FAILURE_DUE_TO_USS_FAIL = 67,          // 失败原因-雷达故障
  HPP_NOTIFY_LEARNING_FAILURE_DUE_TO_SYSTEM_FAIL = 68,       // 失败原因-系统故障
  HPP_NOTIFY_LEARNING_FAILURE_DUE_TO_COMPONENTS_FAIL = 69,   // 失败原因-关联系统故障
  HPP_NOTIFY_LEARNING_FAILURE_DUE_TO_CAMERA_BLOCKAGE = 70,   // 失败原因-摄像头遮挡
  HPP_NOTIFY_LEARNING_FAILURE_DUE_TO_ABNORMAL_TIRE_PRESSURE = 71,  // 失败原因-胎压异常
  HPP_NOTIFY_LEARNING_FAILURE_DUE_TO_SEAT_BELT = 72,         // 失败原因-未系安全带
  HPP_NOTIFY_LEARNING_FAILURE_DUE_TO_ESP_OFF = 73,           // 失败原因-ESP关闭
  HPP_NOTIFY_LEARNING_FAILURE_DUE_TO_CHARGING = 74,          // 失败原因-充电中
  HPP_NOTIFY_PRESS_COMPLETE_OR_STARTPARKING = 75,            //点击完成或开始泊车完成路线学习
  HPP_NOTIFY_PRESS_COMPLETE = 76,                            //点击完成路线学习
  HPP_NOTIFY_ROUTE_SAVING = 77,                              //路线生成中
  HPP_NOTIFY_AVOIDING_OBSTACLE_TAKEOVER_REQUEST = 78,        // 避障等待时，用户接管请求
  HPP_NOTIFY_CANNOT_ACCELERATE = 79,        // 巡航中无法加速
  HPP_NOTIFY_RELEASE_BRAKE = 80,            // 用户激活时踩住制动踏板不放
  HPP_NOTIFY_SURROUNDING_AWARENESS = 81,    // 用户松开刹车后，开始自动巡航
  HPP_NOTIFY_TURN_LEFT = 82,                // 即将左转
  HPP_NOTIFY_TURN_RIGHT = 83,               // 即将右转
  HPP_NOTIFY_CROSSROAD_ATTENTION = 84,      // 即将经过路口，提醒注意安全
  HPP_NOTIFY_AVOIDING_CAR = 85,             // 避让前或侧方车辆
  HPP_NOTIFY_ENTER_RAMP = 86,               // 上坡/下坡
  HPP_NOTIFY_PEDESTRIAN_AWARENESS = 87,     // 遭遇行人，避障等待
  HPP_NOTIFY_AVOIDING_OBSTACLE = 88,        // 遭遇障碍物，绕行或避障等待时
  HPP_NOTIFY_APPROACHING_TARGET_SPOT = 89,  // 前方接近常用车位附近
  HPP_NOTIFY_APPROACHING_PARKING_SPOT = 90, // 到达常用车位附近，找到车位且路径规划成功
  HPP_NOTIFY_TARGET_SPOT_OCCUPIED_AND_FIND_OTHER = 91, // 到达目标车位附近，目标车位被占但找到其他车位
  HPP_NOTIFY_TARGET_SLOT_OCCUPIED = 92,     // 到达常用车位附近时，车位被占用，且无其他可泊车位，回到APA模式并挂P档
  HPP_NOTIFY_HPP_QUIT = 93,                 // 主动介入接管退出 HPP
  HPP_NOTIFY_HPP_QUIT_TAKE_OVER1 = 94,      // 避障等待超时、司机解开安全带、开车门、打开前后盖、关闭后视镜，紧急退出 HPP，并挂P档
  HPP_NOTIFY_HPP_QUIT_TAKE_OVER2 = 95,      // 与VCU/EPS握手失败
  HPP_NOTIFY_HPP_PARKING_SUCCESS = 96,      // HPP泊车完成
  HPP_NOTIFY_HPP_PARKING_FAILURE = 97,      // HPP泊车失败
  HPP_NOTIFY_LONG_DURATION_FOLLOWING = 98,  // 后方长时间跟车
  // 泊入
  HPP_NOTIFY_PARKING_AND_WATCH_OUT = 99,    // 泊车进行时
  HPP_NOTIFY_PARKING_CONTINUE = 100,        // 泊车继续时
  HPP_NOTIFY_PARKING_GEAR_SHIFT_UNRESPONSIVE = 101, // 系统控车泊入时，驾驶员换挡，不执行
  HPP_NOTIFY_PARKING_ERRATIC_OBSTRUCTIVE = 102,     // 泊入过程中识别到路径上障碍物
  HPP_NOTIFY_PARKING_SUSPEND_NO_BREAK = 103,        // 暂停时，用户未踩住刹车时，泊车继续按钮置灰
  HPP_NOTIFY_PARKING_BREAK_SUSPEND = 104,           // 系统控车泊入时，泊车暂停（驾驶员踩刹车）
  HPP_NOTIFY_PARKING_DOOR_OPEN_SUSPEND = 105,       // 系统控车泊入时，泊车暂停（打开车门）
  HPP_NOTIFY_PARKING_HATCH_OPEN_SUSPEND = 106,      // 系统控车泊入时，泊车暂停（打开车前盖）
  HPP_NOTIFY_PARKING_TRUNK_OPEN_SUSPEND = 107,      // 系统控车泊入时，泊车暂停（打开车后盖）
  HPP_NOTIFY_PARKING_SEATBELT_SUSPEND = 108,        // 系统控车泊入时，泊车暂停（驾驶员松开安全带）
  HPP_NOTIFY_PARKING_ACC_SUSPEND = 109,             // 系统控车泊入时，泊车暂停（驾驶员踩油门）
  HPP_NOTIFY_PARKING_RELEASE_BREAK_CONTINUE = 110,  // 暂停恢复时，驾驶员踩住了刹车
  HPP_NOTIFY_PARKING_STATIC_OBSTRUCTIVE = 111,      // 泊入路径上有不可自主移动障碍物且无法重规划，跳暂停界面
  HPP_NOTIFY_PARKING_SUSPEND_RELEASE_ACC = 112,     // 泊车已暂停，且踩住加速踏板，继续按钮置灰
  HPP_NOTIFY_PARKING_SUSPEND_CLOSE_DOOR = 113,      // 泊车已暂停，且车门打开，继续按钮置灰
  HPP_NOTIFY_PARKING_SUSPEND_CLOSE_HATCH = 114,     // 泊车已暂停，且车前盖打开，继续按钮置灰
  HPP_NOTIFY_PARKING_SUSPEND_CLOSE_TRUNK = 115,     // 泊车已暂停，且车后备箱打开，继续按钮置灰
  HPP_NOTIFY_PARKING_SUSPEND_FASTEN_SEATBELT = 116, // 泊车已暂停，且未系安全带，继续按钮置灰
  HPP_NOTIFY_PARKING_SUSPEND = 117,                 // 泊车已暂停，且无干扰恢复项
  HPP_NOTIFY_PARKING_QUIT_TAKE_OVER = 118,          // 请立即接管车辆，记忆泊车已退出
  HPP_NOTIFY_PARKING_QUIT = 119,                    // 记忆泊车已退出
   // 记忆泊车功能开启
  HPP_NOTIFY_HPP_DISABLE = 151,             // 记忆泊车功能未开启
  HPP_NOTIFY_IVI_SW_MISMATCH = 152,         // 请联系客服处理，记忆泊车暂不可用
  HPP_NOTIFY_CAMERA_FAIL = 153,             // 摄像头故障，记忆泊车暂不可用
  HPP_NOTIFY_USS_FAIL = 154,                // 雷达故障，记忆泊车暂不可用
  HPP_NOTIFY_COMPONENTS_FAIL = 155,         // 关联系统故障，记忆泊车暂不可用
  HPP_NOTIFY_CAMERA_BLOCKAGE = 156,         // 摄像头遮挡，记忆泊车暂不可用
  HPP_NOTIFY_VEH_NOT_READY = 157,           // 车辆未启动，记忆泊车暂不可用
  HPP_NOTIFY_ABNORMAL_TIRE_PRESSURE = 158,  // 胎压异常，记忆泊车暂不可用
  HPP_NOTIFY_NOT_UNDER_GROUND = 159,        // 车辆未在地库，记忆泊车暂不可用
  HPP_NOTIFY_ROUTE_FULL = 160,              // 路线库已满，记忆泊车暂不可用
  HPP_NOTIFY_CHARGING = 161,                // 充电中，记忆泊车暂不可用
  HPP_NOTIFY_EPB_NOT_RELEASE = 162,         // EPB未释放，记忆泊车暂不可用  
   // 记忆泊车
  HPP_NOTIFY_PEDESTRIAN_WARNING = 181,      // 请注意行人
  HPP_NOTIFY_AVOIDING_PEDESTRIAN = 182,     // 正在避让行人
  HPP_NOTIFY_NARROW_ROAD = 183,             // 道路狭窄，请注意安全
  HPP_NOTIFY_COMPLEX_ENVIRONMENT = 184,     // 环境复杂，请注意安全
  HPP_NOTIFY_ABOUT_TO_PARK_IN_PARKING_SPACE = 185,  // 即将泊入你的车位
  HPP_NOTIFY_HMI_HPP_OUTPUT= 186,           // 松开刹车，开始记忆泊车
  HPP_NOTIFY_LEARNING_FAILURE_DUE_TO_CLICK_EXIT_BUTTON = 212,  // 失败原因-点击退出按钮
  HPP_NOTIFY_LEARNING_FAILURE_DUE_TO_TIMEOUT = 213,  //失败原因-学习超时
} _ENUM_PACKED_ HppNotifyReq;

typedef struct {
  boolean available; // 点是否有效
  Point3f point;
} _STRUCT_ALIGNED_ Traj_Point;

typedef enum {
  HPPButtonState_None                           = 0,  // 无按钮提示
  Complete_Unavailable_Cancel                   = 1,  // 完成按钮(置灰)+取消按钮
  Complete_Available_Cancel                     = 2,  // 完成按钮(高亮)+取消按钮
  Complete_Unavailable_StartParking_Unavailable = 3,  // 完成按钮(置灰)+开始泊车按钮(置灰）
  Complete_Unavailable_StartParking_Available   = 4,  // 完成按钮(置灰)+开始泊车按钮(高亮）
  Complete_Available_StartParking_Unavailable   = 5,  // 完成按钮(高亮)+开始泊车按钮(置灰）
  Complete_Available_StartParking_Available     = 6,  // 完成按钮(高亮)+开始泊车按钮(高亮）
} _ENUM_PACKED_ HPPButtonState;

typedef enum {
    HPP_Remind_NONE = 0,  // 无消息
    HPP_Remind_Gear_P_Request = 1,  // 请挂入P档
    HPP_Remind_Drive_Morethan_10_Meters = 2,  //请行驶超过10m后点击完成
    HPP_Remind_Park_in_Parking_Space = 3,  // 请泊入车位后点击完成
} _ENUM_PACKED_ HppRemind;

typedef enum  {
  Notify_None = 0,  // 无
  HPP_Notify  = 1,  // HPA文言
  APA_Notify  = 2,  // APA文言
} _ENUM_PACKED_ NotifyType;

// HMI输出信息
typedef struct {
  HppNotifyReq hpp_notify_req;              // 座舱交互提示

  HppStatus hpp_status;                     // 记忆泊车状态
  boolean memory_parking_available;         // 记忆泊车是否可用
  boolean is_first_time_using;              // 是否首次使用记忆泊车(当前停车位置无历史路线)
  boolean memory_parking_resume_available;  // 记忆泊车是否可恢复 
  boolean route_learning_available;         // 路线学习是否可用
  uint32 speed_bumps_count;                 // 减速带计数
  float32 learning_distance;                // 已学习距离           (米)
  float32 distance_to_parking_space;        // 距离泊车车位距离     (米)
  float32 estimated_remaining_time;         // 到达终点的预估时间   (秒)

  uint32 pedestrian_avoidance_count;  // 避让行人次数
  uint32 vehicle_avoidance_count;     // 避让车辆次数
  uint32 hpp_time_minute;             // 路线记忆时间、记忆泊车时间   (分钟)
  TakeoverReqLevel hpp_takeover_req_lv; // 请求接管提醒等级
  boolean is_position_need_refresh;  // 是否需要刷新定位
  uint8_t his_traj_point_size;       // 历史轨迹点数量
  Point3f his_traj_point[HMI_HPP_HIS_TRAJ_POINT_MAX_NUM]; // 历史轨迹点(米)
  Traj_Point start_point;            // 起点(米)
  Traj_Point end_point;              // 终点(米)
  boolean hpp_active_resp;              // 是否激活
  boolean hpp_active_denied;            // 激活失败
  FuncButtonState hpa_active_button_sts; // 记忆泊车按钮
  uint32 parking_space_count;               // 记忆泊车/学习时感知到的车位数量
  uint32 route_saving_process;              // 路线保存百分比
  uint32 guidance_step;                     // 记忆泊车三部引导步骤
  float32 cruise_distance;                    // 已巡航距离           (米)
  boolean gear_p_request;                      // 请挂入P档
  HPPButtonState hpp_button_display;        // 记忆泊车按钮显示
  HppRemind hpp_Remind;                     //小弹窗提示
  NotifyType notify_type_recommendation;     //记忆泊车文言类型显示
} _STRUCT_ALIGNED_ HmiHppOutput;

typedef struct {
  int32 relative_id;          // 备注:当前车道id为0，向左依次-1，向右依次+1
  uint8 lane_types_size;                                                // 车道类型数量                            
  LaneTypeMsg lane_types[CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM];     // 车道类型列表
  uint8 merge_split_points_size;                                                            // 车道分合流数量
  LaneMergeSplitPointData merge_split_points[CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM];     // 车道分合流信息
  float32 speed_limit;       // 限速值       (公里/小时)
} _STRUCT_ALIGNED_ HmiLaneInfo;

typedef struct {
  Point2f point[2]; // 每个停止线两个端点
} _STRUCT_ALIGNED_ HmiStopLine;

typedef struct {
  Point2f ground_marking_points_set[HMI_GROUND_MARKING_POINTS_NUM];   // 地面标识点集
  float32 orientation_angle;                                          // 地面标识的朝向角度
  LaneDrivableDirection turn_type;                                    // 地面标识对应的车道通行方向
} _STRUCT_ALIGNED_ HmiLaneGroundMarking;

// RADS状态
typedef enum {
  RADS_OFF = 0,
  RADS_PASSIVE = 1,
  RADS_STANDBY = 2,
  RADS_PRE_ACTIVE = 3,
  RADS_TRACING = 4,
  RADS_SUSPEND = 5,
  RADS_COMPLETE = 6,
  RADS_ABORT = 7,
  RADS_ERROR = 8,
} _ENUM_PACKED_ RadsStatus;

typedef enum {
  RADS_NOTIFY_NOT_READY = 0,
  RADS_NOTIFY_FUNC_FAILURE = 1,            // 倒车循迹功能故障
  RADS_NOTIFY_NO_ENTER_VEH_DOOR_OPEN = 2,  // 车门未关，请关闭车门，倒车循迹不可用，AVM界面点击循迹倒车功能按钮，还没进入循迹倒车功能界面
  RADS_NOTIFY_NO_ENTER_VEH_HATCH_OPEN = 3, // 车前盖未关，请关闭车前盖，倒车循迹不可用，AVM界面点击循迹倒车功能按钮，还没进入循迹倒车功能界面
  RADS_NOTIFY_NO_ENTER_VEH_TRUNK_OPEN = 4, // 后备箱未关，请关闭后备箱，倒车循迹不可用，AVM界面点击循迹倒车功能按钮，还没进入循迹倒车功能界面
  RADS_NOTIFY_VEH_NOT_READY = 5,           // 车辆未启动，倒车循迹不可用
  RADS_NOTIFY_SYSTEM_UNAVAILABLE = 6,      // 系统故障，倒车循迹不可用
  RADS_NOTIFY_HIGH_SPEED = 7,              // 速度超限（＞0kph），请降低车速，倒车循迹不可用
  RADS_NOTIFY_NO_ACTIVE_VEH_DOOR_OPEN = 8, // 车门未关，请关闭车门，在循迹倒车界面，还没有点开始倒车
  RADS_NOTIFY_NO_ACTIVE_VEH_HATCH_OPEN = 9,  // 前盖未关，请关闭车前盖，在循迹倒车界面，还没有点开始倒车
  RADS_NOTIFY_NO_ACTIVE_VEH_TRUNK_OPEN = 10, // 后备箱未关，请关闭后备箱，在循迹倒车界面，还没有点开始倒车
  RADS_NOTIFY_NARROW_ROAD_INHIBIT = 11,      // 所在环境过窄
  RADS_NOTIFY_STEERING_WHEEL_ANG_INHIBIT = 12, // 方向盘角度过大超出控制范围
  RADS_NOTIFY_LIGHT_INHIBIT = 13,          // 光照不满足功能激活条件
  RADS_NOTIFY_VEH_ON_SLOPE = 14,           // 坡度超限，循迹倒车暂不可用
  RADS_NOTIFY_SEATBELT_UNFASTENED = 15,    // 安全带未系，循迹倒车暂不可用
  RADS_NOTIFY_CAR_CHARGING_PORT = 16,      // 充电中，循迹倒车暂不可用
  RADS_NOTIFY_GEAR_NOT_R = 17,             // 当前不处于R档, 循迹倒车暂不可用
  RADS_NOTIFY_RELEASE_BRAKE = 18,          // 用户激活时踩住制动踏板不放
  RADS_NOTIFY_SURROUNDING_AWARENESS = 19,  // 循迹过程中，请注意周围环境
  RADS_NOTIFY_TURN_LEFT = 20,              // 即将左转
  RADS_NOTIFY_TURN_RIGHT = 21,             // 即将右转
  RADS_NOTIFY_CROSSROAD_ATTENTION = 22,    // 即将经过路口，提醒注意安全
  RADS_NOTIFY_AVOIDING_CAR = 23,           // 避让车辆
  RADS_NOTIFY_ENTER_NARROW_ROAD = 51,      // 正在通过窄路
  RADS_NOTIFY_ENTER_RAMP = 25,             // 上坡/下坡
  RADS_NOTIFY_PEDESTRIAN_AWARENESS = 26,   // 遭遇行人，避障等待
  RADS_NOTIFY_AVOIDING_OBSTACLE = 27,      // 遭遇障碍物，绕行
  RADS_NOTIFY_BREAK_SUSPEND = 28,          // 倒车循迹时，暂停（驾驶员踩刹车）
  RADS_NOTIFY_DOOR_OPEN_SUSPEND = 29,      // 倒车循迹时，暂停（打开车门）
  RADS_NOTIFY_HATCH_OPEN_SUSPEND = 30,     // 倒车循迹时，暂停（打开车前盖）
  RADS_NOTIFY_TRUNK_OPEN_SUSPEND = 31,     // 倒车循迹时，暂停（打开车后盖）
  RADS_NOTIFY_SEATBELT_SUSPEND = 32,       // 倒车循迹时，暂停（驾驶员松开安全带）
  RADS_NOTIFY_ACC_SUSPEND = 33,            // 倒车循迹时，暂停（驾驶员踩油门）
  RADS_NOTIFY_MIRROR_FOLD_SUSPEND = 34,    // 倒车循迹时, 暂停（折叠后视镜）
  RADS_NOTIFY_SWITCH_OFF_QUIT = 35,        // 软开关点击退出
  RADS_NOTIFY_SPEED_ABNORMAL = 36,         // 超速，功能终止（瞬时车速超限＞xx）暂停状态下发生位移，循迹倒车退出
  RADS_NOTIFY_SUSPEND_TIMEOUT_QUIT = 37,   // 暂停界面停留60s后，自动退出
  RADS_NOTIFY_TAKE_OVER_QUIT = 38,    // 主动介入接管退出，驾驶员干预，功能终止（方向盘，挡位）
  RADS_NOTIFY_FUN_FAILURE_QUIT = 39,  // 系统故障、异常情况紧急退出，并挂P档
  RADS_NOTIFY_COMPLETE = 40,          // 倒车循迹完成
  RADS_NOTIFY_MIRROR_OFF = 41,        // 后视镜关闭，循迹倒车暂不可用，AVM界面点击循迹倒车功能按钮，还没进入循迹倒车功能界面
  RADS_NOTIFY_DISTANCE_SHORT = 42,    // 记忆路线长度不满足循迹开始条件
  RADS_NOTIFY_START_AVAILABLE = 43,   // 倒车循迹可开始
  RADS_NOTIFY_REVERSE_START = 44,     // 倒车已开始，请注意周围环境
  RADS_NOTIFY_WAIT_OBSTACLE = 45,     // 遭遇障碍物，等待
  RADS_NOTIFY_VERSION_UNMATCHED = 46, // 版本不匹配，倒车循迹不可用
  RADS_NOTIFY_CAMERA_FAULT = 47,    // 摄像头故障，倒车循迹不可用
  RADS_NOTIFY_RADAR_FAULT = 48,     // 雷达故障，倒车循迹不可用
  RADS_NOTIFY_ASSOCIATED_SYSTEM_FAULT = 49,    // 关联系统故障，倒车循迹不可用
  RADS_NOTIFY_CAMERA_OBSTRUCTION = 50,         // 摄像头遮挡，倒车循迹不可用
  //RADS_NOTIFY_MIRROR_FOLDING = 51,             // 正在折叠后视镜
  RADS_NOTIFY_DRIVING_DISTANCE_SHORT = 52,     // 记录数据无效或距离短时，无法激活
  RADS_NOTIFY_ENVIRONMENT_ABNORMAL = 53,       // 环境异常，倒车循迹不可用
  RADS_NOTIFY_OPEN_MIRROR_REQ = 54,            // 后视镜关闭，请打开后视镜，在循迹倒车界面，还没有点开始倒车
  RADS_NOTIFY_FASTEN_SEATBELT_REQ = 55,        // 未系安全带，请系好安全带，在循迹倒车界面，还没有点开始倒车
  RADS_NOTIFY_BRAKE_STOP_REQ = 56,             // 用户未踩刹车导致车辆移动，请刹停车辆，在循迹倒车界面，还没有点开始倒车
  RADS_NOTIFY_RESUME_AVAILABLE = 57,           // 暂停条件解除，用户未点击继续
  RADS_NOTIFY_CAMERA_FAULT_EXIT = 58,          // 摄像头故障，倒车循迹退出
  RADS_NOTIFY_RADAR_FAULT_EXIT = 59,           // 雷达故障，倒车循迹退出
  RADS_NOTIFY_SYSTEM_FAULT_EXIT = 60,          // 系统故障，倒车循迹退出
  RADS_NOTIFY_ASSOCIATED_SYSTEM_FAULT_EXIT = 61, // 关联系统故障，倒车循迹退出
  RADS_NOTIFY_CAMERA_OBSTRUCTION_EXIT = 62,      // 摄像头遮挡，倒车循迹退出
  RADS_NOTIFY_HIGH_SPEED_EXIT = 63,              // 车速超调，功能终止（车速超调＞1kph），倒车循迹不可用
  RADS_NOTIFY_ESP_OFF_EXIT = 64,                 // ESP关闭，功能终止
  RADS_NOTIFY_EPB_ACTIVE_EXIT = 65,              // EPB拉起，功能终止
  RADS_NOTIFY_TRAJECTORY_ABNORMAL_EXIT = 66,          // 轨迹超限：控制和规划误差过大，功能终止
  RADS_NOTIFY_SLOPE_ABNORMAL_EXIT = 67,               // 坡度超限，大于15%，功能终止
  RADS_NOTIFY_ADAS_ACTIVE_EXIT = 68,                  // 其他功能激活或关联功能激活（如ABS/TCS/ESP等），功能终止
  RADS_NOTIFY_NUMBER_OF_INTERRUPTIONS_OVERLIMIT = 69, // 中断次数超限，功能终止
  RADS_NOTIFY_INTERRUPTION_TIME_OUT= 70,              // 中断超时，功能终止
  RADS_NOTIFY_PARKING_TIME_OUT = 71,                  // 泊车超时，功能终止
  RADS_NOTIFY_SPACE_INSUFFICIENT_EXIT= 72,            // 空间不足，功能终止（空间受限，过程中规划失败）
  RADS_NOTIFY_TIRE_PRESSURE_ABNORMAL_EXIT = 73,       // 胎压异常，功能终止
  RADS_NOTIFY_ENVIRONMENT_ABNORMAL_EXIT = 74,         // 环境异常，功能终止（通用）
  RADS_NOTIFY_DOOR_OPEN_EXIT = 75,          // 车门开启，功能终止
  RADS_NOTIFY_TRUNK_OPEN_EXIT = 76,         // 后备箱开启，功能终止
  RADS_NOTIFY_HATCH_OPEN_EXIT = 77,         // 机舱盖开始，功能终止
  RADS_NOTIFY_CHARGING_EXIT = 78,           // 充电中，功能终止，A/B流程（searching/控车状态）复用
  RADS_NOTIFY_ADAS_ACTIVE = 79,             // 其他功能(ABS/TCS/ESP)激活，循迹倒车不可用
  RADS_NOTIFY_TIRE_PRESSURE_ABNORMAL = 80,  // 胎压异常，循迹倒车不可用
  RADS_NOTIFY_BREAK_THEN_START = 81,        // 车辆静止&&有可用循迹倒车路线&&（刹车未踩下&&EPB未拉起），异常暂停
  RADS_NOTIFY_BREAK_RELEASED = 82,          // 系统未控车，车辆溜车
  RADS_NOTIFY_RELEASE_ACC_PEDAL = 83,       // 车辆静止&&(刹车踩‖EPB拉起)&&有可用循迹到车路线&&油门踏板踩住时，无法进入B流程
  RADS_NOTIFY_MAIN_SWITCH_OFF = 84,         // 设置项未开启，循迹倒车暂不可用
  RADS_NOTIFY_SHAKE_HANDS_FAIL = 85,        // 执行器握手失败，无法开始循迹倒车
} _ENUM_PACKED_ RadsNotifyReq;

typedef struct {
  RadsStatus rads_status;                   // RADS状态机状态
  RadsNotifyReq rads_notify_req;            // 座舱交互提示
  float32 rads_remain_distance;             // 倒车剩余距离 (米)
  float32 remain_distance_percentage;          // 倒车剩余进度 (0.00 - 1.00)
  boolean rads_active_resp;                    // 是否激活
  boolean rads_active_denied;                  // 激活失败
  uint16 path_point_size;                     // 路径点数量
  Point2f path_point[RADS_MAP_POINT_MAX_NUM];  // 路径点
  FuncButtonState rads_active_button_sts;      // 循迹倒车按钮
  FuncButtonState rads_start_button_sts;       // 开始倒车按钮
  FuncButtonState rads_continue_button_sts;    // 继续倒车按钮
} _STRUCT_ALIGNED_ HmiRadsInfo;

// 一键贴边状态
typedef enum {
  PA_OFF = 0,
  PA_PASSIVE = 1,
  PA_STANDBY = 2,
  PA_PRE_ACTIVE = 3,
  PA_GUIDANCE = 4,
  PA_SUSPEND = 5,
  PA_COMPLETE = 6,
  PA_ABORT = 7,
  PA_ERROR = 8,
} _ENUM_PACKED_ PaStatus;

// 一键贴边座舱交互提示
typedef enum {
  PA_NOTIFY_NONE = 0,
  PA_NOTIFY_FUNC_AVAILABLE = 1,            // 功能可开启
  PA_NOTIFY_RELEASE_BREAK = 2,             // 点击开始或继续，未松开刹车
  PA_NOTIFY_PARKING_START = 3,             // 点击开始，松开刹车
  PA_NOTIFY_SURROUNDING_AWARENESS = 4,     // 请注意周围环境
  PA_NOTIFY_COMPLETE = 5,                  // 一键贴边已完成
  PA_NOTIFY_WAIT_OBSTACLE = 6,             // 障碍物阻挡，请注意周围环境
  PA_NOTIFY_VERSION_UNMATCHED = 7,         // 版本不匹配
  PA_NOTIFY_CAMERA_FAULT = 8,              // 相机故障，倒车循迹不可用
  PA_NOTIFY_RADAR_FAULT = 9,               // 雷达故障，倒车循迹不可用
  PA_NOTIFY_SYSTEM_FAULT = 10,             // 系统故障，倒车循迹不可用
  PA_NOTIFY_ASSOCIATED_SYSTEM_FAULT = 11,  // 关联系统故障，倒车循迹不可用
  PA_NOTIFY_CAMERA_OBSTRUCTION = 12,       // 相机遮挡，倒车循迹不可用
  PA_NOTIFY_VEH_NOT_READY = 13,            // 车辆未启动，倒车循迹不可用
  PA_NOTIFY_VEH_DOOR_OPEN = 14,            // 请关闭车门
  PA_NOTIFY_VEH_HATCH_OPEN = 15,           // 请关闭车前盖
  PA_NOTIFY_VEH_TRUNK_OPEN = 16,           // 请关闭后备箱
  PA_NOTIFY_HIGH_SPEED = 17,               // 车速超调，倒车循迹不可用
  PA_NOTIFY_VEH_ON_SLOPE = 18,             // 坡度超限，循迹倒车暂不可用
  PA_NOTIFY_ADAS_ACTIVE = 19,              // 其他原因导致的建图不可用
  PA_NOTIFY_TIRE_PRESSURE_ABNORMAL = 20,   // 胎压异常，功能终止
  PA_NOTIFY_ENVIRONMENT_ABNORMAL = 21,     // 其他原因导致的建图不可用
  PA_NOTIFY_CHARGING = 22,                 // 充电中
  PA_NOTIFY_SEATBELT_UNFASTENED = 23,      // 未系安全带
  PA_NOTIFY_CLOSE_DOOR_REQ = 24,           // 请关闭车门
  PA_NOTIFY_CLOSE_TRUNK_REQ = 25,          // 请关闭后备箱
  PA_NOTIFY_CLOSE_HATCH_REQ = 26,          // 请关闭车前盖
  PA_NOTIFY_DIRECTION_UNAVAILABLE = 27,    // 无可用贴边方向
  PA_NOTIFY_RELEASE_ACC_PEDAL = 28,        // 车辆静止，踩油门
  PA_NOTIFY_BRAKE_SUSPEND = 29,            // 踩刹车暂停
  PA_NOTIFY_ACC_SUSPEND = 30,              // 干预油门暂停
  PA_NOTIFY_OPEND_DOOR_SUSPEND = 31,       // 打开车门暂停
  PA_NOTIFY_OPEND_TRUNK_SUSPEND = 32,      // 打开后备箱暂停
  PA_NOTIFY_OPEND_HATCH_SUSPEND = 33,      // 打开前舱盖暂停
  PA_NOTIFY_UNFASTENED_SEATBELT_SUSPEND = 34, // 解开安全带
  PA_NOTIFY_WAIT_OBSTACLE_SUSPEND = 35,       // 障碍物阻挡暂停
  PA_NOTIFY_RESUME_AVAILABLE = 36,            // 暂停条件解除，用户未点击继续
  PA_NOTIFY_CONTINUE_PARKING = 37,            // 暂停条件解除，用户点击继续
  PA_NOTIFY_CAMERA_FAULT_EXIT = 38,           // 相机故障，功能退出
  PA_NOTIFY_RADAR_FAULT_EXIT = 39,            // 雷达故障，功能退出
  PA_NOTIFY_SYSTEM_FAULT_EXIT = 40,           // 系统故障，功能退出
  PA_NOTIFY_ASSOCIATED_SYSTEM_FAULT_EXIT = 41, // 关联系统故障，功能退出
  PA_NOTIFY_CAMERA_OBSTRUCTION_EXIT = 42,    // 相机遮挡，功能退出 
  PA_NOTIFY_HIGH_SPEED_EXIT = 43,            // 超速，功能终止
  PA_NOTIFY_SPEED_ABNORMAL_EXIT = 44,        // 车速超调，功能终止
  PA_NOTIFY_ESP_OFF_EXIT = 45,               // ESP关闭，功能终止
  PA_NOTIFY_EPB_ACTIVE_EXIT = 46,            // EPB拉起，功能终止
  PA_NOTIFY_TRAJECTORY_ABNORMAL_EXIT = 47,   // 轨迹超限，功能终止
  PA_NOTIFY_SLOPE_ABNORMAL_EXIT = 48,        // 坡度超限，功能终止
  PA_NOTIFY_ADAS_ACTIVE_EXIT = 49,           // 其他功能激活，功能终止
  PA_NOTIFY_NUMBER_OF_INTERRUPTIONS_OVERLIMIT_EXIT = 50, // 中断次数超限，功能终止
  PA_NOTIFY_INTERRUPTION_TIME_OUT_EXIT = 51, // 中断超时，功能终止
  PA_NOTIFY_PARKING_TIME_OUT_EXIT = 52,      // 泊车超时，功能终止
  PA_NOTIFY_STEP_TIMES_EXCEED_EXIT = 53,     // 超步，功能终止
  PA_NOTIFY_DRIVER_INTERVENTION_EXIT = 54,   // 驾驶员干预,功能终止
  PA_NOTIFY_SPACE_INSUFFICIENT_EXIT= 55,       // 空间不足，功能终止
  PA_NOTIFY_TIRE_PRESSURE_ABNORMAL_EXIT = 56,  // 胎压异常，功能终止
  PA_NOTIFY_CHARGING_AND_PARKING_EXIT = 57,    // 充电中，泊车退出
  PA_NOTIFY_ENVIRONMENT_ABNORMAL_EXIT = 58,    // 环境异常，功能终止
  PA_NOTIFY_BREAK_THEN_START = 59,             // 开始前未踩刹车
  PA_NOTIFY_BREAK_RELEASED = 60,               // 系统未控车，车辆溜车
} _ENUM_PACKED_ PaNotify;

typedef struct {
  PaStatus pa_status;         // PA状态机状态
  PaNotify pa_notify;         // 座舱交互提示
  boolean pa_active_resp;     // 是否激活
  boolean pa_active_denied;   // 激活失败
  float pa_remain_distance;   // 剩余距离 (米)
  float remain_distance_percentage;  // 剩余进度（0.00-1.00）
  PaDirection pa_direction;          // 用户选择的贴边方向
  PaDirection available_direction[HMI_PA_DIRECTION_AVAILABLE_MAX_NUM]; // 可选择的贴边方向
  FuncButtonState ftba_active_button_sts;   // 一键贴边按钮(预留)
  FuncButtonState pa_continue_button_sts;   // 继续一键贴边按钮
  FuncButtonState pa_start_button_sts;   // 开始一键贴边按钮
} _STRUCT_ALIGNED_ HmiPaInfo;

// 窄路通行状态
typedef enum {
  NRA_OFF = 0,
  NRA_PASSIVE = 1,
  NRA_STANDBY = 2,
  NRA_PRE_ACTIVE = 3,
  NRA_GUIDANCE = 4,
  NRA_SUSPEND = 5,
  NRA_COMPLETE = 6,
  NRA_ABORT = 7,
  NRA_ERROR = 8,
} _ENUM_PACKED_ NraStatus;

// 窄路通行状态
typedef enum {
  NRA_NOTIFY_NONE = 0,
  NRA_NOTIFY_FUNC_AVAILABLE = 1,            // 功能可开启
  NRA_NOTIFY_RELEASE_BREAK = 2,             // 点击开始或继续，未松开刹车
  NRA_NOTIFY_PARKING_START = 3,             // 点击开始，松开刹车
  NRA_NOTIFY_SURROUNDING_AWARENESS = 4,     // 请注意周围环境
  NRA_NOTIFY_COMPLETE = 5,                  // 一键贴边已完成
  NRA_NOTIFY_VERSION_UNMATCHED = 6,         // 版本不匹配
  NRA_NOTIFY_CAMERA_FAULT = 7,              // 相机故障，倒车循迹不可用
  NRA_NOTIFY_RADAR_FAULT = 8,               // 雷达故障，倒车循迹不可用
  NRA_NOTIFY_SYSTEM_FAULT = 9,             // 系统故障，倒车循迹不可用
  NRA_NOTIFY_ASSOCIATED_SYSTEM_FAULT = 10,  // 关联系统故障，倒车循迹不可用
  NRA_NOTIFY_CAMERA_OBSTRUCTION = 11,       // 相机遮挡，倒车循迹不可用
  NRA_NOTIFY_VEH_NOT_READY = 12,            // 车辆未启动，倒车循迹不可用
  NRA_NOTIFY_VEH_DOOR_OPEN = 13,            // 请关闭车门
  NRA_NOTIFY_VEH_HATCH_OPEN = 14,           // 请关闭车前盖
  NRA_NOTIFY_VEH_TRUNK_OPEN = 15,           // 请关闭后备箱
  NRA_NOTIFY_GEAR_NOT_IN_FORWAR = 16,       // 请切换至D档
  NRA_NOTIFY_ROAD_LANE_TOO_NARROW = 17,     // 道路过窄
  NRA_NOTIFY_HIGH_SPEED = 18,               // 车速超限，倒车循迹不可用
  NRA_NOTIFY_VEH_ON_SLOPE = 19,             // 坡度超限，循迹倒车暂不可用
  NRA_NOTIFY_ADAS_ACTIVE = 20,              // 其他原因导致的建图不可用
  NRA_NOTIFY_TIRE_PRESSURE_ABNORMAL = 21, // 胎压异常，功能终止
  NRA_NOTIFY_ENVIRONMENT_ABNORMAL = 22,     // 其他原因导致的建图不可用
  NRA_NOTIFY_CHARGING = 23,                 // 充电中
  NRA_NOTIFY_SEATBELT_UNFASTENED = 24,      // 未系安全带
  NRA_NOTIFY_CLOSE_DOOR_REQ = 25,           // 请关闭车门
  NRA_NOTIFY_CLOSE_TRUNK_REQ = 26,          // 请关闭后备箱
  NRA_NOTIFY_CLOSE_HATCH_REQ = 27,          // 请关闭车前盖
  NRA_NOTIFY_FORWAR_GEAR_REQ = 28,          // 请挂D挡
  NRA_NOTIFY_RELEASE_ACC_PEDAL = 29,        // 车辆静止，踩油门
  NRA_NOTIFY_BRAKE_SUSPEND = 30,            // 踩刹车暂停
  NRA_NOTIFY_ACC_SUSPEND = 31,              // 干预油门暂停
  NRA_NOTIFY_OPEND_DOOR_SUSPEND = 32,       // 打开车门暂停
  NRA_NOTIFY_OPEND_TRUNK_SUSPEND = 33,      // 打开后备箱暂停
  NRA_NOTIFY_OPEND_HATCH_SUSPEND = 34,      // 打开前舱盖暂停
  NRA_NOTIFY_UNFASTENED_SEATBELT_SUSPEND = 35, // 解开安全带
  NRA_NOTIFY_WAIT_OBSTACLE_SUSPEND = 36,       // 障碍物阻挡暂停
  NRA_NOTIFY_RESUME_AVAILABLE = 37,            // 暂停条件解除，用户未点击继续
  NRA_NOTIFY_CONTINUE_PARKING = 38,            // 暂停条件解除，用户点击继续
  NRA_NOTIFY_CAMERA_FAULT_EXIT = 39,           // 相机故障，功能退出
  NRA_NOTIFY_RADAR_FAULT_EXIT = 40,            // 雷达故障，功能退出
  NRA_NOTIFY_SYSTEM_FAULT_EXIT = 41,           // 系统故障，功能退出
  NRA_NOTIFY_ASSOCIATED_SYSTEM_FAULT_EXIT = 42, // 关联系统故障，功能退出
  NRA_NOTIFY_CAMERA_OBSTRUCTION_EXIT = 43,    // 相机遮挡，功能退出 
  NRA_NOTIFY_HIGH_SPEED_EXIT = 44,            // 超速，功能终止
  NRA_NOTIFY_SPEED_ABNORMAL_EXIT = 45,        // 车速超调，功能终止
  NRA_NOTIFY_ESP_OFF_EXIT = 46,               // ESP关闭，功能终止
  NRA_NOTIFY_EPB_ACTIVE_EXIT = 47,            // EPB拉起，功能终止
  NRA_NOTIFY_TRAJECTORY_ABNORMAL_EXIT = 48,   // 轨迹超限，功能终止
  NRA_NOTIFY_SLOPE_ABNORMAL_EXIT = 49,        // 坡度超限，功能终止
  NRA_NOTIFY_ADAS_ACTIVE_EXIT = 50,           // 其他功能激活，功能终止
  NRA_NOTIFY_NUMBER_OF_INTERRUPTIONS_OVERLIMIT_EXIT = 51, // 中断次数超限，功能终止
  NRA_NOTIFY_INTERRUPTION_TIME_OUT_EXIT = 52, // 中断超时，功能终止
  NRA_NOTIFY_PARKING_TIME_OUT_EXIT = 53,      // 窄路通行超时，功能终止
  NRA_NOTIFY_STEP_TIMES_EXCEED_EXIT = 54,     // 超步，功能终止
  NRA_NOTIFY_DRIVER_INTERVENTION_EXIT = 55,   // 驾驶员干预,功能终止
  NRA_NOTIFY_SPACE_INSUFFICIENT_EXIT= 56,       // 空间不足，功能终止
  NRA_NOTIFY_TIRE_PRESSURE_ABNORMAL_EXIT = 57,  // 胎压异常，功能终止
  NRA_NOTIFY_CHARGING_AND_PARKING_EXIT = 58,    // 充电中，泊车退出
  NRA_NOTIFY_ENVIRONMENT_ABNORMAL_EXIT = 59,    // 环境异常，功能终止
  NRA_NOTIFY_OPEND_DOOR_SUSPEND_EXIT = 60,       // 打开车门暂停
  NRA_NOTIFY_OPEND_TRUNK_SUSPEND_EXIT = 61,      // 打开后备箱暂停
  NRA_NOTIFY_OPEND_HATCH_SUSPEND_EXIT = 62,      // 打开前舱盖暂停
  NRA_NOTIFY_BREAK_THEN_START = 63,              // 开始前，刹车未踩下
  NRA_NOTIFY_BREAK_RELEASED = 64,                // 系统未控车，车辆溜车
  NRA_NOTIFY_BRAKE_STOP_REQ = 65,                // 请刹停车辆
  NRA_NOTIFY_EPB_ACTIVE = 66,                    // 请释放EPB
} _ENUM_PACKED_ NraNotify;

typedef struct {
  NraStatus nra_status;         // PA状态机状态
  NraNotify nra_notify;         // 座舱交互提示
  boolean nra_active_resp;      // 是否激活
  boolean nra_active_denied;    // 是否激活失败
  float nra_distance;           // 已行驶距离 (米)
  FuncButtonState nra_active_button_sts; // 窄路通行按钮(预留)
  FuncButtonState nra_start_button_sts; // 窄路通行开始按钮
  FuncButtonState nra_continue_button_sts; // suspend情况下窄路通行继续按钮
} _STRUCT_ALIGNED_ HmiNraInfo;

typedef enum {
  HMI_SR_INVALID = 0,
  HMI_SR_PILOT = 1,       // 行车SR 
  HMI_SR_PARKING = 2,     // 泊车SR
} _ENUM_PACKED_ HmiSrInfo;

typedef enum {
  HMI_PARKING_NONE = 0,     // 无泊车声音 
  HMI_PARKING_VOICE_1 = 1,  // 泊车声音1 开始泊车
  HMI_PARKING_VOICE_2 = 2,  // 泊车声音2 泊车完成
  HMI_PARKING_VOICE_3 = 3,  // 泊车声音3 泊车激活异常
  HMI_PARKING_VOICE_4 = 4,  // 泊车声音4 搜车位/泊车准备异常
  HMI_PARKING_VOICE_5 = 5,   // 泊车声音5 搜车位异常退出
  HMI_PARKING_VOICE_6 = 6,  // 泊车声音6 
  HMI_PARKING_VOICE_7 = 7,  // 泊车声音7 
} _ENUM_PACKED_ HmiParkingVoice; //(E541)

typedef enum {
    DOOR_NONE = 0,        // No request
    DOOR_UNLCOK = 1,      // 解锁
    DOOR_LOCK = 2,        // 上锁
    DOOR_RESERVE = 3,     // 保留
} _ENUM_PACKED_ DoorLockReq;

typedef struct {
  boolean blue_light_state;  // 小蓝灯信号请求值
} _STRUCT_ALIGNED_ BlueLightSignalCommand;

typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  HmiCommon hmi_common;     // 通用信息(通用提醒、轨迹信息)
  uint8 hmi_line_topo_size;
  HmiLineInfo hmi_line_topo[HMI_TOPO_LINE_MAX_NUM];         // 可变车道线信息
  uint8 hmi_line_size;
  HmiLineInfo hmi_line[CAMERA_PERCEPTION_LANE_LINE_SET_LANE_MAX_NUM];         // 车道线
  uint8 hmi_stop_line_size;
  HmiStopLine hmi_stop_line[CAMERA_PERCEPTION_STOP_LANE_LINE_MAX_NUM];        // 停止线
  uint8 hmi_inhibit_line_size;
  HmiLineInfo hmi_inhibit_line[CAMERA_PERCEPTION_INHIBIT_LANE_LINE_MAX_NUM];  // 禁止线
  uint8 hmi_lane_ground_marking_size;
  HmiLaneGroundMarking hmi_lane_ground_marking[CAMERA_PERCEPTION_LANE_GROUND_MARKING_MAX_NUM];  // 车道地面标识
  uint8 hmi_lane_info_size;
  HmiLaneInfo hmi_lane_info[CAMERA_PERCEPTION_LANE_MAX_NUM];              // 道路信息
  uint8 hmi_obj_info_size;
  HmiObjInfo hmi_obj_info[FUSION_OBJECT_MAX_NUM];  // 障碍物信息
  int32 cipv_track_id;                          // CIPV ID ; -1 表示无
  int32 cutin_track_id;                         // cutin ID； -1 表示无
  HmiApaInfo hmi_apa_info;                      // 泊车信息
  HmiAccInfo hmi_acc_info;                      // acc信息
  HmiSccInfo hmi_scc_info;                      // scc信息
  HmiNoaInfo hmi_noa_info;                      // noa信息
  HmiAdasInfo hmi_adas_info;                    // adas信息
  HmiCalibInfo calib_info;                      // 标定信息
  uint8 sensor_info_size;
  HmiSensorInfo sensor_info[HMI_SENSOR_INFO_MAX_NUM]; // 传感器故障/遮挡信息
  HmiSwitchInfo hmi_switch_info;                      // 开关状态反馈
  HmiHppOutput hmi_hpp_output;                        // hpp信息
  EhpOutput ehp_output;                               // ehp信息
  RunningMode running_mode;                           // 运行模式 
  uint8 ground_lines_size;                                          // 视觉接地线数量
  GroundLine ground_lines[CAMERA_PERCEPTION_GROUND_LINES_MAX_NUM];  // 视觉接地线信息
  uint8 decelers_size;                                        // 视觉减速带数量
  Deceler decelers[CAMERA_PERCEPTION_DECELERS_MAX_NUM];       // 视觉减速带信息
  HmiRadsInfo hmi_rads_info;                                  // RADS信息
  HmiPaInfo hmi_pa_info;                                      // 一键贴边信息
  HmiNraInfo hmi_nra_info;                                    // 窄路通行信息
  HmiSrInfo hmi_sr_info;                                      // 当前行车/泊车的SR的标志位
  DoorLockReq door_lock_req;                                  // 车门锁请求
  TurnSignalCommand hmi_turn_signal_command;                  // 转向灯控制信号
  LightSignalCommand hmi_light_signal_command;                // 远近光灯控制请求
  BlueLightSignalCommand hmi_blue_light_signal_command;       // 小蓝灯控制请求
  HornSignalCommand hmi_horn_signal_command;                  // 喇叭控制请求
  RearViewMirrorCommand hmi_rear_view_mirror_signal_command;  // 后视镜控制请求
  uint8 hmi_free_space_map[HMI_FREE_SPACE_VOXEL_SIZE];        // 可形式区域栅格
  HmiParkingVoice hmi_parking_voice;                          // 泊车提示音请求
} _STRUCT_ALIGNED_ HmiSocOuter;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_HMI_SOC_OUTER_H_
