// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2024/03/29

#ifndef _IFLYAUTO_FSM_H_
#define _IFLYAUTO_FSM_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"
#include "planning_hmi_c.h"
#include "planning_plan_c.h"
#include "hmi_inner_c.h"
#include "ifly_parking_map_c.h"

#define RADS_MAP_POINT_MAX_NUM 110 // RADS记录的最大点数

#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)

typedef enum {
  // manual
  FunctionalState_MANUAL_DRIVING = 0,
  FunctionalState_MANUAL_PARKING = 1,
  // mrc
  FunctionalState_MRC = 2,
  // 行车抑制
  FunctionalState_DRIVING_PASSIVE = 3,
  /* acc */
  FunctionalState_ACC_STANDBY = 4,
  FunctionalState_ACC_ACTIVATE = 5,
  FunctionalState_ACC_OVERRIDE = 6,
  /* scc */
  FunctionalState_SCC_STANDBY = 7,
  FunctionalState_SCC_ACTIVATE = 8,
  FunctionalState_SCC_OVERRIDE = 9,
  /* noa */
  FunctionalState_NOA_STANDBY = 10,
  FunctionalState_NOA_ACTIVATE = 11,
  FunctionalState_NOA_OVERRIDE = 12,
  /* park */
  FunctionalState_PARK_STANDBY = 13,
  FunctionalState_PARK_IN_SEARCHING = 14,
  FunctionalState_PARK_GUIDANCE = 15,
  FunctionalState_PARK_SUSPEND = 16,
  FunctionalState_PARK_COMPLETED = 17,
  FunctionalState_PARK_OUT_SEARCHING = 18,
  FunctionalState_PARK_PRE_ACTIVE = 19,
  // 行/泊车ERROR
  FunctionalState_DRIVING_ERROR = 20,
  FunctionalState_PARK_ERROR = 21,
  /* nra */
  FunctionalState_NRA_PASSIVE = 22,
  FunctionalState_NRA_STANDBY = 23,
  FunctionalState_NRA_PRE_ACTIVE = 24,
  FunctionalState_NRA_SUSPEND = 25,
  FunctionalState_NRA_GUIDANCE = 26,
  FunctionalState_NRA_COMPLETED = 27,
  FunctionalState_NRA_ERROR = 28,
  // hpp
  
  FunctionalState_HPP_STANDBY = 50,
  FunctionalState_HPP_PROACTIVE_MAPPING = 51,
  FunctionalState_HPP_BACKGROUND_MAPPING = 52,
  FunctionalState_HPP_MAP_CHECK = 53,
  FunctionalState_HPP_PRE_ACTIVE_DRIVING = 54,
  FunctionalState_HPP_PRE_ACTIVE_PARKING = 55,
  FunctionalState_HPP_CRUISE_ROUTING = 56,
  FunctionalState_HPP_CRUISE_SEARCHING = 57,
  FunctionalState_HPP_PARKING_OUT = 58,
  FunctionalState_HPP_PARKING_IN = 59,
  FunctionalState_HPP_SUSPEND = 60,
  FunctionalState_HPP_COMPLETE = 61,
  FunctionalState_HPP_ABORT = 62,
  FunctionalState_HPP_ERROR = 63,
  // RADS
  FunctionalState_RADS_PASSIVE = 70,
  FunctionalState_RADS_STANDBY = 71,
  FunctionalState_RADS_PRE_ACTIVE = 72,
  FunctionalState_RADS_TRACING = 73,
  FunctionalState_RADS_SUSPEND = 74,
  FunctionalState_RADS_COMPLETE = 75,
  FunctionalState_RADS_ABORT = 76,
  FunctionalState_RADS_ERROR = 77,

  // CALIBRATION
  FunctionalState_CUSTOMER_CALIBRATION = 80,

  // system error
  FunctionalState_SYSTEM_ERROR = 81,

  // car show mode
  FunctionalState_SHOW_CAR = 82,
} _ENUM_PACKED_ FunctionalState;

typedef enum {
  BackgroundState_HPP_ERROR = 11,
  BackgroundState_HPP_PASSIVE = 12,
  BackgroundState_HPP_STANDBY = 13,
  BackgroundState_HPP_PROACTIVE_MAPPING = 14,
  BackgroundState_HPP_PROACTIVE_MAPPING_PAUSE = 15,
  BackgroundState_HPP_BACKGROUND_MAPPING = 16,
  BackgroundState_HPP_BACKGROUND_MAPPING_PAUSE = 17,
  BackgroundState_HPP_STORE = 18,
  BackgroundState_HPP_STORE_COMPLETE = 19,
  BackgroundState_HPP_STORE_FAIL = 20,
} _ENUM_PACKED_ BackgroundState;

// 功能不可用（退出/抑制）原因
typedef enum {
    REASON_NONE = 0,                        
    REASON_MAIN_SWITCH_OFF = 1,                    // 主开关未开
    REASON_SEAT_BELT_NOT_FASTENED = 2,             // 安全带未系
    REASON_BRAKE_WAS_PRESSED = 3,                  // 刹车被踩
    REASON_DOOR_NOT_CLOSE = 4,                     // 车门未关
    REASON_PARK_TIME_OUT= 5,                       // 泊车超时
    REASON_BAD_WEATHER = 6,                        // 天气恶劣
    REASON_TURN_LIGHT_ON = 7,                      // 转向灯打开
    REASON_GEAR_NOT_IN_FORWARD = 8,                // 挡位不在前进挡
    REASON_EPB_ON = 9,                             // EPB打开
    REASON_AEB_INTERVENTION = 10,                   // AEB使能
    REASON_ESP_ACTIVATE = 11,                       // ESP使能
    REASON_ABS_ACTIVATE = 12,                       // ABS使能
    REASON_TCS_ACTIVATE = 13,                       // TCS使能
    REASON_HIGH_SPEED = 14,                         // 速度过快
    REASON_STANDWAIT_TIME_OUT = 15,                 // standwait超时
    REASON_OVERRIDE_TIME_OUT = 16,                  // override超时
    REASON_VEHICLE_REVERSE = 17,                    // 车辆后溜
    REASON_PLANNING_REQUEST = 18,                   // 规控请求
    REASON_MAP_NAVIGATION_NOT_AVAILABLE = 19,       // 地图导航不可用
    REASON_LONG_CONTROL_FAULT = 20,                 // 纵向控制故障
    REASON_LAT_CONTROL_FAULT = 21,                  // 横向控制故障
    REASON_HANDS_OFF = 22,                          // 脱手
    REASON_STEERING_WHEEL_ANGLE_EXCESSIVE = 23,     // 方向盘转角过大
    REASON_LATERAL_TAKEOVER = 24,                   // 横向接管
    REASON_ACCELERATE_OVERRIDE = 25,                // 纵向超控
    REASON_LANELINE_BLURRED = 26,                   // 车道线不清晰
    REASON_VEHICLE_LANE_DEVIATION_EXCESSIVE = 27,   // 车辆与车道中心线偏差过大
    REASON_ROAD_CURVATURE_EXCESSIVE = 28,           // 道路曲率弯度过大
    REASON_ROAD_WIDTH_NARROW = 29,                  // 道路宽度过窄
    REASON_VEHICLE_FAULT = 30,                      // 严重车辆故障
    REASON_VEHICLE_NOT_READY = 31,                  // 车辆未READY
    REASON_VEHICLE_LANE_DISTANCE_EXCESSIVE = 32,    // 车辆与道路中心距离过大
    REASON_ROAD_WIDTH_WIDE = 33,                    // 道路宽度过宽
    REASON_SLOW_SPEED = 34,                         // 车速过慢
    REASON_NEAR_TO_TOLL_STATION = 35,               // 距离收费站过近
    REASON_LANELINE_LENGTH_SHORT = 36,              // 道路长度过短
    REASON_NEAR_TO_DESTINATION = 37,                // 距离目的地过近
    REASON_EXCESSIVE_SLOPE = 38,                    // 道路坡度过大    
    REASON_LOCATION_UNAVAILABLE = 39,               // 定位失效       
    REASON_PATH_PLANNING_FAILED = 40,               // 路径规划失败
    REASON_GEAR_MISMATCH = 41,                      // 档位错误
    REASON_MAPPING_ERROR = 42,                      // 建图错误
    REASON_MAPPING_DIS_TOO_LONG = 43,               // 建图距离过长
    REASON_DEVIATION_ROUTE = 44,                    // 偏离路线
    REASON_OBSTACLE_INHIBIT = 45,                   // 障碍物抑制
    REASON_OPERATION_TIMEOUT = 46,                  // 操作超时
    REASON_EPB_NOT_ON = 47,                         // 未打开EPB
    REASON_GEAR_NOT_IN_P = 48,                      // 未挂P档
    REASON_GEAR_NOT_IN_R = 49,                      // 未挂R档
    REASON_NOT_STANDSTILL = 50,                     // 车辆未静止
    REASON_ACCELERATOR_PEDAL = 51,                  // 干预加速踏板
    REASON_PARK_SLOT_UNAVAILABLE = 52,              // 无有效车位或车位未选
    REASON_GPS_LOSS_POINT_UNAVAILABLE = 53,         // 消星点不可用
    REASON_SLOPE_ANOMALY = 54,                      // 坡度异常
    REASON_MAPPING_STATUS_ANOMALY = 55,             // 建图状态异常
    REASON_NOT_ON_HPP_LANE = 56,                    // HPP不在记忆路线
    REASON_NOT_IN_PARKING_SLOT = 57,                // 不在车位内
    REASON_POWER_MODE_UNAVAILABLE = 58,             // 电源模式不可用
    REASON_BRAKE_NOT_PRESSED = 59,                  // 未踩刹车踏板
    REASON_SLOT_NOT_SELECTED = 60,                  // 未选择车位
    REASON_SLOT_IS_OCCUPIED = 61,                   // 目标车位被占用
    REASON_CHARGING = 62,                           // 正在充电中
    REASON_REVERSE_EXCEED = 63,                     // 倒车距离过远
    REASON_NOT_BACK_GROUND_MAPPING_ROAD = 64,       // 不符合静默建图道路
    REASON_STEPPED_ON_ACCELERATOR = 65,             // 深踩油门    
    REASON_VDC_NOT_ACTIVATE = 66,                   // VDC未激活
    REASON_PLANNING_INHIBIT_LCC_ACTIVATE = 67,      // 规划判断LCC不可进入           
    REASON_FCW_ACTIVATE = 68,                       // FCW已激活
    REASON_TIMESTAMP_ABNORMAL = 69,                 // 数据时间戳异常                        
    REASON_NO_DATA_INPUT = 70,                      // 无数据输入
    REASON_TIRE_PRESSURE_ABNORMAL = 71,             // 胎压异常
    REASON_RCTB_ACTIVATE = 72,                      // RCTB已激活
    REASON_FCTB_ACTIVATE = 73,                      // FCTB已激活
    REASON_ADAS_ACTIVATE = 74,                      // 其他的ADAS功能已激活
    REASON_TOO_MANY_GEAR_SHIFTS = 75,               // 换挡次数过多
    REASON_PRK_OUT_DIR_INVALID = 76,                // 泊出方向无效
    REASON_HAND_TORQUE_OVER_LIMIT = 77,             // 手力矩超限制
    REASON_CAMERA_BLOCKED = 78,                     // 相机被遮挡
    REASON_PARKING_TRAJECTORY_ABNORMAL = 79,        // 泊车轨迹异常
    REASON_TOO_MANY_PARKING_SUSPEND = 80,           // 泊车中断次数过多
    REASON_ESP_DISBALE = 81,                        // ESP关闭
    REASON_COLLISION_OCCURRED = 82,                 // 发生碰撞
    REASON_CDP_ACTIVATE = 83,                       // CDP激活
    REASON_LONG_ACCEL_LARGE = 84,                   // 纵向舒适性不满足
    REASON_EXTREME_POWER_SAVER = 85,                // 驾驶模式受限
    REASON_CONDITION_NOT_SATISFIED = 86,            // 其他ODD条件不满足
    REASON_HAZARD_LIGHT_ON = 87,                    // 双闪打开
    REASON_LDP_ACTIVATE = 88,                       // LDP激活
    REASON_ELK_ACTIVATE = 89,                       // ELK激活
    REASON_AREA_CONDITION_NOT_SATISFIED = 90,       // 导航开启，但自车不在高架/高速场景
    REASON_HANDS_OFF_PUNISHMENT  = 91,              // 脱手惩罚
    REASON_BRAKE_DISC_OVERHEAT_TIMEOUT = 92,        // 制动盘过热超时
    REASON_LAT_ACTUATOR_FAULT = 93,                 // 横向执行器故障
    REASON_LONG_ACTUATOR_FAULT = 94,                // 纵向执行器故障
    REASON_LAT_SYSTEM_FAULT = 95,                  // 横向系统故障
    REASON_LONG_SYSTEM_FAULT = 96,                 // 纵向系统故障
    REASON_FATAL_VEHICLE_FAULT = 97,               // 需要进行安全停车的故障
    REASON_ACC_BRAKEONLY_PASSIVE = 98,              // ACC brake only 退出
    REASON_RADSTRAJECTORY_INVALID = 99,              // RADS轨迹无效
    REASON_RADS_VEHICLE_MODE_UNMET = 100,            // RADS整车模式不满足
    REASON_CAMERA_ABNORMAL = 101,                     // 摄像头异常
    REASON_RADS_TRA_PLANNING_FAILED = 102,            // RADS轨迹规划失败
} _ENUM_PACKED_ FuncNotAvailableReason;

// 标定模块
typedef enum {
    CALIB_MODULE_NONE = 0,
    CALIB_MODULE_SVC_MODULE = 1,       // 环视
    CALIB_MODULE_IMU_MODULE = 2,       // IMU
    CALIB_MODULE_LIDAR_MODULE = 3,     // LIDAR
    CALIB_MODULE_SIDEVIEW_MODULE = 4,  // 周视
} _ENUM_PACKED_ CalibModule;

// PA刹停
typedef enum {
  APA_STOP_NO_REQUEST =  0,      // 无请求
  APA_STOP_COMFORTABLE = 1,      // 舒适刹停
  APA_STOP_EMERGE = 2,           // 紧急刹停
} _ENUM_PACKED_ APAStopReq;

// 软开关状态
typedef struct {
  boolean acc_main_switch;                     // acc软开关        (true:on / false:off)
  boolean scc_main_switch;                     // scc软开关        (true:on / false:off)
  boolean noa_main_switch;                     // noa软开关        (true:on / false:off)
  
  boolean fcw_main_switch;                     // fcw软开关        (true:on / false:off)
  SensitivityLevel fcw_set_sensitivity_level;  // fcw敏感度等级
  boolean aeb_main_switch;                     // aeb软开关        (true:on / false:off)
  boolean meb_main_switch;                     // meb软开关        (true:on / false:off)
  NotificationMainSwitch tsr_main_switch;      // tsr软开关
  boolean ihc_main_switch;                     // ihc软开关        (true:on / false:off)
  boolean ldw_main_switch;                     // ldw软开关        (true:on / false:off)
  SensitivityLevel ldw_set_sensitivity_level;  // ldw敏感度等级
  boolean elk_main_switch;                     // elk软开关        (true:on / false:off)
  boolean bsd_main_switch;                     // bsd软开关        (true:on / false:off)
  boolean lca_main_switch;                     // lca软开关        (true:on / false:off)
  boolean dow_main_switch;                     // dow软开关        (true:on / false:off)
  boolean fcta_main_switch;                    // fcta软开关
  boolean fctb_main_switch;                    // fctb软开关
  boolean rcta_main_switch;                    // rcta软开关
  boolean rctb_main_switch;                    // rctb软开关
  boolean rcw_main_switch;                     // rcw软开关        (true:on / false:off)
  boolean ldp_main_switch;                     // ldp软开关        (true:on / false:off)
  boolean apa_main_switch;                     // apa软开关        (true:on / false:off)
  boolean hpp_main_switch;                     // hpp软开关        (true:on / false:off)
  boolean rads_main_switch;                    // rads软开关       (true:on / false:off)
  boolean amap_main_switch;                    // amap软开关       (true:on / false:off)
  boolean dai_main_switch;                     // dai软开关        (true:on / false:off)
  boolean dow_secondary_alert_main_switch;     // dow 二级警示语音软开关  (true:on / false:off)
  boolean blue_light_main_switch;              // 小蓝灯软开关      (true:on / false:off)
} _STRUCT_ALIGNED_ SwitchSts;

// 用户泊车请求
typedef struct {
  APAStopReq apa_stop;                          // apa刹停请求
  ApaUserPreference apa_user_preference;        // 泊车用户偏好 
  ApaParkOutDirection apa_park_out_direction;   // 用户选择泊出方向
  uint32 parking_select_slotid;                 // 用户选中泊入车位
  ApaWorkMode apa_work_mode;                    // 泊车模式
  ApaParkingDirection apa_parking_direction;    // 用户选择泊入方向
  ApaFreeSlotInfo apa_free_slot_info;           // 自定义泊车
  uint32 local_map_id;                          // 地图中的目标车位id对应的建图id
  PaDirection pa_direction;                     // 用户选择的贴边方向
  ParkingSpeedSet parking_speed_set;            // 泊车速度设置
  RearViewMirrorCommand rear_view_mirror_signal_command; //规划折叠指令
} _STRUCT_ALIGNED_ ParkingReq;

// 用户泊车请求
typedef struct {
  uint8 need_gnss_loss_signal;                        // 是否需要消星，用于研测可视化模拟消星点触发 0:默认值 1:不需要 2:需要 
} _STRUCT_ALIGNED_ HPPReq;

// 用户行车请求
typedef struct {
  LaneChangeStatus lane_change_sts;          // 变道信息
  PilotUserPreference pilot_usr_preference;  // 行车用户偏好 
  float32 acc_curise_real_spd;               // acc巡航车速（实际车速 m/s）
  float32 acc_curise_time_interval;          // acc设定时距
  LaneChangeStyle lane_change_style;         // 变道风格设置
  boolean noa_cruise_dclc_resp;              // 变道确认提醒开关(false: 关闭， true: 打开)
  boolean is_overtake_lane_change_confirmed; // 超车变道确认结果汇总(true:确认变道, false:未确认变道)
  boolean traffic_light_stop_go;             // 直行红绿灯启停设置(false: 关闭， true: 打开)
  boolean obstacle_bypass;                   // 借道绕行设置(false: 关闭， true: 打开)
  boolean stand_wait;                        // 是否是stand_wait状态
  TurnSwitchStateEnum turn_switch;           // 转向开关状态
  boolean has_obstacle_ahead;                // 前方是否存在障碍物（如大型车辆/行人）
} _STRUCT_ALIGNED_ PilotReq;

// 系统运行模式
typedef enum  {
    RUNNING_MODE_HIGHWAY = 0,              // 行车
    RUNNING_MODE_PARKING = 1,              // 泊车
    RUNNING_MODE_CALIBRATION = 2,          // 标定
    RUNNING_MODE_MANUAL_DRIVING = 3,       // 手动驾驶
    RUNNING_MODE_ACE = 4,            
    RUNNING_MODE_MEMORY_HIGHWAY = 5,       // 记忆行车
    RUNNING_MODE_MEMORY_PARKING = 6,       // 记忆泊车          
    RUNNING_MODE_REVERSE_FOLLOW_TRACE = 7, // 循迹倒车  
    RUNNING_MODE_PA = 8,                   // 一键贴边
    RUNNING_MODE_NRA = 9,                  //窄路通行   
} _ENUM_PACKED_ RunningMode;

typedef enum {
    SYSTEM_STATE_NO_FAULT = 0,    // 无故障
    SYSTEM_STATE_ALERT = 1,       // 系统报警
    SYSTEM_STATE_PARKING_RESTRICTED = 2, // 泊车受限
    SYSTEM_STATE_DRIVING_RESTRICTED = 3, // 行车受限
    SYSTEM_STATE_RESTRICTED = 4,  // 系统受限
    SYSTEM_STATE_RESERVED = 5, // 保留
} _ENUM_PACKED_ SystemState;

typedef struct {
    uint16 points_size; // 有效点数，点间隔是 1m ,待确认
    Coordinate points[RADS_MAP_POINT_MAX_NUM];  // 点的坐标,使用开机系boot
    float length;                         // 长度       (米)
    uint8 buffer[PROTO_STR_LEN];          // 保留字段，用于提高序列化/反序列化的速度
} _STRUCT_ALIGNED_ RadsMap;

typedef struct {
   float nra_distance; //nra行驶距离
} _STRUCT_ALIGNED_ NraReq;

typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  FunctionalState current_state;                // 状态机当前状态 <需对照状态跳转图查询>
  BackgroundState background_state;             // 状态机后台状态
  uint32 state_duration;                        // 当前状态已经持续时间         (毫秒)
  FuncNotAvailableReason not_available_reason;  // 功能不可用原因
  PilotReq pilot_req;                           // 用户行车请求
  ParkingReq parking_req;                       // 用户泊车请求
  HPPReq hpp_req;                               // 用户HPP请求
  SwitchSts switch_sts;                         // 功能软开关状态
  RunningMode running_mode;                     // 运行模式         
  CalibModule calib_module;                     // 标定模块
  SystemState system_state;                     // 系统状态
  EHPIn ehp_req;                                // 地图管理请求
  RadsMap rads_map;                             // RADS轨迹信息
  NraReq nra_req;                               //NRA信息
} _STRUCT_ALIGNED_ FuncStateMachine;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_FSM_H_
