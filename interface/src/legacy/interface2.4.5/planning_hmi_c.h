// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_LEGACY_INTERFACE2_4_5_PLANNING_HMI_H_
#define _IFLYAUTO_LEGACY_INTERFACE2_4_5_PLANNING_HMI_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#ifdef __cplusplus
namespace iflyauto {
  namespace interface_2_4_5 {
#endif

#define AD2HMI_OBSTACLE_NUM 16
#define ALC_OUTPUT_INFO_NUM 64

#pragma pack(4)

//  LDW功能输出的结构体定义
typedef enum {
  LDW_FUNCTION_FSM_WORK_STATE_UNAVAILABLE = 0,                // Unavailable
  LDW_FUNCTION_FSM_WORK_STATE_OFF = 1,                        // Off
  LDW_FUNCTION_FSM_WORK_STATE_STANDBY = 2,                    // Standby
  LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_NO_INTERVENTION = 3,     // Active(No Intervention)
  LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION = 4,   // Active(Left Intervention)
  LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION = 5,  // Active(Right Intervention)
} _ENUM_PACKED_ LDWFunctionFSMWorkState;

typedef struct {
  LDWFunctionFSMWorkState ldw_state;  // LDW功能状态机状态
  boolean ldw_left_warning;           // LDW功能触发左侧报警标志位    (true:Left Warning / false:No Warning)
  boolean ldw_right_warning;          // LDW功能触发右侧报警标志位    (true:Rirht Warning / false:No Warning)
} _STRUCT_ALIGNED_ LDWOutputInfoStr;

//  LDP功能输出的结构体定义
typedef enum {
  LDP_FUNCTION_FSM_WORK_STATE_UNAVAILABLE = 0,                // Unavailable
  LDP_FUNCTION_FSM_WORK_STATE_OFF = 1,                        // Off
  LDP_FUNCTION_FSM_WORK_STATE_STANDBY = 2,                    // Standby
  LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_NO_INTERVENTION = 3,     // Active(No Intervention)
  LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION = 4,   // Active(Left Intervention)
  LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION = 5,  // Active(Right Intervention)
} _ENUM_PACKED_ LDPFunctionFSMWorkState;

typedef struct {
  LDPFunctionFSMWorkState ldp_state;  // LDP功能状态机状态
  boolean ldp_left_intervention_flag;  // LDP功能触发左侧报警标志位    (true:Left Intervention / false:No Intervention)
  boolean
      ldp_right_intervention_flag;  // LDP功能触发右侧报警标志位    (true:Rirht Intervention / false:No Intervention)
} _STRUCT_ALIGNED_ LDPOutputInfoStr;

// ELK功能输出结构体定义
typedef enum {
  ELK_FUNCTION_FSM_WORK_STATE_UNAVAILABLE = 0,                // Unavailable
  ELK_FUNCTION_FSM_WORK_STATE_OFF = 1,                        // Off
  ELK_FUNCTION_FSM_WORK_STATE_STANDBY = 2,                    // Standby
  ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_NO_INTERVENTION = 3,     // Active(No Intervention)
  ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION = 4,   // Active(Left Intervention)
  ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION = 5,  // Active(Right Intervention)
} _ENUM_PACKED_ ELKFunctionFSMWorkState;

typedef struct {
  ELKFunctionFSMWorkState elk_state;  // ELK功能状态机状态
  boolean elk_left_intervention_flag;  // ELK功能触发左侧报警标志位    (true:Left Intervention / false:No Intervention)
  boolean
      elk_right_intervention_flag;  // ELK功能触发右侧报警标志位    (true:Rirht Intervention / false:No Intervention)
} _STRUCT_ALIGNED_ ELKOutputInfoStr;

// TSR功能输出结构体定义
typedef enum {
  TSR_FUNCTION_FSM_WORK_STATE_UNAVAILABLE = 0,  // Unavailable
  TSR_FUNCTION_FSM_WORK_STATE_OFF = 1,          // Off
  TSR_FUNCTION_FSM_WORK_STATE_STANDBY = 2,      // Standby
  TSR_FUNCTION_FSM_WORK_STATE_ACTIVE = 3,       // Active
} _ENUM_PACKED_ TSRFunctionFSMWorkState;

typedef struct {
  TSRFunctionFSMWorkState tsr_state;  // TSR功能状态
  uint32 tsr_speed_limit;             // TSR识别到的限速标识牌    (公里/小时)
  boolean tsr_warning;                // TSR超速报警标志位 (true:Warning / false:No Warning)
} _STRUCT_ALIGNED_ TSROutputInfoStr;

// IHC功能输出结构体定义
typedef enum {
  IHC_FUNCTION_FSM_WORK_STATE_UNAVAILABLE = 0,  // Unavailable
  IHC_FUNCTION_FSM_WORK_STATE_OFF = 1,          // Off
  IHC_FUNCTION_FSM_WORK_STATE_STANDBY = 2,      // Standby
  IHC_FUNCTION_FSM_WORK_STATE_ACTIVE = 3,       // Active
} _ENUM_PACKED_ IHCFunctionFSMWorkState;

typedef struct {
  IHCFunctionFSMWorkState ihc_state;  // IHC功能状态
  boolean ihc_request_status;         // IHC请求状态  (true:Request / false:No Request)
  boolean ihc_request;                // IHC请求      (true:HighBeam / false:LowBeam)
} _STRUCT_ALIGNED_ IHCOutputInfoStr;

/*  ALC功能输出结构体定义
 *  发送给工控机的可视化工具，做换道可视化
 *  字段含义解释链接
 * http://wiki.iflytek.com/pages/viewpage.action?pageId=517586574
 *  字段稳定后，再改成具体的结构体
 */
typedef struct {
  uint8 lc_request_size;
  uint8 lc_request[ALC_OUTPUT_INFO_NUM];  // 换道请求
  uint8 lc_status_size;
  uint8 lc_status[ALC_OUTPUT_INFO_NUM];  // 换道状态
  uint8 lc_invalid_reason_size;
  uint8 lc_invalid_reason[ALC_OUTPUT_INFO_NUM];  // 无法换道原因
  uint8 lc_back_reason_size;
  uint8 lc_back_reason[ALC_OUTPUT_INFO_NUM];  // 换道返回原因
} _STRUCT_ALIGNED_ ALCOutputInfoStr;

typedef struct {
  boolean has_cipv;  // 是否存在cipv (true:yes / false:no)
  int32 cipv_id;     // cipv的id，对应融合障碍物id
                     // <FusionObjectsInfo.FusionObject.FusionAdditional.track_id>
} _STRUCT_ALIGNED_ CIPVInfoStr;

// Lane change direction
typedef enum {
  LC_LEFT = 0,   // Left
  LC_RIGHT = 1,  // Right
  LC_OTHER = 2,  // Other
} _ENUM_PACKED_ LaneChangeDirection;

// Lane change direction
typedef enum {
  RAMP_NONE = 0,
  RAMP_LEFT = 1,
  RAMP_RIGHT = 2,
} _ENUM_PACKED_ RampDirection;

//  Lane change status
typedef enum {
  LC_NO_CHANGE = 0,      // No change
  LC_START = 1,          // Start
  LC_WAITING = 2,        // Waiting
  LC_STARTING = 3,       // Starting
  LC_CANCELLED = 4,      // Cancelled
  LC_REJECTED = 5,       // Reject lane change
  LC_FAILED = 6,         // Failed
  LC_FAILED_MANUAL = 7,  // Failed,need handover
  LC_COMPLETED = 8,      // Completed
} _ENUM_PACKED_ LaneChangeStatus;

//  Lane change intent
typedef enum {
  NO_INTENT = 0,           // 无意图
  OUT_INTENT = 1,          // 下匝道
  IN_INTENT = 2,           // 汇入主路
  SLOWING_INTENT = 3,      // 前方车辆慢行
  FASTLANE_INTENT = 4,     // 进入快车道
  BLINKSWITCH_INTENT = 5,  // 用户主动打灯变道
} _ENUM_PACKED_ LaneChangeIntent;

typedef enum {
  LC_SOURCE_NONE = 0,
  LC_SOURCE_INT = 1,  // 打灯变道
  LC_SOURCE_ACT = 2,  // 主动变道
  LC_SOURCE_MAP = 3,  // 地图变道
} _ENUM_PACKED_ LaneChangeSource;

// Obstacle status
typedef enum {
  OBSTACLE_STATUS_NORMAL = 0,     // Normal
  OBSTACLE_STATUS_FOLLOWING = 1,  // Following
  OBSTACLE_STATUS_TOO_CLOSE = 2,  // Too close
  OBSTACLE_STATUS_DANGER = 3,     // Danger
  OBSTACLE_STATUS_OTHER = 4,      // Other
} _ENUM_PACKED_ ObstacleLonStatus;

// Avoid obstacle
typedef enum {
  AVOID_NO_HIDING = 0,       // No hiding
  AVOID_HIDING = 1,          // Hiding
  AVOID_OBSTACLE_OTHER = 2,  // Other
} _ENUM_PACKED_ AvoidObstacle;

// Landing point
typedef struct {
  Point3d relative_pos;
  float32 heading;
} _STRUCT_ALIGNED_ LandingPoint;

// Lccs trajectory polyline
typedef struct {
  float32 c0;
  float32 c1;
  float32 c2;
  float32 c3;
  float32 c4;
  float32 c5;
  float32 start_x;
  float32 end_x;
} _STRUCT_ALIGNED_ LCCS_TrajPoly;

// Trajectory point
typedef struct {
  LCCS_TrajPoly start;
  LCCS_TrajPoly middle;
  LCCS_TrajPoly end;
} _STRUCT_ALIGNED_ LCCSTrajectoryPoint;

typedef enum {
  FRONT_LEFT = 0,
  FRONT_MIDDLE = 1,
  FRONT_RIGHT = 2,
  RIGHT_MIDDLE = 3,
  RIGHT_BACK = 4,
  BACK_MIDDLE = 5,
  BACK_LEFT = 6,
  LEFT_MIDDLE = 7,
  MIDDLE = 8,
} _ENUM_PACKED_ AnchorPoint;

// Obstacle info
typedef struct {
  uint32 id;
  float32 speed_x;               // 车体坐标系下, 障碍物X方向速度
  float32 speed_y;               // 车体坐标系下, 障碍物Y方向速度
  float32 heading;               // 障碍物朝向
  ObjectType type;               // 障碍物类型: 骑车人, 卡车, 小车, 公交车, 锥桶
  float32 center_x;              // 车体坐标系下, 障碍物中心X坐标
  float32 center_y;              // 车体坐标系下, 障碍物中心Y坐标
  Shape3f size;                  // 障碍物大小
  ObstacleLonStatus lon_status;  // 障碍物状态
  AnchorPoint anchor;            // 参考点
} _STRUCT_ALIGNED_ ObstacleInfo;

typedef enum {
  NO_BRAKE = 0,
  BRAKE = 1,
} _ENUM_PACKED_ BrakeState;

typedef enum {
  AVOID_NONE = 0,
  AVOID_LEFT = 1,
  AVOID_RIGHT = 2,
} _ENUM_PACKED_ AvoidObstacleDirection;

typedef enum {
  PARKING_PAUSE_OTHER = 0,         // 泊车暂停原因：其他
  PARKING_PAUSE_FOR_OBSTACLE = 2,  // 泊车暂停原因：有障碍物
  PARKING_PAUSE_FOR_BREAK = 3,     // 泊车暂停原因：刹车
} _ENUM_PACKED_ APAPauseReason;

typedef enum {
  AVOID_OTHER = 0,       // 避让其他类型障碍物
  AVOID_PEDESTRIAN = 2,  // 避让行人
  AVOID_VEHICLE = 3,     // 避让车辆
} _ENUM_PACKED_ AvoidObstacleType;

typedef enum {
  EMERGRENCY_LEVEL_NORMAL = 0,  // 出现正常紧急情况，无法继续
  EMERGRENCY_LEVEL_HIGH = 1,    // 出现危险紧急情况，无法继续
} _ENUM_PACKED_ EmergencyLevel;

// Ad to Hmi data
typedef struct {
  uint64 timestamp;  // ms
  LaneChangeDirection lane_change_direction;
  LaneChangeStatus lane_change_status;
  LaneChangeIntent lane_change_intent;
  LaneChangeSource lane_change_source;
  uint32 noa_exit_warning_level_distance;  // 单位是米 驶出地图剩余距离
  AvoidObstacle avoid_status;              // 是否在避让状态
  boolean is_curva;                        // 弯道限速
  LandingPoint landing_point;              // 变道结束位置点
  LCCSTrajectoryPoint tp;                  // 暂无
  uint8 obstacle_info_size;
  ObstacleInfo obstacle_info[AD2HMI_OBSTACLE_NUM];
  uint32 cutin_track_id;
  float32 cruise_speed;                // 最高巡航速度，单位是km/h
  AvoidObstacleDirection avoiddirect;  // 躲避方向，当前正在躲避的障碍物(纵向有重叠且正在躲避)
  uint32 distance_to_ramp;             // 单位是米 离匝道的距离
  uint32 distance_to_split;            // 单位是米 离前方分流的距离
  uint32 distance_to_merge;            // 单位是米 离前方汇流的距离
  uint32 distance_to_toll_station;     // 单位是米 离收费站的距离
  uint32 distance_to_tunnel;           // 单位是米 离隧道的距离
  boolean is_within_hdmap;             // 能否启用NOA功能
  RampDirection ramp_direction;        // 匝道方向
} _STRUCT_ALIGNED_ AD2HMIData;

typedef struct {
  boolean is_avaliable;                   // 是否可进入hpp        (0:否/1:是)
  float32 distance_to_parking_space;      // 离泊车位的距离       (米)
  AvoidObstacle avoid_status;             // 避让状态
  AvoidObstacleType avoid_obstacle_type;  // 避让类型
  boolean is_approaching_turn;            // 是否即将进入弯道     (0:否/1:是)
  boolean is_approaching_intersection;    // 是否即将进入路口     (0:否/1:是)
  boolean is_approaching_speed_bumps;     // 是否存在减速带       (0:否/1:是)
  EmergencyLevel emergency_level;         // 紧急情况的等级
  boolean is_parking_space_occupied;      // 记忆泊车的停车位是否已被占用 (0:否/1:是)
  boolean is_new_parking_space_found;     // 是否发现新的可用车位         (0:否/1:是)
  boolean is_on_hpp_lane;                 // 是否在路线上          (0:否/1:是)
  boolean is_reached_hpp_trace_start;     // 是否到达记忆泊车起点   (0:否/1:是)
  float32 accumulated_driving_distance;   // 已行驶距离             (米)
} _STRUCT_ALIGNED_ HPPHMIData;

typedef struct {
  float32 distance_to_parking_space;    // 离泊入还剩的距离 (米)
  boolean is_parking_pause;             // 泊车是否暂停     (0:不暂停/1:暂停)
  APAPauseReason parking_pause_reason;  // 泊车暂停原因
} _STRUCT_ALIGNED_ APAHMIData;

// PlanningHMI模块输出结构体定义
typedef struct {
  Header header;
  LDWOutputInfoStr ldw_output_info;
  LDPOutputInfoStr ldp_output_info;
  ELKOutputInfoStr elk_output_info;
  TSROutputInfoStr tsr_output_info;
  IHCOutputInfoStr ihc_output_info;
  ALCOutputInfoStr alc_output_info;
  CIPVInfoStr cipv_info;  // cipv信息
  AD2HMIData ad_info;
  HPPHMIData hpp_info;
  APAHMIData apa_info;
} _STRUCT_ALIGNED_ PlanningHMIOutputInfoStr;

#pragma pack()
#ifdef __cplusplus
  }  // namespace interface_2_4_5
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_PLANNING_HMI_H_