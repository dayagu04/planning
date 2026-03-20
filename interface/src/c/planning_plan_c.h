// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_PLANNING_PLAN_
#define _IFLYAUTO_PLANNING_PLAN_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#define PLANNING_REFERENCE_POLY_NUM 4
#define PLANNING_TRAJ_POINTS_MAX_NUM 201
#define PLANNING_PARKING_SLOT_MAX_NUM 16

#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)

// Planning模块记录的标签信息
typedef struct {
  uint8 plan_strategy_name[PROTO_STR_LEN];  // 规划策略说明 <最大64个字符>
} _STRUCT_ALIGNED_ PlanMeta;

// 规划期望加速度取值范围
typedef struct {
  /** 规划期望加速度下限值。
   *  单位：米/秒^2   (m/s^2)
   *  备注：正值：加速；负值：减速
   **/
  float64 min_a;

  /** 规划期望加速度上限值。
   *  单位：米/秒^2   (m/s^2)
   *  备注：正值：加速；负值：减速
   **/
  float64 max_a;
} _STRUCT_ALIGNED_ AccelerationRange;

// 实时规划跟踪的目标参考值内容
typedef struct {
  /** 规划轨迹多项式参数。
   *  备注：数组深度固定为4。[0]：三次项系数；[1]：二次项系数；[2]：一次项系数；[3]：常数项系数
   **/
  float64 polynomial[PLANNING_REFERENCE_POLY_NUM];

  /** 规划输出的目标车速。
   *  备注：正值：前进；负值：后退
   **/
  float64 target_velocity;
  AccelerationRange acceleration_range_limit;  // 规划期望加速度取值范围
  LateralManeuverGear lateral_maneuver_gear;   // 规划期望方向盘转角挡位
} _STRUCT_ALIGNED_ TargetReference;

// 长时规划各轨迹点信息
typedef struct {
  float64 x;  // 规划轨迹点在绝对坐标系下的x坐标。    (米)
  float64 y;  // 规划轨迹点在绝对坐标系下的y坐标。    (米)

  /** 规划轨迹点在enu坐标系下的自车朝向。
   *  单位：弧度      (rad)
   *  备注：朝x轴正向绕z轴逆时针旋转为正（0~Π），顺时针旋转为负（0~-Π）
   **/
  float64 heading_yaw;
  float64 curvature;  // 规划轨迹点的曲率

  /** 从规划起始点到当前点的相对时间。
   *  单位：秒
   *  备注：取值范围[0.0~5.0s]
   **/
  float64 t;

  /** 该规划点处目标车速。
   *  单位：米/秒
   *  备注：正值：前进；负值：后退
   **/
  float64 v;

  /** 该规划点处纵向目标加速度。
   *  单位：米/秒^2
   *  备注：正值：加速；负值：减速
   **/
  float64 a;
  float64 distance;  // 从规划起点到当前点的轨迹长度。       (米)
  float64 jerk;      // 该规划点处的纵向jerk                (米/秒^3)
} _STRUCT_ALIGNED_ TrajectoryPoint;

typedef struct {
  // 规划轨迹有效性。 (true：有效(轨迹求解正常且轨迹信息校验正常) /
  // false：无效(轨迹求解失败、轨迹信息校验失败等))
  boolean available;
  TrajectoryType trajectory_type;    // 输出规划轨迹类型
  TargetReference target_reference;  // 实时规划跟踪的目标参考值内容
  uint8 trajectory_points_size;                                     // 长时规划各轨迹点数量
  TrajectoryPoint trajectory_points[PLANNING_TRAJ_POINTS_MAX_NUM];  // 长时规划各轨迹点信息
} _STRUCT_ALIGNED_ Trajectory;

// 规划转向灯请求
typedef struct {
  boolean available;                 // 转向灯请求有效性。       (true:有效/false:无效)
  TurnSignalType turn_signal_value;  // 转向灯请求值
} _STRUCT_ALIGNED_ TurnSignalCommand;

// 规划远近灯光请求
typedef struct {
  boolean available;                   // 灯光信号请求有效性。     (true:有效/false:无效)
  LightSignalType light_signal_value;  // 灯光信号请求值
} _STRUCT_ALIGNED_ LightSignalCommand;

// 规划喇叭请求
typedef struct {
  boolean available;                 // 喇叭请求有效性。         (true:有效/false:无效)
  HornSignalType horn_signal_value;  // 喇叭请求值
} _STRUCT_ALIGNED_ HornSignalCommand;

// 规划挡位请求
typedef struct {
  boolean available;                    // 挡位请求有效性。         (true:有效/false:无效)
  GearCommandValue gear_command_value;  // 挡位请求值
} _STRUCT_ALIGNED_ GearCommand;

// 规划后视镜请求
typedef struct {
  boolean available;                                // 挡位请求有效性。   (true:有效/false:无效)
  RearViewMirrorSignalType rear_view_mirror_value;  // 后视镜折叠请求值
} _STRUCT_ALIGNED_ RearViewMirrorCommand;

// 规划开环打方向请求
typedef struct {
  boolean available;                       // 开环打方向请求有效性。       (true:有效/false:无效)
  float64 jerk_factor;                     // 方向盘转角变化率限值。       (rad/s^3)
  boolean need_steering_wheel_stationary;  // 是否需要自车静止打方向。     (true:需要/false:不需要)
  float64 steering_wheel_rad_limit;        // 最小方向盘期望转角。         (弧度rad)
} _STRUCT_ALIGNED_ OpenLoopSteeringCommand;

typedef enum {
  APA_NONE = 0,
  APA_IN_PROGRESS = 1,
  APA_FINISHED = 2,
  APA_FAILED = 3,
} _ENUM_PACKED_ ApaPlanningStatus;

typedef enum {
  HPP_UNKNOWN = 0,         // 未知规划状态
  HPP_RUNNING = 1,         // 规划状态正常
  HPP_ROUTING_COMPLETED = 2,       // 巡航阶段完成
  HPP_ROUTING_PLANNING_FAILED = 3, // 巡航阶段规划失败
  HPP_PARKING_COMPLETED = 4,       // 泊车阶段完成
  HPP_PARKING_PLANNING_FAILED = 5 // 泊车阶段规划失败
} _ENUM_PACKED_ HppPlanningStatus;

typedef enum {
  RADS_UNKNOWN = 0,         // 未知规划状态
  RADS_RUNNING = 1,         // 规划状态正常
  RADS_COMPLETED = 2,       // rads场景规划完成
  RADS_RUNNING_FAILED = 3,  // 规划失败
} _ENUM_PACKED_ RadsPlanningStatus;

typedef enum {
  NSA_UNKNOWN = 0,         // 未知规划状态
  NSA_RUNNING = 1,         // 规划状态正常
  NSA_COMPLETED = 2,       // NSA场景规划完成
  NSA_RUNNING_FAILED = 3,  // 规划失败
} _ENUM_PACKED_ NSAPlanningStatus;

typedef enum {
  NOT_FAIL = 0,                    // 规划成功
  NO_TARGET_POSE = 1,              // 没有目标终点失败
  PATH_PLAN_FAIL = 2,              // 路径规划失败
  FOLD_MIRROR_FAILED = 3,          // 后视镜折叠之后未成功完成泊车失败
  STUCK_FAILED_TIME = 4,           // 卡住时间过长失败
  GEAR_CHANGE_COUNT_TOO_MUCH = 5,  // 挡位变化次数过多失败
} _ENUM_PACKED_ ApaPlanningFailedReason;

typedef struct _ApaPlanningGearChangeStatus {
  boolean has_request_continue_parking;  // 是否要继续泊车请求
  uint8 remaining_gear_change_count;     // 剩余换挡次数
} _STRUCT_ALIGNED_ ApaPlanningGearChangeStatus;

typedef struct _PlanningStatus {
  boolean standstill;           // 车辆是否静止     (true:车辆静止/false:车辆非静止)
  boolean ready_to_go;          // 车辆是否要起步   (true:车辆将要起步/false:车辆非将要起步)
  ApaPlanningStatus apa_planning_status;    // 泊车在规划中的状态
  HppPlanningStatus hpp_planning_status;    // 记忆泊车在规划中的状态
  RadsPlanningStatus rads_planning_status;  // 循迹倒车在规划中的状态
  NSAPlanningStatus nsa_planning_status;    // 窄路通行功能在规划中的状态
  ApaPlanningFailedReason apa_planning_failed_reason;  // 泊车规划失败原因
  ApaPlanningGearChangeStatus apa_planning_gear_change_status;  // 泊车挡位变化状态
} _STRUCT_ALIGNED_ PlanningStatus;

typedef enum {
  REQUEST_LEVEL_NO_REQ = 0,  // 无退自动请求
  REQUEST_LEVEL_MILD = 1,    // 请系统退自动，接管
  REQUEST_LEVEL_MIDDLE = 2,  // 请系统尽快退自动，接管[默认值：2s]
  REQUEST_LEVEL_URGENT = 3,  // 请系统立刻退自动，接管
} _ENUM_PACKED_ RequestLevel;
typedef enum {
  REQUEST_REASON_NO_REASON = 0,
  REQUEST_REASON_VEHICLE_STATE = 1,  // 四门两盖、安全带
  REQUEST_REASON_SINGAL_LOSS = 2,    // 上游信号缺少
  REQUEST_REASON_COMPUTE_FAIL = 3,   // 求解失败
  REQUEST_REASON_ON_INTERSECTION_LEFT_LANE = 4,   //在路口左转车道
  REQUEST_REASON_ON_INTERSECTION_RIGHT_LANE = 5,   //在路口右转车道
  REQUEST_REASON_ON_INTERSECTION_OTHER = 6,   //在路口其它原因无法通过
  REQUEST_REASON_ON_ROUNDABOUT = 7,   //环岛无法通过
  REQUEST_REASON_BORROW_FAILED = 8       //绕行失败接管提示
} _ENUM_PACKED_ RequestReason;

typedef struct {
  RequestLevel take_over_req_level;  // 请求类型(NONE:无请求)
  RequestReason request_reason;      // 请求原因
} _STRUCT_ALIGNED_ PlanningRequest;

// 规划成功的车位信息
typedef struct {
  uint32 id;  // 车位id
  boolean is_narrow_slot;  // 窄车位标志
} _STRUCT_ALIGNED_ SuccessfulSlotsInfo;

// Planning模块输出信息
typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  PlanMeta meta;
  Trajectory trajectory;
  TurnSignalCommand turn_signal_command;
  LightSignalCommand light_signal_command;
  HornSignalCommand horn_signal_command;
  GearCommand gear_command;
  RearViewMirrorCommand rear_view_mirror_signal_command;
  OpenLoopSteeringCommand open_loop_steering_command;
  PlanningStatus planning_status;
  uint8 successful_slot_info_list_size;                                         // 规划成功的车位数量
  SuccessfulSlotsInfo successful_slot_info_list[PLANNING_PARKING_SLOT_MAX_NUM]; // 规划成功的车位信息
  PlanningRequest planning_request;                                             // 向系统提出接管请求
} _STRUCT_ALIGNED_ PlanningOutput;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_PLANNING_PLAN_
