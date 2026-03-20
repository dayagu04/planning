// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2024/03/29

#ifndef _IFLYAUTO_ICC_C_H_
#define _IFLYAUTO_ICC_C_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"
#include "control_command_c.h"
#include "func_state_machine_c.h"
#include "degraded_driving_function_c.h"
#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)
typedef struct {
  MsgHeader msg_header;
  // MsgMeta msg_meta;
  FunctionalState current_state;                // 状态机当前状态 <需对照状态跳转图查询>
  uint32 state_duration;                        // 当前状态已经持续时间         (毫秒)
  FuncNotAvailableReason not_available_reason;  // 功能不可用原因
  PilotReq pilot_req;                           // 用户行车请求
  ParkingReq parking_req;                       // 用户泊车请求
  SwitchSts switch_sts;                         // 功能软开关状态
  RunningMode running_mode;                     // 运行模式         
  CalibModule calib_module;                     // 标定模块
  SystemState system_state;                     // 系统状态
  EHPIn ehp_req;                                // 地图管理请求
  // RadsMap rads_map;                             // RADS轨迹信息
} _STRUCT_ALIGNED_ FuncStateMachineMcu;         //功能状态机消息，由PNC转换后发给MCU的消息，仅在MCU上使用

typedef struct {
  uint64 IccTimeStamp;
} _STRUCT_ALIGNED_ TimedalyMeasure;

typedef struct {
  MsgHeader msg_header;
  TimedalyMeasure timedaly_measure;                      
  // MsgMeta msg_meta;
  LatControlMode lat_control_mode;                       // 横向控制模式
  float64 steering;                                      // 目标方向盘转角      (度deg)
  float64 steering_rate;                                 // 目标方向盘转角      (deg/s)
  float64 steering_torque;                               // 目标方向盘转矩      (Nm)
  LonControlMode lon_control_mode;                       // 纵向控制模式
  float64 throttle;                                      // 目标油门开度,        [0,100]
  float64 brake;                                         // 目标制动踏板强度     [0,100]
  float64 speed;                                         // 目标油门开度         [0,100]
  float64 acceleration;                                  // 目标加速度           (m/s^2)
  float64 axle_torque;                                   // 目标驱动力矩         (Nm)
  float64 stop_distance;                                 // 目标停车距离         (米)
  GearCommandValue gear_command_value;                   // 目标挡位
  TurnSignalType turn_signal_cmd;                        // 转向灯
  LightSignalType light_signal_cmd;                      // 灯光请求
  HornSignalType horn_signal_cmd;                        // 喇叭请求
  RearViewMirrorSignalType rear_view_mirror_signal_cmd;  // 后视镜请求
  ControlTrajectory control_trajectory;                  // 控制预测自车的轨迹
  ControlStatus control_status;                          // controller状态
  boolean standstill_request;
  boolean driver_off_request;
  boolean axle_torque_request;   // 扭矩+减速度接口使用
  boolean acceleration_request;  // 扭矩+减速度接口使用/加速度接口使用
  float64 acc_cmd;               // 加速度接口使用
  EPBCtrlCmdEnum epb_cmd;
  float64 steering_torque_max;  // 目标方向盘转矩最大值      (Nm)
  float64 steering_torque_min;  // 目标方向盘转矩最大值      (Nm)
  boolean amap_request_flag;    // 0:no request  1:request
  float64 amap_trq_limit_max;   // 单位 : Nm
  uint32 meb_request_status;
  float64 meb_request_value;  // 单位:m/ss
} _STRUCT_ALIGNED_ ControlOutputOnMcu;      //控制输出消息，由PNC转换后发给MCU的消息，仅在MCU上使用

typedef struct { 
  MsgHeader msg_header;
  // MsgMeta msg_meta;
  FunctionStatus acc;
  FunctionStatus lcc;
  FunctionStatus hnoa;
  FunctionStatus mnoa;
  FunctionStatus aeb;
  FunctionStatus ldw;
  FunctionStatus ldp;
  FunctionStatus elk;
  FunctionStatus tsr;
  FunctionStatus isli;
  FunctionStatus ihc;
  FunctionStatus rpa;
  FunctionStatus apa;
  FunctionStatus rads;
  FunctionStatus meb;
  FunctionStatus hpp;
  FunctionStatus mrm;
  FunctionStatus pa;
  FunctionStatus nra;
  FunctionStatus mapping;
  FunctionStatus fcta_fctb;
  FunctionStatus rcta_rctb;
  FunctionStatus rcw;
  FunctionStatus dow;
  FunctionStatus bsd;
  FunctionStatus amap;
  FunctionStatus dai;
} _STRUCT_ALIGNED_ DegradedDrivingFunctionMcu_E541;   //E541功能降级消息，由PNC转换后发给MCU的消息，仅在MCU上使用

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_ICC_C_H_