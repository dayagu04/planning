// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_CONTROL_COMMAND_H_
#define _IFLYAUTO_CONTROL_COMMAND_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#ifdef __cplusplus
namespace iflyauto {
#endif

#define CONTROL_RESULT_POINTS_MAX_NUM 32

#pragma pack(4)

typedef enum {
  LatControlMode_LatControlType_LAT_NONE = 0,     // 横向不操控
  LatControlMode_LatControlType_STEER = 1,        // 转向
  LatControlMode_LatControlType_STEER_RATE = 2,   // 转速
  LatControlMode_LatControlType_STEER_TORQUE = 3  // 扭矩
} _ENUM_PACKED_ LatControlType;

typedef enum {
  LonControlMode_LonControlType_LON_NONE = 0,
  LonControlMode_LonControlType_THROTTLE = 1,
  LonControlMode_LonControlType_BRAKE = 2,
  LonControlMode_LonControlType_SPEED = 3,
  LonControlMode_LonControlType_ACCELERATION = 4,
  LonControlMode_LonControlType_AXLE_TORQUE = 5,
  LonControlMode_LonControlType_STOP_DISTANCE = 6
} _ENUM_PACKED_ LonControlType;

typedef enum {
  ControlStatus_ControlStatusType_IDLE = 0,   /* 未进自动 */
  ControlStatus_ControlStatusType_START = 1,  /* 起步状态 */
  ControlStatus_ControlStatusType_CRUISE = 2, /* 正常行驶状态 */
  ControlStatus_ControlStatusType_STOP = 3    /* 制动停车状态 */
} _ENUM_PACKED_ ControlStatusType;

typedef struct {
  uint8 control_result_points_size;
  Point3d control_result_points[CONTROL_RESULT_POINTS_MAX_NUM];  // 控制预测自车的轨迹点     <最大32个>
} _STRUCT_ALIGNED_ ControlTrajectory;

// 横向控制模式
typedef struct {
  boolean available;                // 横向控制信息有效性   (true:有效/false:无效)
  LatControlType lat_control_mode;  // 横向控制模式
} _STRUCT_ALIGNED_ LatControlMode;

// 纵向控制模式
typedef struct {
  boolean available;                // 纵向控制信息有效性   (true:有效/false:无效)
  LonControlType lon_control_mode;  // 纵向控制模式
} _STRUCT_ALIGNED_ LonControlMode;

// controller状态
typedef struct {
  boolean available;                      // 控制状态信息有效性   (true:有效/false:无效)
  ControlStatusType control_status_type;  // 控制状态
} _STRUCT_ALIGNED_ ControlStatus;

typedef enum {
  EPBCtrlCmdEnum_No_Request = 0,  // 无请求
  EPBCtrlCmdEnum_Released = 1,    // 请求释放
  EPBCtrlCmdEnum_Applied = 2,     // 请求夹起
} _ENUM_PACKED_ EPBCtrlCmdEnum;

typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
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
} _STRUCT_ALIGNED_ ControlOutput;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_CONTROL_COMMAND_H_