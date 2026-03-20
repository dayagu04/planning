// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_VEHICLE_SERVICE_H_
#define _IFLYAUTO_VEHICLE_SERVICE_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)

typedef enum {
  WheelSpeed_Forward = 0,
  WheelSpeed_Back = 1,
} _ENUM_PACKED_ WheelSpeedDirEnum;  // vehicle_speed_direction/fl_wheel_speed_direction枚举值

typedef enum {
  EPBState_FullyReleased = 0,
  EPBState_FullyApplied = 1,
  EPBState_Releasing = 2,
  EPBState_Applying = 3,
  EPBState_Fault = 4,
} _ENUM_PACKED_ EPBStateEnum;  // epb_state枚举值

typedef enum {
  ShiftLeverState_P = 0,
  ShiftLeverState_R = 1,
  ShiftLeverState_N = 2,
  ShiftLeverState_D = 3,
  ShiftLeverState_M = 4,
} _ENUM_PACKED_ ShiftLeverStateEnum;  // shift_lever_state枚举值

typedef enum {
  TurnSwitchState_Off = 0,
  TurnSwitchState_Left_Turn = 1,
  TurnSwitchState_Right_Turn = 2,
  TurnSwitchState_Left_Change = 3,
  TurnSwitchState_Right_Change = 4,
} _ENUM_PACKED_ TurnSwitchStateEnum;  // turn_switch_state枚举值

typedef enum {
  WiperState_Off = 0,
  WiperState_LowSpeed = 1,
  WiperState_HighSpeed = 2,
} _ENUM_PACKED_ WiperStateEnum;  // wiper_state枚举值

typedef enum {
  ActuatorStatus_NotReady = 0,
  ActuatorStatus_Ready = 1,
  ActuatorStatus_Active = 2,
  ActuatorStatus_TemporaryFailed = 3,
  ActuatorStatus_PermanentlyFailed = 4,
} _ENUM_PACKED_ ActuatorStatusEnum;  // 执行器状态枚举值

typedef enum {
  POWER_MODE_LOW_OFF = 0,         // 低压电源关闭
  POWER_MODE_LOW_VOLTAGE_LOW = 1, // 低压电压低
  POWER_MODE_LOW_ON = 2,          // 低压电源打开
} _ENUM_PACKED_ PowerMode_Low;  // 低压电源状态

typedef enum {
  No_Crash = 0,    // 无安全气囊打开
  Belt_Crash = 1,  // 安全带收紧
  Front_Crash = 2, // 前气囊打开
  Left_Crash = 3,  // 左气囊打开
  Right_Crash = 4, // 右气囊打开
  Rear_Crash = 5,  // 后气囊打开
  Roll_Crash = 6,  // 顶部气囊打开
} _ENUM_PACKED_ CrashOutSts;  // 安全气囊状态

typedef enum {
  Not_Overrange = 0,           // 无超限
  Angle_Overrange = 1,         // 角度超限
  AngleSpd_Overrange = 2,      // 角速度超限
  Torque_Overrange = 3,        // 力矩超限
  EPS_Overrange_reserved = 4,  // 预留位
} _ENUM_PACKED_ EPSOverRange;  // EPS超限状态

typedef enum {
  Arming_Sts_Not_Active = 0, // 未激活
  Arming_successful = 1,     // 功能激活成功
  DisArming_successful = 2,  // 哨兵模式触发成功
  Arming_Sts_Invalid = 3,    // 哨兵模式不可用
} _ENUM_PACKED_ ArmingSts;   // 哨兵模式

typedef enum {
  Rearview_Mirror_Invalid = 0,         // 信号无效
  Rearview_Mirror_Fold = 1,            // 后视镜折叠
  Rearview_Mirror_Unfold = 2,          // 后视镜未折叠
} _ENUM_PACKED_ RearviewMirrorSts;  // 后视镜状态

typedef enum {
  DriverGazeRegion_Driver_Windscreen = 0,         // 驾驶员侧挡风玻璃
  DriverGazeRegion_Passenger_Windscreen = 1,      // 副驾侧挡风玻璃
  DriverGazeRegion_Left_Rear_Mirror = 2,          // 左后视镜
  DriverGazeRegion_Right_Rear_Mirror = 3,         // 右后视镜
  DriverGazeRegion_Left_Side_Window = 4,         //左侧车窗
  DriverGazeRegion_Right_Side_Window = 5,         //右侧车窗
  DriverGazeRegion_Central_Rear_Mirror = 6,      //中间后视镜
  DriverGazeRegion_Central_driver_Instrument = 7, //中央仪表盘
  DriverGazeRegion_Other_Region = 8,             //其他区域
} _ENUM_PACKED_ DriverGazeRegion;

typedef enum {
  DmsStatus_DMS_Iint = 0,         //DMS初始状态
  DmsStatus_DMS_Standby = 1,      //DMS待命状态
  DmsStatus_DMS_Active = 2,       //DMS激活状态
  DmsStatus_DMS_Fault = 3,        //DMS故障状态
  DmsStatus_DMS_Camera_Block = 4, //DMS摄像头被遮挡
} _ENUM_PACKED_ DmsStatus;

typedef enum {
  HODAreaStatus_NO_Touch = 0,   //没有握
  HODAreaStatus_Touch = 1,      //触碰
  HODAreaStatus_Grab = 2,       //握住
} _ENUM_PACKED_ HODAreaStatus;

typedef enum {
  VS_DRIVE_MODE_NONE = 0,        // 无输入
  VS_DRIVE_MODE_NORMAL = 1,      // 正常模式
  VS_DRIVE_MODE_COMFORTABLE = 2, // 舒适模式
  VS_DRIVE_MODE_SNOW = 3,        // 雪地模式
  VS_DRIVE_MODE_MUD = 4,         // 泥泞模式
  VS_DRIVE_MODE_OFFROAD = 5,     // 越野模式
  VS_DRIVE_MODE_SAND = 6,        // 沙地模式
  VS_DRIVE_MODE_SPORT = 7,       // 运动模式
  VS_DRIVE_MODE_ECO = 8,         //节能模式
  VS_DRIVE_MODE_INDIVIDUAL = 9,    //自定义模式 
  VS_DRIVE_MODE_EHV = 10,    //节能混动模式   
  VS_DRIVE_MODE_WADE = 11,    //涉水模式
  VS_DRIVE_MODE_ROCK = 12,    //岩石模式 
  VS_DRIVE_MODE_AI = 13,    //AI模式 
  VS_DRIVE_MODE_SMART = 14,    //智能模式 
  VS_DRIVE_MODE_DEEPMUD = 15,    //泥泞模式 
  VS_DRIVE_MODE_DEEPSNOW = 16,    //深雪模式                    
} _ENUM_PACKED_ VsDriveMode; 

typedef enum { 
    VS_CAR_MODE_NORMAL_MODE = 0,     // 普通模式
    VS_CAR_MODE_FACTORY_MODE = 1,    // 工厂模式    
    VS_CAR_MODE_TRANSPORT_MODE = 2,  // 运输模式
    VS_CAR_MODE_TRAILER_MODE = 3, // 拖车模式
    VS_CAR_MODE_SHOWCAR_MODE = 4, // 展车模式
    VS_CAR_MODE_SHORTREST_MODE = 5, // 小憩模式
    VS_CAR_MODE_PET_MODE = 6, // 宠物模式
    VS_CAR_MODE_EES_MODE = 7,  // 极致省电模式
    VS_CAR_MODE_WIPER_REPAIR_MODE = 8, // 雨刮维修模式
    VS_CAR_MODE_CAMPING_MODE = 9, // 露营模式
    VS_CAR_MODE_WASH_CAR_MODE = 10,  // 洗车模式
    VS_CAR_MODE_DYNO_MODE = 11, //动力模式
    VS_CAR_MODE_CRASH_MODE = 12, //碰撞模式
    VS_CAR_MODE_FACTORY_PAUSED = 13, //工厂暂停
    VS_CAR_MODE_TRANSPORT_PAUSED = 14, //运输暂停
} _ENUM_PACKED_ VsCarMode;

typedef enum {
  ANTI_THEFT_ST_IDLE = 0,   //空闲
  ANTI_THEFT_ST_ENGAGEMENT = 1, //预定
  ANTI_THEFT_ST_SURVEILLANCE = 2,  //监控
  ANTI_THEFT_ST_ALARM = 3, //报警
} _ENUM_PACKED_ AntiTheftSt;

typedef enum {
  DRIVER_TIRED_INACTIVE = 0, //未激活
  DRIVER_TIRED_LEVEL1 = 1,  //等级1
  DRIVER_TIRED_LEVEL2 = 2,  //等级2
  DRIVER_TIRED_LEVEL3 = 3, //等级3
} _ENUM_PACKED_ DriverTired;

typedef struct {
  MsgHeader msg_header;                        // SOC消息发送信息
  SensorMeta sensor_meta;                      // CAN报文到达MCU信息
  float32 vehicle_speed;                       // 本车实际车速 单位:m/s
  boolean vehicle_speed_available;             // 本车实际车速有效性 (true:有效/false:无效)
  float32 vehicle_speed_display;               // 本车仪表车速 单位:m/s
  boolean vehicle_speed_display_available;     // 本车仪表车速有效性 (true:有效/false:无效)
  uint8 vehicle_speed_direction;               // 本车车速方向 (0:前进/1:后退)
  boolean vehicle_speed_direction_available;   // 本车车速方向有效性 (true:有效/false:无效)
  float32 fl_wheel_speed;                      // 前左轮速 单位:m/s
  boolean fl_wheel_speed_available;            // 前左轮速有效性 (true:有效/false:无效)
  float32 fr_wheel_speed;                      // 前右轮速 单位:m/s
  boolean fr_wheel_speed_available;            // 前右轮速有效性 (true:有效/false:无效)
  float32 rl_wheel_speed;                      // 后左轮速 单位:m/s
  boolean rl_wheel_speed_available;            // 后左轮速有效性 (true:有效/false:无效)
  float32 rr_wheel_speed;                      // 后右轮速 单位:m/s
  boolean rr_wheel_speed_available;            // 后右轮速有效性 (true:有效/false:无效)
  uint32 fl_wheel_speed_pulse;                 // 前左轮速脉冲
  boolean fl_wheel_speed_pulse_available;      // 前左轮速脉冲有效性 (true:有效/false:无效)
  uint32 fr_wheel_speed_pulse;                 // 前右轮速脉冲
  boolean fr_wheel_speed_pulse_available;      // 前右轮速脉冲有效性 (true:有效/false:无效)
  uint32 rl_wheel_speed_pulse;                 // 后左轮速脉冲
  boolean rl_wheel_speed_pulse_available;      // 后左轮速脉冲有效性 (true:有效/false:无效)
  uint32 rr_wheel_speed_pulse;                 // 后右轮速脉冲
  boolean rr_wheel_speed_pulse_available;      // 后右轮速脉冲有效性 (true:有效/false:无效)
  uint8 fl_wheel_speed_direction;              // 前左轮速方向 (0:前进/1:后退)
  boolean fl_wheel_speed_direction_available;  // 前左轮速方向有效性 (true:有效/false:无效)
  uint8 fr_wheel_speed_direction;              // 前右轮速方向 (0:前进/1:后退)
  boolean fr_wheel_speed_direction_available;  // 前右轮速方向有效性 (true:有效/false:无效)
  uint8 rl_wheel_speed_direction;              // 后左轮速方向 (0:前进/1:后退)
  boolean rl_wheel_speed_direction_available;  // 后左轮速方向有效性 (true:有效/false:无效)
  uint8 rr_wheel_speed_direction;              // 后右轮速方向 (0:前进/1:后退)
  boolean rr_wheel_speed_direction_available;  // 后右轮速方向有效性 (true:有效/false:无效)
  float32 steering_wheel_angle;  // 本车方向盘转角 单位:rad 左转弯为正，右转弯为负；方向盘转角原始信号，未做零位校正
  boolean steering_wheel_angle_available;        // 本车方向盘转角有效性 (true:有效/false:无效)
  float32 steering_wheel_angle_speed;            // 本车方向盘转速 单位:rad/s 左转弯为正，右转弯为负
  boolean steering_wheel_angle_speed_available;  // 本车方向盘转速有效性 (true:有效/false:无效)
  float32 yaw_rate;                              // 本车横摆角速度 单位:rad/s 左转弯为正，右转弯为负
  boolean yaw_rate_available;                    // 本车横摆角速度有效性 (true:有效/false:无效)
  float32 long_acceleration;                     // 本车纵向加速度 单位:m/ss
                              // 平路上，加速为正，减速为负；传感器原始信号，未去除道路坡度产生的影响
  boolean long_acceleration_available;  // 本车纵向加速度有效性 (true:有效/false:无效)
  float32 lat_acceleration;             // 本车横向加速度 单位:m/ss
                             // 平路上，左转为正，右转为负；传感器原始信号，未去除道路坡度产生的影响
  boolean lat_acceleration_available;       // 本车横向加速度有效性 (true:有效/false:无效)
  float32 accelerator_pedal_pos;            // 实际加速踏板开度百分比 范围:[0-100]
  boolean accelerator_pedal_pos_available;  // 实际加速踏板开度有效性 (true:有效/false:无效)
  float32 brake_pedal_pos;            // 实际制动踏板开度百分比 范围:[0-100] 不是所有车均有此信号
  boolean brake_pedal_pos_available;  // 实际制动踏板开度有效性 (true:有效/false:无效)
  boolean brake_pedal_pressed;        // 制动踏板是否被驾驶员踩下状态 (true:被踩下/false:未踩下)
  boolean brake_pedal_pressed_available;  // 制动踏板是否被驾驶员踩下状态有效性 (true:有效/false:无效)
  uint8 epb_state;                        // EPB系统状态
  boolean epb_state_available;            // EPB系统状态有效性        (true:有效/false:无效)
  uint8 auto_hold_state;                  // 自动驻车系统状态         (0:Not Active/1:Active)
  boolean auto_hold_state_available;      // 自动驻车系统状态有效性 (true:有效/false:无效)
  uint32 shift_lever_state;               // 驾驶员操作的换档杆状态
  boolean shift_lever_state_available;  // 驾驶员操作的换档杆状态有效性 (true:有效/false:无效)
  uint32 gear_lever_state;              // 变速箱实际挡位 <数值表示档位数值，0表示Invalid>
  boolean gear_lever_state_available;   // 变速箱实际挡位有效性 (true:有效/false:无效)
  uint8 driver_gear_intervention;               // 驾驶员挡位干预信号
  boolean driver_gear_intervention_available;  // 驾驶员挡位干预信号有效位 (true:有效/false:无效)
  uint8 turn_switch_state;              // 转向开关状态 (0: OFF 1: Left_turn 2: Right_turn 3:Left_Change 4: Right_Change)
  boolean turn_switch_state_available;  // 转向开关状态有效性 (true:有效/false:无效)
  boolean left_turn_light_state;  // 左转向灯状态 true:有效/false:无效 已对转向灯原始信号周期性跳变做过滤波处理
  boolean left_turn_light_state_available;  // 左转向灯状态有效性 (true:有效/false:无效)
  boolean right_turn_light_state;  // 右转向灯状态 true:有效/false:无效 已对转向灯原始信号周期性跳变做过滤波处理
  boolean right_turn_light_state_available;  // 右转向灯状态有效性 (true:有效/false:无效)
  boolean hazard_light_state;  // 双闪状态 true:有效/false:无效 已对转向灯原始信号周期性跳变做过滤波处理
  boolean hazard_light_state_available;        // 双闪状态有效性 (true:有效/false:无效)
  float32 driver_hand_torque;                  // 驾驶员手力矩 单位:Nm 左转弯为正，右转弯为负
  boolean driver_hand_torque_available;        // 驾驶员手力矩有效性 (true:有效/false:无效)
  boolean driver_hands_off_state;              // 驾驶员脱手状态 (true:HandsOff/false:HandsOn)
  boolean driver_hands_off_state_available;    // 驾驶员脱手状态有效性 (true:有效/false:无效)
  boolean door_lock_state;                     // 门锁状态 (false:unlock  true:lock)
  boolean door_lock_state_available;           // 门锁状态有效性(true:有效/false:无效)
  boolean fl_door_state;                       // 前左车门状态 (true:On/false:Off)
  boolean fl_door_state_available;             // 前左车门状态有效性 (true:有效/false:无效)
  boolean fr_door_state;                       // 前右车门状态 (true:On/false:Off)
  boolean fr_door_state_available;             // 前右车门状态有效性 (true:有效/false:无效)
  boolean rl_door_state;                       // 后左车门状态 (true:On/false:Off)
  boolean rl_door_state_available;             // 后左车门状态有效性 (true:有效/false:无效)
  boolean rr_door_state;                       // 后右车门状态 (true:On/false:Off)
  boolean rr_door_state_available;             // 后右车门状态有效性 (true:有效/false:无效)
  boolean hood_state;                          // 前舱盖状态 (true:On/false:Off)
  boolean hood_state_available;                // 前舱盖状态有效性 (true:有效/false:无效)
  boolean trunk_door_state;                    // 后备箱状态 (true:On/false:Off)
  boolean trunk_door_state_available;          // 后备箱状态有效性 (true:有效/false:无效)
  boolean fl_seat_belt_state;                  // 前左安全带状态 (true:On/false:Off)
  boolean fl_seat_belt_state_available;        // 前左安全带状态有效性 (true:有效/false:无效)
  boolean fr_seat_belt_state;                  // 前右安全带状态 (true:On/false:Off)
  boolean fr_seat_belt_state_available;        // 前右安全带状态有效性 (true:有效/false:无效)
  boolean rl_seat_belt_state;                  // 后左安全带状态 (true:On/false:Off)
  boolean rl_seat_belt_state_available;        // 后左安全带状态有效性 (true:有效/false:无效)
  boolean rm_seat_belt_state;                  // 后中安全带状态 (true:On/false:Off)
  boolean rm_seat_belt_state_available;        // 后中安全带状态有效性 (true:有效/false:无效)
  boolean rr_seat_belt_state;                  // 后右安全带状态 (true:On/false:Off)
  boolean rr_seat_belt_state_available;        // 后右安全带状态有效性 (true:有效/false:无效)
  uint32 wiper_state;                          // 雨刮状态
  boolean wiper_state_available;               // 雨刮状态有效性 (true:有效/false:无效)
  boolean front_fog_light_state;               // 前雾灯状态 (true:On/false:Off)
  boolean front_fog_light_state_available;     // 前雾灯状态有效性 (true:有效/false:无效)
  boolean rear_fog_light_state;                // 后雾灯状态 (true:On/false:Off)
  boolean rear_fog_light_state_available;      // 后雾灯状态有效性 (true:有效/false:无效)
  boolean auto_light_state;                    // 自动灯光控制状态 (true:On/false:Off)
  boolean auto_light_state_available;          // 自动灯光控制状态有效性 (true:有效/false:无效)
  boolean low_beam_state;                      // 近光灯状态 (true:On/false:Off)
  boolean low_beam_state_available;            // 近光灯状态有效性 (true:有效/false:无效)
  boolean high_beam_state;                     // 远光灯状态 (true:On/false:Off)
  boolean high_beam_state_available;           // 远光灯状态有效性 (true:有效/false:无效)
  float32 left_solar_sensor_value;             // 左光照传感器值 单位:W/m^2)
  boolean left_solar_sensor_value_available;   // 左光照传感器值有效性 (true:有效/false:无效)
  float32 right_solar_sensor_value;            // 右光照传感器值 单位:W/m^2)
  boolean right_solar_sensor_value_available;  // 右光照传感器值有效性 (true:有效/false:无效)
  float32 up_solar_sensor_value;            // 上光照传感器值 单位:W/m^2)
  boolean up_solar_sensor_value_available;  // 上光照传感器值有效性 (true:有效/false:无效)
  boolean esp_active;                          // 车身稳定性控制功能激活状态 (true:Active/false:Inactive)
  boolean esp_active_available;        // 车身稳定性控制功能激活状态有效性 (true:有效/false:无效)
  boolean abs_active;                  // 车轮防抱死控制功能激活状态 (true:Active/false:Inactive)
  boolean abs_active_available;        // 车轮防抱死控制功能激活状态有效性 (true:有效/false:无效)
  boolean tcs_active;                  // 牵引力控制功能激活状态 (true:Active/false:Inactive)
  boolean tcs_active_available;        // 牵引力控制功能激活状态有效性 (true:有效/false:无效)
  float32 power_train_current_torque;  // 动力系统当前输出扭矩 单位:Nm
  boolean power_train_current_torque_available;  // 动力系统当前输出扭矩有效性 (true:有效/false:无效)
  float32 power_train_allow_torque_max;          // 动力系统当前允许请求的最大扭矩 单位:Nm
  boolean power_train_allow_torque_max_available;  // 动力系统当前允许请求的最大扭矩有效性 (true:有效/false:无效)
  float32 power_train_allow_torque_min;  // 动力系统当前允许请求的最小扭矩 单位:Nm
  boolean power_train_allow_torque_min_available;  // 动力系统当前允许请求的最小扭矩有效性 (true:有效/false:无效)
  boolean power_train_override_flag;            // 动力系统超控标志位 (true:Override/false:Not Override)
  boolean power_train_override_flag_available;  // 动力系统超控标志位有效性 (true:有效/false:无效)
  uint32 pilot_long_control_actuator_status;    // 行车纵向控制执行器状态
  boolean pilot_long_control_actuator_status_available;  // 行车纵向控制执行器状态有效性 (true:有效/false:无效)
  uint32 pilot_lat_control_actuator_status;              // 行车横向控制执行器状态
  boolean pilot_lat_control_actuator_status_available;  // 行车横向控制执行器状态有效性 (true:有效/false:无效)
  uint32 aeb_actuator_status;                           // AEB执行器状态
  boolean aeb_actuator_status_available;                // AEB执行器状态有效性 (true:有效/false:无效)
  uint32 parking_long_control_actuator_status;          // 泊车纵向控制执行器状态
  boolean parking_long_control_actuator_status_available;  // 泊车纵向控制执行器状态有效性 (true:有效/false:无效)
  uint32 parking_lat_control_actuator_status;              // 泊车横向控制执行器状态
  boolean parking_lat_control_actuator_status_available;  // 泊车横向控制执行器状态有效性 (true:有效/false:无效)
  float32 total_odometer;                                 // 总里程 单位:km
  float32 ext_temperature;                                // 外部温度 单位:℃
  boolean ext_temperature_available;                      // 外部温度有效性 (true:有效/false:无效)
  float32 steering_wheel_angle_bias;                      // 仅限给control用 方向盘零偏
                                      // 直行时方向盘转角为angle,则steering_wheel_angle_bias=-angle
  float32 yaw_rate_offset;      // 仅限给control用 车辆静止状态下,取10秒yaw_rate平均值
  PowerMode_Low powermode_low;  // 低压状态
  CrashOutSts crash_out_sts;    // 安全气囊状态
  boolean eps_drv_override;     // EPS反馈的override状态 (true:Active/false:Inactive)
  boolean eps_drv_override_available;     // EPS反馈的override有效性 (true:有效/false:无效)
  EPSOverRange eps_overrange;             // EPS超限状态
  boolean eps_overrange_available;        // EPS超限状态有效性(true:有效/false:无效)
  boolean esp_vdc_active;      // vdc激活状态(true:Active/false:Inactive)
  boolean esp_vdc_fault;       // vdc故障状态(true:fault/false:no fault)
  boolean esp_ebd_fault;       // ebd故障状态(true:Active/false:Inactive)
  boolean veh_standstill;      // 车辆静止状态(true:Active/false:Inactive)
  boolean veh_standstill_available;      // 车辆静止有效性 (true:有效/false:无效)
  float32 esp_pressure;        // 主缸压力 单位bar
  boolean esp_pressure_available;      // 主缸压力有效性 (true:有效/false:无效)
  ArmingSts armingsts;        // 哨兵模式
  boolean vcu_ready;          // 车辆ready状态(true:Active/false:Inactive)
  boolean vcu_ready_available;      // 车辆ready状态有效性 (true:有效/false:无效)
  uint8   bms_soc ;                 // 车辆soc 【0-100】
  boolean charge_sts;               // 充电状态(true:充电中/false:未充电)
  float32 tire_pressureFL;          // 左前胎压 单位bar
  float32 tire_pressureFR ;         // 右前胎压 单位bar
  float32 tire_pressureRL;          // 左后胎压 单位bar
  float32 tire_pressureRR;          // 右后胎压 单位bar
  RearviewMirrorSts rearview_mirror_sts; // 后视镜状态(0:invalid/1:fold/2:unfold)
  float32 front_motor_speed; //前电机转速 单位rpm
  boolean front_motor_speed_available; //前电机转速有效性 (true:有效/false:无效)
  uint8   front_motor_speed_direction; //前电机转速方向（0:前进/1:后退）
  boolean front_motor_speed_direction_available;//前电机转速方向有效性 (true:有效/false:无效)
  float32 rear_motor_speed; //后电机转速 单位rpm
  boolean rear_motor_speed_available; //后电机转速有效性 (true:有效/false:无效)
  uint8   rear_motor_speed_direction; //后电机转速方向（0:前进/1:后退）
  boolean rear_motor_speed_direction_available; //后电机转速方向有效性(true:有效/false:无效)
  HODAreaStatus left_area_hod_sts; //HOD左区域驾驶员手握状态
  boolean left_area_hod_sts_available; //HOD左区域驾驶员手握状态有效位(true:有效/false:无效)
  HODAreaStatus right_area_hod_sts; //HOD右区域驾驶员手握状态
  boolean right_area_hod_sts_available; //HOD右区域驾驶员手握状态有效位(true:有效/false:无效)
  boolean hod_sys_sts; //HOD系统故障状态(true:发生故障/false:无故障)
  float32 incar_temp; //车内温度 单位℃
  uint16 cooling_sys_sts; //冷却系统状态
  float32 inlet_temp; //域控入口水温 单位 ℃
  DmsStatus dms_sts; //dms检测状态
  boolean driver_abnormal_behavior; //驾驶员非正常行为(true:发生非正常行为/false:未发生非正常行为)
  DriverGazeRegion driver_gaze_region; //驾驶员凝视区域
  boolean driver_eye_on_road_sts; //(true:观察路面/false:未观察路面)
  boolean drive_present_sts; //驾驶员是否存在(true:存在/false:不存在)
  boolean distraction_sts; //分心提示 （true:分心/false:未分心）
  uint8 vehicle_sts; //整车状态 用于判断产线/拖车等状态
  uint16 time_year; //TBOX时间-年
  uint8 time_month; //TBOX时间-月
  uint8 time_day; //TBOX时间-日
  uint8 time_hour; //TBOX时间-时
  uint8 time_minute; //TBOX时间-分
  uint8 time_second; //TBOX时间-秒
  float32 dew_point_temp; //露点温度 单位℃
  float32 glass_temp; //玻璃温度 单位℃
  boolean heating_wire_temp; //前档加热丝工作状态(true:Active/false:Inactive)
  VsCarMode vs_car_mode;//车辆模式
  VsDriveMode vs_drive_mode;//驾驶模式
  uint8 driver_drowsiness_state;//驾驶员疲劳状态
  boolean driver_drowsiness_state_available;//驾驶员疲劳状态有效位(true:有效/false:无效)
  uint8 dms_mouth_open; //驾驶员嘴部状态
  boolean dms_mouth_open_available; //驾驶员嘴部状态有效位(true:有效/false:无效)
  uint8 dms_eye_open; //驾驶员眼部状态
  boolean dms_eye_open_available; //驾驶员眼部状态有效位(true:有效/false:无效)
  uint8 dms_eye_occlusion; //驾驶员眼部遮挡状态
  boolean dms_eye_occlusion_available; //驾驶员眼部遮挡状态有效位(true:有效/false:无效)
  float32 vertical_acceleration; //垂直加速度 （单位m/ss）
  boolean vertical_acceleration_available; //垂直加速度有效位(true:有效/false:无效)
  float32 roll_rate;//翻滚角速度（单位rad/s）
  boolean roll_rate_available;//翻滚角速度有效位(true:有效/false:无效)
  boolean brake_disc_overheat; //制动盘过热(true:过热/false:未过热)
  boolean brake_disc_overheat_available; //制动盘过热有效位(true:有效/false:无效)
  boolean cdp_state;  //cdp功能激活状态(true:Active/false:Inactive)(仅支持E541)
  boolean cdp_state_available; //cdp功能激活状态有效位(true:有效/false:无效)(仅支持E541)
  boolean hba_state; //hba功能激活状态(true:Active/false:Inactive)(仅支持E541)
  boolean hba_state_available; //hba功能激活状态有效位(true:有效/false:无效)(仅支持E541)
  boolean esp_switch_state; // esp开关状态(true:开/false:关)
  boolean esp_switch_state_available; //esp开关状态有效位(true:有效/false:无效)
  boolean tpms_state;//胎压报警状态(true:Active/false:Inactive)
  boolean tpms_state_available;//胎压报警状态有效位(true:有效/false:无效)
  boolean epb_switch_state; //EPB开关状态(true:开/false:关)
  boolean epb_switch_state_available;//EPB开关状态有效位(true:有效/false:无效)
  uint8 apa_direction_intervention; //泊车驾驶员干预方向(仅支持E541)
  boolean apa_direction_intervention_available;//泊车驾驶员干预方向有效位(true:有效/false:无效)(仅支持E541)
  uint8 apa_epb_intervention; //EPB干预状态(true:Active/false:Inactive)(仅支持E541)
  boolean apa_epb_intervention_available;//EPB干预状态有效位(true:有效/false:无效)(仅支持E541)
  uint8 apa_gear_intervention; //档位干预状态(true:Active/false:Inactive)(仅支持E541)
  boolean apa_gear_intervention_available;//档位干预状态有效位(true:有效/false:无效)(仅支持E541)
  boolean trailer_state; //拖车状态(true:拖车/false:未拖车)
  boolean trailer_state_available; //拖车状态有效位(true:有效/false:无效)
  uint8 rainfall_state; //降雨量(0:无雨 1:小雨 2:中雨 3:大雨)
  boolean rainfall_state_available; //降雨量有效位(true:有效/false:无效)
  boolean pdc_err_state; //PDC系统状态(true:异常/false:正常)(仅支持E541)
  boolean pdc_err_state_available;//PDC系统状态有效位(true:正常/false:异常)(仅支持E541)
  boolean steer_motor_act_torque_available; //转向扭矩状态有效位（true：正常/false:异常）
  float32 steer_motor_act_torque; //转向扭矩 （单位：N/m）
  AntiTheftSt anti_theft_st; //车辆防盗状态
  boolean fl_door_lock_state;//左前门锁状态(false:unlock true:lock)
  boolean fr_door_lock_state;//右前门锁状态(false:unlock true:lock)
  boolean rl_door_lock_state;//左后门锁状态(false:unlock true:lock)
  boolean rr_door_lock_state;//右后门锁状态(false:unlock true:lock)
  DriverTired  driver_tired; //驾驶员疲劳状态
  boolean  driver_smoke; //驾驶员抽烟状态(false:inactive true:active)
  boolean  driver_phone; //驾驶员打电话状态(false:inactive true:active)
} _STRUCT_ALIGNED_ VehicleServiceOutputInfo;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_VEHICLE_SERVICE_H_