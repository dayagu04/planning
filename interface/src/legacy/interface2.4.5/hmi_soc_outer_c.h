// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2024/03/29

#ifndef _IFLYAUTO_LEGACY_INTERFACE2_4_5_HMI_SOC_OUTER_H_
#define _IFLYAUTO_LEGACY_INTERFACE2_4_5_HMI_SOC_OUTER_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#define HMI_LINE_NUM 8
#define HMI_OBJECT_NUM 10
#define HMI_APA_SLOT_NUM 5
#define HMI_APA_OBJ_NUM 10

#include "interface2.4.5/common_c.h"
#include "interface2.4.5/hmi_mcu_inner_c.h"
#include "interface2.4.5/planning_hmi_c.h"

#ifdef __cplusplus
namespace iflyauto {
  namespace interface_2_4_5 {
#endif

#pragma pack(4)

// 公共显示
typedef struct {
  boolean is_sharp_turn;       // 正通过大曲率弯道提醒
  boolean is_weather_abnormal; // 恶劣天气提醒
} _STRUCT_ALIGNED_ HmiCommon;

// NOA状态
typedef enum {
  NOA_UNAVAILABLE,
  NOA_OFF,
  NOA_STANDBY,
  NOA_ACTIVE,
  NOA_STANDSTILL,
  NOA_OVERRIDE_LONGITUDINAL, // 纵向超控
  NOA_OVERRIDE_LATERAL,      // 横向超控,
  NOA_SECURE,
} _ENUM_PACKED_ NoaState;

// NOA行驶中语音提醒内容
typedef enum {
  REMIND_NO_WARNNING,
  REMIND_DRIVER_VIEW_ABNORMAL,
  REMIND_DRIVER_HAND_SOFF_TIP,
  REMIND_HWP_RAMPST_FR,
  REMIND_HWP_RAMPST_FL,
  REMIND_HWP_GEOFENCEAWAY,
  REMIND_HWP_TUNNELST_AHEAD,
  REMIND_HWP_TOLLBOOTH_AHEAD,
  REMIND_HWP_ROADEND_AHEAD,
  REMIND_HWP_LANEEND_AHEAD,
  REMIND_HWP_LANEMERGE_AHEAD,
} _ENUM_PACKED_ NoaVoiceRemind;

// 接管请求等级
typedef enum {
  TAKEOVER_NO_REQUEST,
  TAKEOVER_REQ_LEVEL_MILD,
  TAKEOVER_REQ_LEVEL_MIDDLE,
  TAKEOVER_REQ_LEVEL_URGENT,
} _ENUM_PACKED_ TakeoverReqLevel;

// noa功能接管原因
typedef enum {
  TIPS_INACTIVE,
  TIPS_SYS_FAULT,
  TIPS_ROAD_NOT_SUPPORT,
  TIPS_VEH_FAULT,
  TIPS_DOOR_OR_HOOD_NOT_CLOSED,
  TIPS_SEAT_BELT_NOT_FASTENED,
  TIPS_GEAR_ERROR,
  TIPS_DISTRACTION_WARNING,
  TIPS_LMU_NOT_AVAILABLE,
  TIPS_AUTODRIVING_NOT_AVAILABLE,
  TIPS_REQUEST_DRIVER_LC_LEFT,   // 变道失败，请手动向左变道
  TIPS_REQUEST_DRIVER_LC_RIGHT,  // 变道失败，请手动向右变道
  TIPS_CAUTION_CONE,             // 锥桶接管提醒
  TIPS_DROPPED_OUT,              // 即将退出，请准备接管
  TIPS_ARRIVE_DEST,              // 即将到达终点，请准备接管
  TIPS_MERGE_LEFT_FAIL,          // 向左汇入失败，请准备接管
  TIPS_MERGE_RIGHT_FAIL,         // 向右汇入失败，请准备接管
  LANE_SPLIT_LEFT_FAIL,          // 向左入匝道失败，请准备接管
  LANE_SPLIT_RIGHT_FAIL,         // 向右入匝道失败，请准备接管
} _ENUM_PACKED_ NoaTorTips;

// 超控方式
typedef enum {
  SPEED_UP = 0,       // Speed up, 加油门
  TURN_STEERING = 1,  // Turn steering, 打方向盘
  OVER_TAKE = 2,      // Over take, 加油门+打方向盘
  NO_OVERRIDE = 3,    // No override, 无操作
} _ENUM_PACKED_ NoaOverrideMode;

// 变道提示
typedef enum {
  LAN_CHG_REMIND_NO_WARNING,
  LAN_CHG_REMIND_TURN_LEFT,
  LAN_CHG_REMIND_TURN_RIGHT,
} _ENUM_PACKED_ NoaSafeLanChgRemind;

// 车门和安全带提示
typedef enum {
  DOOR_OR_SEAT_BELT_NO_REQUEST,
  REQUEST_WHEN_FUNC_STARTED,      // 功能未启动时提示
  REQUEST_WHEN_FUNC_NOT_STARTED,  // 功能启动时提示
} _ENUM_PACKED_ DoorOrSeatBeltReq;

// 刹车提示
typedef enum {
  BREAKE_NO_REQUEST,
  HIT_BRAKE,      // 请踩刹车
  RELEASE_BRAKE,  // 请松刹车
} _ENUM_PACKED_ BrakeingReq;

// NOA进出提示
typedef enum {
  NOA_IN_OUT_REMIND_NO_DISPALY,
  NOA_IN,    // 进入
  NOA_OUT,   // 退出
} _ENUM_PACKED_ NoaInOutRemind;

// 路况提示
typedef enum {
  NONE_ROAD_CONDITION_HINT,
  TURN_LEFT_HINT,
  TURN_RIGHT_HINT,
  STRAIGHT_HINT,
  ROUNDABOUT_INTERSECTION_HINT,
  U_TURN_HINT,
  WAITING_PEDESTRIANS_PASS,
} _ENUM_PACKED_ RoadConditionHint;

// AutoDrive(ACC/SCC/NOA)不可用/退出原因
typedef enum {
  NOT_ENABLE_REASON_NO_DISPALY,
  NOT_ENABLE_OF_SYS_FAULT,
  NOT_ENABLE_OF_SHARP_TURN,
  NOT_ENABLE_OF_BAD_WEATHER,
  NOT_ENABLE_OF_CAMERA_COVRED,
  NOT_ENABLE_OF_NOT_TITE_SEATBELT,
  NOT_ENABLE_OF_DOOR_OPEN,
  NOT_ENABLE_OF_RADAR_COVERD,
  NOT_ENABLE_OF_ROAD_NOT_SUPPORT,
  NOT_ENABLE_OF_GEAR_NOT_CORRECT,
  NOT_ENABLE_OF_HD_MAP_UPGRADE,
  NOT_ENABLE_OF_HD_MAP_NOT_UP_TO_DATE,
  NOT_ENABLE_OF_HAND_OFF_PUNISHMENT,  // 脱手导致的无法进入
  NOT_ENABLE_OF_NOT_CENTERED,  // 车辆不居中，导致无法进入
  NOT_ENABLE_OF_BEND,          // 弯道导致无法激活
  NOT_ENABLE_OF_TIRE_PRESSURE, // 胎压不足导致无法激活
  NOT_ENABLE_OF_OTHER_REASON,  // 其他抑制条件导致
} _ENUM_PACKED_ ADNotEnableReason;

// NOA功能显示
typedef struct {
  float32 noa_odometer_info;        // noa工作的总里程      (公里)
  float32 noa_total_odometer_info;  // noa开启导航总里程    (公里)
  uint32 noa_hour;                  // noa使用时间，小时位  (小时)
  uint32 noa_minute;                // noa使用时间，分钟位  (分钟)
  float32 noa_maxspd;               // noa使用期间最高车速  (公里/小时)
  float32 noa_average_spd;          // noa 平均速度        (公里/小时)
  // noa 状态
  NoaState noa_state;
  // 提示用户：noa无法激活/退出原因
  ADNotEnableReason noa_not_satisfied_condition;
  // noa是否为激活状态
  boolean noa_activate_resp;  // (true: on / false: off)
  // 提示用户：行驶中语音提醒内容,提醒驾驶员安全驾驶
  NoaVoiceRemind noa_voice_remind;
  // ADU硬件故障状态
  boolean noa_adu_error;  // (true: error / false: no error)
  // 接管请求等级
  TakeoverReqLevel noa_takeover_req_lv;
  // 提示用户：noa功能接管原因
  NoaTorTips noa_tor_tips;
  // 当前环境是否禁止DCLC功能开启
  boolean noa_changelane_function_forbidden;  // (true: forbidden / false: not forbidden)
  // 驾驶员操控方式
  NoaOverrideMode noa_override_mode;
  // noa语音提示开关反馈
  NoaVoicePromptSwitch noa_voice_prompt_set_resp;
  boolean noa_cruise_dclc_swset_resp;  // noa变道确认提醒开关反馈(true: 提醒 false:不提醒)
  // 提示用户：变道语音提示
  NoaSafeLanChgRemind noa_safe_lanchg_remind;
  // 提示用户：车门提示
  DoorOrSeatBeltReq noa_close_door_req;
  // 提示用户：安全带提示
  DoorOrSeatBeltReq noa_belted_req;
  // 提示用户：刹车提示
  BrakeingReq noa_brakeing_req;
  // 提示用户：scc功能开启/退出状态
  NoaInOutRemind scc_in_out_remind;
  uint32 distance_to_destination; // 距离目的地距离
  uint32 distance_to_amp;         // 距离匝道距离
  uint32 distance_to_tunnel;      // 距离隧道距离
  uint32 distance_to_split;       // 单位是米 离前方分流的距离
  uint32 distance_to_merge;       // 单位是米 离前方汇流的距离
  boolean upgrade_from_acc;
  boolean upgrade_from_scc;
  RoadConditionHint road_condition_hint;

} _STRUCT_ALIGNED_ HmiNoaInfo;

// 车道线索引
typedef enum {
  LEFT_LINE,
  RIGHT_LINE,
  LEFT_LEFT_LINE,
  RIGHT_RIGHT_LINE,
  LEFT_LEFT_LEFT_LINE,
  RIGHT_RIGHT_RIGHT_LINE,
  LEFT_LEFT_LEFT_LEFT_LINE,
  RIGHT_RIGHT_RIGHT_RIGHT_LINE,
} _ENUM_PACKED_ LinePositionIndex;

// 车道线类型
typedef enum {
  UNKNOW_LINE_TYPE,               //（未知）   
  SINGLE_SOLID_LINE_TYPE,         //（单实线） 
  SINGLE_DASHED_LINE_TYPE,        //（单虚线) 
  DOUBLE_DASHED_SOLID_LINE_TYPE,  //（双_虚线_实线） 
  DOUBLE_SOLID_DASHED_LINE_TYPE,  //（双_实线_虚线）
  DOUBLE_DASHED_DASHED_LINE_TYPE, //（双_虚线_虚线） 
  DOUBLE_SOLID_SOLID_LINE_TYPE,   //（双_实线_实线）
  LEFT_ROADEDGE_LINE_TYPE,        //（左侧路沿）
  RIGHT_ROADEDGE_LINE_TYPE,       //（右侧路沿）
} _ENUM_PACKED_ LaneLineType;

// 车道线信息
typedef struct {
  LaneLineType l_line_marking_type;
  float32 l_line_marking_dis;          // 车道线方程常数项
  float32 l_line_marking_poly_coe_a0;  // 车道线方程一次项
  float32 l_line_marking_poly_coe_a1;  // 车道线方程二次项
  float32 l_line_marking_poly_coe_a2;  // 车道线方程三次项
  Point2f l_line_marking_start; // 车道线起点坐标 (单位m)
  Point2f l_line_marking_end;   // 车道线终点坐标
  LinePositionIndex line_index; // 车道线索引
  uint8 line_id;                // 车道线ID
  LaneLineColor line_color;     // 车道线颜色
  float32 line_width;           // 车道线宽度
} _STRUCT_ALIGNED_ HmiLineInfo;

// 障碍物位姿
typedef struct {
  float32 yaw;            // 障碍物位姿     (弧度rad)
  float32 pitch;          // 障碍物位姿     (弧度rad)
  float32 roll;           // 障碍物位姿     (弧度rad)
} _STRUCT_ALIGNED_ HmiTargetOrientation;

// 障碍物信息
typedef struct {
  float32 target_long_position;    // 障碍物纵向距离       (米)
  float32 target_lat_position;     // 障碍物横向距离       (米)
  HmiTargetOrientation  target_orientation; // 障碍物位姿
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
  Point2f apa_slot_corner_points1;  // 角点的坐标      (米) <TODO:待改为repeated>
  Point2f apa_slot_corner_points2;
  Point2f apa_slot_corner_points3;
  Point2f apa_slot_corner_points4;
} _STRUCT_ALIGNED_ HmiApaSlotInfo;

// 当前泊车状态
typedef enum {
  APA_FAILURE,
  APA_INIT,
  APA_STANDBY,
  APA_ERROR,
  APA_SEARCHING,
  APA_READY,
  APA_NO_READY,
  APA_ACTIVE_WAIT,
  APA_ACTIVE_CONTROL,
  APA_SUSPEND_ACTIVATE,
  APA_SUSPEND_CLOSE,
  APA_SECURE,
  APA_COMPLETED_SUCESSFULLY,
} _ENUM_PACKED_ ApaStatus;

// AVM的当前工作状态
typedef enum {
  APA_AVM_STANDBY,
  AVM_ON,
  APA_ON,
  ACC_ON_APA_UNAVAIABLE,  // acc on, apa临时不可用
} _ENUM_PACKED_ ApaAvmStatus;

// 提示用户：完成操作
typedef enum {
  INDICATION_NO_REQUEST,
  INDICATION_PARKING_GEAR,  // 请挂P档，激活电子手刹
  INDICATION_APP_CONNECT,   // 请打开手机APP，连接车辆
  INDICATION_GET_OFF,       // 连接成功
} _ENUM_PACKED_ ApaDriverOperationIndication;

// 蜂鸣提示音频率
typedef enum {
  BEEPRATE_1HZ,
  BEEPRATE_2HZ,
  BEEPRATE_4HZ,
  BEEPRATE_WARNING_ON,
} _ENUM_PACKED_ ApaPdcAudibleBeeprate;

// 用户选择：是否确定选择此车位
typedef enum {
  APA_MENU_NO_REQUEST,
  APA_MENU_CANCEL,
  APA_MENU_CONFIRMATION,
} _ENUM_PACKED_ ApaMenu;

// 提示用户：泊车不可用原因
typedef enum {
  APA_AVAILABILITY,
  APA_INAVAILABILITY_OF_VEHICLE_NOT_CRANKING,  // 车辆未启动
  APA_INAVAILABILITY_OF_HIGH_VEHSPD,           // 当前车速过高
  APA_INAVAILABILITY_OF_ACC_OPEN,
} _ENUM_PACKED_ ApaAvaliableState;

// 提示用户：泊出不可用原因
typedef enum {
  APA_PARKOUT_NO_REQUEST,
  APA_PARKOUT_SMOOTH_ROAD,
  APA_PARKOUT_PARK_SPACE_LIMIT,
} _ENUM_PACKED_ ApaParkoutCondition;

// 方向灯状态
typedef enum {
  TURNLIGHT_UNACTIVATED,
  LEFTTURNLIGHT_ACTIVATED,
  RIGHTTURNLIGHT_ACTIVATED,
  HAZARDLIGHT_ACTIVATED,
} _ENUM_PACKED_ ApaIcmTurnlightFb;

// APA请求的目标档位
typedef enum {
  TargetGear_NO_REQUEST,
  TargetGear_P,
  TargetGear_R,
  TargetGear_N,
  TargetGear_D,
} _ENUM_PACKED_ TargetGearReq;

// APA功能信息
typedef struct {
  uint32 apa_ble_current_cnt;  // 当前泊车次数
  uint32 apa_ble_total_cnt;    // 总计泊车次数
  // 当前泊车模式
  ApaParkingDirection apa_mode_status;
  // 当前泊车状态
  ApaStatus apa_status;
  // AVM的当前工作状态
  ApaAvmStatus apa_avm_status;
  // 提示用户：车门提示
  DoorOrSeatBeltReq apa_close_door_req;
  // 提示用户：安全带提示
  DoorOrSeatBeltReq apa_belted_req;
  // 提示用户：刹车提示
  BrakeingReq apa_brakeing_req;
  // APA请求声音提醒
  boolean apa_warning_req;  // (true: warning / false: no warning)
  // 请用户选择：泊入还是泊出
  boolean apa_park_demand;  // (true: request / false: no request)
  // 请用户选择：选择泊出方向
  boolean apa_parkoutdir_indication;  // (true: request / false: no request)
  // 提示用户：移开障碍物后，再继续泊
  boolean apa_move_object_indication;  // (true: request / false: no request)
  // 提示用户：完成操作
  ApaDriverOperationIndication apa_driver_operation_indication;
  // 提示用户：踩住刹车，退P档，松手刹
  boolean apa_parkout;  // (true: request / false: no request)
  // 提示用户：泊车超时，自动泊车退出
  boolean apa_parktimeout;  // (true: request / false: no request)
  // 蜂鸣提示音频率
  ApaPdcAudibleBeeprate apa_pdc_audible_beeprate;
  // 请用户选择：是否开始泊车
  boolean apa_start;  // (true: request / false: no request)
  // 请用户选择：是否确定选择此车位
  ApaMenu apa_menu;
  // 提示用户：泊车不可用原因
  ApaAvaliableState apa_avaliable_state;
  // 提示用户：泊出不可用原因
  ApaParkoutCondition apa_parkout_condition;
  // 泊车过程中档杆是否参与
  boolean apa_acm_lever_intervention;  // (true: intervention / false: not intervention)
  // 显示方向灯状态
  ApaIcmTurnlightFb apa_icm_turnlight_fb;
  // 显示APA请求的目标档位
  TargetGearReq apa_targear_req;
  int32 apa_parkspace_number;                      // 车位数量
  HmiApaSlotInfo apa_slot_info[HMI_APA_SLOT_NUM];  // apa界面显示车位线信息    <最大5个>
  int32 apa_obj_number;
  HmiObjInfo apa_obj_info[HMI_APA_OBJ_NUM];  // apa界面显示障碍物信息 <最大10个>
} _STRUCT_ALIGNED_ HmiApaInfo;

// acc功能开启/退出状态
typedef enum {
  ACC_IN_OUT_REMIND_NO_DISPALY,
  ACC_IN,   // acc已开启
  ACC_OUT,  // acc已退出
} _ENUM_PACKED_ AccInOutRemind;

// ACC状态
typedef enum {
  ACC_UNAVAILABLE,
  ACC_OFF,
  ACC_STANDBY,
  ACC_ACTIVE,
  ACC_STAND_ACTIVE,
  ACC_STAND_WAIT,
  ACC_OVERRIDE,
  ACC_SECURE,
} _ENUM_PACKED_ AccStatus;

// ACC功能信息
typedef struct {
  // 提示用户：ACC不可用/退出原因
  ADNotEnableReason acc_notenable_reason;
  // 提示用户：acc功能开启/退出状态
  AccInOutRemind acc_in_out_remind;
  // 状态机状态
  AccStatus acc_status;
  boolean acc_active_resp;         // acc是否为激活状态            (true:active / false:not active)
  boolean acc_driver_denied;       // 提示用户：激活失败，原因见AccSccNotEnableReason
  TakeoverReqLevel acc_takeover_req_lv;  // 提示用户：接管
  boolean front_car_starts;        // 提示用户：前车已起步
  boolean acc_go_indicator;        // 提示用户即将起步
  boolean acc_driver_go;           // 请用户选择：是否需要跟车起步  (true:request / false:no request)
  int32 acc_set_headway;           // driver selected follow headway
  float32 acc_set_speed;           // 巡航速度                 (公里/小时)
  boolean intelligent_following;   // 是否处于智慧跟车状态
  boolean too_close_to_front;      // 距离前车过近提示
  // 提示用户：车门提示
  DoorOrSeatBeltReq acc_close_door_req;
  // 提示用户：安全带提示
  DoorOrSeatBeltReq acc_belted_req;
  // 提示用户：刹车提示
  BrakeingReq acc_brakeing_req;
  boolean downgrade_from_scc;
  boolean downgrade_from_noa;

} _STRUCT_ALIGNED_ HmiAccInfo;

// SCC状态
typedef enum {
  SCC_UNAVAILABLE,
  SCC_OFF,
  SCC_STANDBY,
  SCC_ACTIVE,
  SCC_STAND_ACTIVE,
  SCC_STAND_WAIT,
  SCC_OVERRIDE_LONGITUDINAL, // 纵向超控
  SCC_OVERRIDE_LATERAL,      // 横向超控
  SCC_OVERRIDE,              // 横向+纵向
  SCC_SECURE,
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
  NEITHER_SIDE_LANE_MARK_DETECTED,
  LEFT_SIDE_LANE_MARK_DETECTED,
  RIGHT_SIDE_LANE_MARK_DETECTED,
  BOTH_SIDE_LANE_MARKS_DETECTED,
} _ENUM_PACKED_ SccLineDetectStatus;

// scc功能开启/退出状态
typedef enum {
  SCC_IN_OUT_REMIND_NO_DISPALY,
  SCC_IN,                               // scc已开启
  SCC_OUT,                              // scc已退出
  SCC_INT_BUT_LATERAL_CONTROL_FAILURE,  // scc开启横向不可用
} _ENUM_PACKED_ SccInOutRemind;

// 变道状态
typedef enum {
  NONE_LANE_CHANGE,
  UNAVAILABLE_LANE_CHANGE,
  CANCEL_LANE_CHANGE,
  SUCCESS_LANE_CHANGE,
  FAIL_LANE_CHANGE,
  REQUEST_LANE_CHANGE, // 请求状态，需用户确认
  WAITING_LANE_CHANGE,
  ONGOING_LANE_CHANGE,
  SOLID_INHIBITED_LANE_CHANGE,    // 实线抑制
  OBSTACLE_INHIBITED_LANE_CHANGE, // 障碍物抑制
} _ENUM_PACKED_ LC_STATUS;

// 变道方向
typedef enum {
  NONE_LC_DIRECTION,
  LEFT_LANE_CHANGE,
  RIGHT_LANE_CHANGE,
} _ENUM_PACKED_ LC_DIRECTION;

// 变道原因
typedef enum {
  NONE_LC_REASON,
  DRIVER_COMMAND,
  PRECEEDING_VEHICLE_SLOW,
  CONSTRUCTION_ZONE,
  OBSTACLE,
  DRIVE_OUT_OF_OVERTAKING_LANE,
  EVASION_CONE,
  NAVIGATION,
  FAULT_REMINDER,
  LANE_BLOCK,
  ENTER_MAINROAD,
  ENTER_RAMP,
  AVOID_RIGHTMOST_RAMP_ENTRANCE,
  TRAFFIC_JAM_AHEAD,
  HIGH_EFFICIENCY,
  LANE_SPLIT,
  LANE_MERGE,
} _ENUM_PACKED_ LC_REASON;

// 窄路提醒
typedef enum {
  NONE_NARROW_ROAD_TIPS,
  NARROW_ROAD_CAN_PASS,
  NARROW_ROAD_CANNOT_PASS,
} _ENUM_PACKED_ NarrowRoadTips;

// 变道信息
typedef struct {
  LC_STATUS lc_status; // 变道状态
  LC_DIRECTION lc_direction;
  LC_REASON lc_reason;
  int32 obstacle_id; // 当变道受抑制时，干扰的障碍物ID
} _STRUCT_ALIGNED_ HmiLaneChange;

// 智慧躲闪的内容
typedef enum {
  NONE_DODGE,     // 未闪躲
  AVOID_OVERSIZED_VEHICLE, // 躲避大车
  AVOID_OBSTACLE, // 躲避障碍物
} _ENUM_PACKED_ DodgeType;

// 智慧躲闪信息
typedef struct {
  DodgeType dodge_type;
  int32 object_id;
} _STRUCT_ALIGNED_ HmiIntelligentEvasion;

// SCC功能信息
typedef struct {
  // SCC状态
  SccStatus scc_status;
  boolean scc_active_resp;       // scc主功能反馈               (true:active / false:not active)
  boolean scc_driver_denied;     // 提示用户：激活失败
  // 脱手告警等级，提示用户：手脱离方向盘
  SccHandsOffWarn scc_hands_off_warning;
  TakeoverReqLevel scc_takeover_req_lv;  // 提示用户：接管
  // 车道线检测状态
  SccLineDetectStatus scc_line_detect_status;
  // 提示用户：车门提示
  DoorOrSeatBeltReq scc_close_door_req;
  // 提示用户：安全带提示
  DoorOrSeatBeltReq scc_belted_req;
  // 提示用户：刹车提示
  BrakeingReq scc_brakeing_req;
  // 提示用户：SCC不可用/退出原因
  ADNotEnableReason scc_notenable_reason;
  // 提示用户：scc功能开启/退出状态
  SccInOutRemind scc_in_out_remind;

  boolean upgrade_from_acc;
  boolean downgrade_from_noa;

  HmiIntelligentEvasion intelligent_evasion; // 智慧躲闪
  HmiLaneChange lane_change; // 变道信息
  NarrowRoadTips narrow_road_Tips;
} _STRUCT_ALIGNED_ HmiSccInfo;

// 交通信号图标
typedef enum {
  NONE_TRAFFIC_SIGN,
  NO_OVERTAKING_SIGN, 
  END_OF_NO_OVERTAKING_SIGN,
  PROHIBITING_TURNING_LEFT_SIGN,
  PROHIBITING_TURNING_RIGHT_SIGN, 
  PROHIBITING_U_TURNING_SIGN, 
  NO_ENTRY_SIGN,
  NO_PASSING_SIGN,
  PROHIBITING_MOTOR_VEHICLE_ENTERING_SIGN, 
  PROHIBITING_STANDING_AND_PARKING_SIGN, 
  PROHIBITING_LONG_PARKING_SIGN,
  STOP_SIGN_SIGN, 
  YIELD_SIGN_SIGN,
  MOTORWAY_START_SIGN,
  MOTORWAY_END_SIGN,
} _ENUM_PACKED_ TrafficSignType;

// 红绿灯状态
typedef enum {
  NONE_TRAFFIC_LIGHT_TYPE,
  LEFT_TURN_TRAFFIC_LIGHT, 
  STRAIGHT_TRAFFIC_LIGHT, 
  RIGHT_TURN_TRAFFIC_LIGHT, 
  U_TURN_TRAFFIC_LIGHT,
  UNABLE_RECOGNIZE_TRAFFIC_LIGHT, // 无法识别
} _ENUM_PACKED_ HmiTrafficLightType;
// 红绿灯颜色状态
typedef enum {
  NONE_TRAFFIC_LIGHT_COLOR, 
  GREEN_TRAFFIC_LIGHT,
  GREEN_FLASHING_TRAFFIC_LIGHT, 
  YELLOW_TRAFFIC_LIGHT,
  YELLOW_FLASHING_TRAFFIC_LIGHT, 
  RED_TRAFFIC_LIGHT,
  RED_FLASHING_TRAFFIC_LIGHT, 
} _ENUM_PACKED_ HmiTrafficLightColor;
// 红绿灯信息
typedef struct {
  HmiTrafficLightType  traffic_light_type;
  HmiTrafficLightColor traffic_light_color;
  int32 traffic_light_countdown_number;
} _STRUCT_ALIGNED_ HmiTrafficLight;

// LDW、ELK功能信息。含义参考planning_hmi.proto
typedef struct {
  LDWOutputInfoStr ldw_output_info;
  LDPOutputInfoStr ldp_output_info;
  ELKOutputInfoStr elk_output_info;
  TSROutputInfoStr tsr_output_info;
  IHCOutputInfoStr ihc_output_info;
  ALCOutputInfoStr alc_output_info;
  TrafficSignType traffic_sign;
  HmiTrafficLight traffic_light;
} _STRUCT_ALIGNED_ HmiOtherInfo;

// HMI输出信息

typedef enum {
  HMI_CALIB_NO_MESSAGE = 0,    // 无消息
  HMI_AVM_RELOAD_SUCCESS = 1,  // AVM加载新标定成功
  HMI_AVM_ERROR = 2,           // AVM模块错误
  HMI_CALIB_SUCCESS = 3,       // 标定成功
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
  HMI_CALIB_ERROR_BAD_ROLL_RESULT = 16,
  HMI_CALIB_ERROR_BAD_PITCH_RESULT = 17,
  HMI_CALIB_ERROR_BAD_YAW_RESULT = 18,
  HMI_CALIB_ERROR_BAD_X_RESULT = 19,
  HMI_CALIB_ERROR_BAD_Y_RESULT = 20,
  HMI_CALIB_ERROR_BAD_Z_RESULT = 21,
  HMI_CALIB_TIMEOUT = 22,
  HMI_CALIB_TERMINATE_ABNORMALLY = 23,
  HMI_CALIB_UNKNOWN_ERROR = 24,
  HMI_CALIB_INIT_ERROR = 25,
  HMI_CALIB_IGNORED = 26,
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

typedef struct {
  Header header;            // 头信息
  HmiCommon hmi_common;     // 通用信息
  uint8 hmi_line_info_size;
  HmiLineInfo hmi_line_info[HMI_LINE_NUM];  // 车道线信息
  uint8 hmi_obj_info_size;
  HmiObjInfo hmi_obj_info[HMI_OBJECT_NUM];  // 障碍物信息
  HmiObjInfo hmi_cipv_info;                 // CIPV信息
  HmiApaInfo hmi_apa_info;                  // 泊车信息
  HmiAccInfo hmi_acc_info;                  // acc信息
  HmiSccInfo hmi_scc_info;                  // scc信息
  HmiNoaInfo hmi_noa_info;                  // noa信息
  HmiOtherInfo hmi_other_info;              // adas信息
  HmiCalibInfo calib_info;                  // 标定信息
  HmiSensorInfo sensor_info;                // 传感器故障/遮挡信息
} _STRUCT_ALIGNED_ HmiSocOuter;

#pragma pack()
#ifdef __cplusplus
  }  // namespace interface_2_4_5
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_LEGACY_INTERFACE2_4_5_HMI_SOC_OUTER_H_