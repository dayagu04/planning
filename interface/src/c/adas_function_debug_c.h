// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_ADAS_FUNCTION_DEBUG_H_INCLUDE_
#define _IFLYAUTO_ADAS_FUNCTION_DEBUG_H_INCLUDE_

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
  FCWState_Unavailable = 0,
  FCWState_Off = 1,
  FCWState_Standby = 2,
  FCWState_Active = 3,
} _ENUM_PACKED_ FCWStateEnum;  // fcw_state枚举值

typedef enum {
  FCW_NoWarning = 0,
  FCW_Warning_Level1 = 1,
  FCW_Warning_Level2 = 2,
} _ENUM_PACKED_ FCWWarningLevelEnum;  // fcw_warning_level枚举值

typedef struct {
  uint8 fcw_state;                     // FCW功能状态
  uint8 fcw_warning_level;             // FCW功能报警等级
  boolean fcw_Prefill_request_status;  // Prefill请求状态  (true:Request/false:No Request)
  boolean fcw_jerk_request_status;     // Jerk请求状态     (true:Request/false:No Request)
} _STRUCT_ALIGNED_ FCWDebugOutputInfo;

typedef enum {
  PCWState_Unavailable = 0,
  PCWState_Off = 1,
  PCWState_Standby = 2,
  PCWState_Active = 3,
} _ENUM_PACKED_ PCWStateEnum;  // pcw_state枚举值

typedef enum {
  PCW_NoWarning = 0,
  PCW_Warning_Level1 = 1,
  PCW_Warning_Level2 = 2,
} _ENUM_PACKED_ PCWWarningLevelEnum;  // pcw_warning_level枚举值

typedef struct {
  uint8 pcw_state;                     // PCW功能状态
  uint8 pcw_warning_level;             // PCW功能报警等级
  boolean pcw_prefill_request_status;  // Prefill请求状态  (true:Request/false:No Request)
  boolean pcw_jerk_request_status;     // Jerk请求状态     (true:Request/false:No Request)
} _STRUCT_ALIGNED_ PCWDebugOutputInfo;

typedef enum {
  AEBState_Unavailable = 0,
  AEBState_Off = 1,
  AEBState_Standby = 2,
  AEBState_Active = 3,
} _ENUM_PACKED_ AEBStateEnum;  // aeb_state枚举值

typedef struct {
  uint8 aeb_state;                          // AEB功能状态
  boolean aeb_deceleration_request_status;  // AEB减速请求状态      (true:Request/false:No Request)
  float32 aeb_deceleration_request_value;   // AEB减速请求值 单位:m/s^2 AEB激活时,请求值为负值
  boolean aeb_hold_request_status;          // AEB停车保压请求状态  (true:Request/false:No Request)
} _STRUCT_ALIGNED_ AEBDebugOutputInfo;

typedef enum {
  AEBPState_Unavailable = 0,
  AEBPState_Off = 1,
  AEBPState_Standby = 2,
  AEBPState_Active = 3,
} _ENUM_PACKED_ AEBPStateEnum;  // aebp_state枚举值

typedef struct {
  uint8 aebp_state;                          // AEBP功能状态
  boolean aebp_deceleration_request_status;  // AEBP减速请求状态     (true:Request/false:No Request)
  float32 aebp_deceleration_request_value;   // AEBP减速请求值 单位:m/s^2 AEBP激活时,请求值为负值
  boolean aebp_hold_request_status;          // AEBP停车保压请求状态 (true:Request/false:No Request)
} _STRUCT_ALIGNED_ AEBPDebugOutputInfo;

typedef enum {
  BSDState_Unavailable = 0,
  BSDState_Off = 1,
  BSDState_Standby = 2,
  BSDState_Active = 3,
} _ENUM_PACKED_ BSDStateEnum;  // bsd_state枚举值

typedef enum {
  BSD_NoWarning = 0,
  BSD_Warning_Level1 = 1,
  BSD_Warning_Level2 = 2,
} _ENUM_PACKED_ BSDWarningLevelEnum;  // bsd_left_warning_level/bsd_right_warning_level枚举值

typedef struct {
  uint8 bsd_state;                // BSD功能状态
  uint8 bsd_left_warning_level;   // BSD左侧报警状态
  uint8 bsd_right_warning_level;  // BSD右侧报警状态
  uint8 bsd_left_fence_judge;
  uint8 bsd_right_fence_judge;
  uint8 bsd_left_fence_reprocess_judge;
  uint8 bsd_right_fence_reprocess_judge;
  uint8 bsd_left_warning_front;
  uint8 bsd_right_warning_front;
  uint8 bsd_fr_warning_code_total;
  uint8 bsd_rr_warning_code_total;
  uint8 bsd_fl_warning_code_total;
  uint8 bsd_rl_warning_code_total;
} _STRUCT_ALIGNED_ BSDDebugOutputInfo;

typedef enum {
  LCAState_Unavailable = 0,
  LCAState_Off = 1,
  LCAState_Standby = 2,
  LCAState_Active = 3,
} _ENUM_PACKED_ LCAStateEnum;  // lca_state枚举值

typedef enum {
  LCA_NoWarning = 0,
  LCA_Warning_Level1 = 1,
  LCA_Warning_Level2 = 2,
} _ENUM_PACKED_ LCAWarningLevelEnum;  // lca_left_warning_level/lca_right_warning_level枚举值

typedef struct {
  uint8 lca_state;                // LCA功能状态
  uint8 lca_left_warning_level;   // LCA左侧报警状态
  uint8 lca_right_warning_level;  // LCA右侧报警状态
  uint8 lca_left_area_car_flag;
  uint8 lca_right_area_car_flag;
} _STRUCT_ALIGNED_ LCADebugOutputInfo;

typedef enum {
  DOWState_Unavailable = 0,
  DOWState_Off = 1,
  DOWState_Standby = 2,
  DOWState_Active = 3,
} _ENUM_PACKED_ DOWStateEnum;  // dow_state枚举值

// dow_front_left_warning_level/dow_front_right_warning_level/dow_rear_left_warning_level枚举值
typedef enum {
  DOW_NoWarning = 0,
  DOW_Warning_Level1 = 1,
  DOW_Warning_Level2 = 2,
} _ENUM_PACKED_ DOWWarningLevelEnum;

typedef struct {
  uint8 dow_state;
  uint8 dow_front_left_warning_level;   // DOW前左侧报警状态
  uint8 dow_front_right_warning_level;  // DOW前右侧报警状态
  uint8 dow_rear_left_warning_level;    // DOW后左侧报警状态
  uint8 dow_rear_right_warning_level;   // DOW后右侧报警状态
  uint8 dow_rr_warning_code_total;
  uint8 dow_rl_warning_code_total;
} _STRUCT_ALIGNED_ DOWDebugOutputInfo;

typedef enum {
  RCWState_Unavailable = 0,
  RCWState_Off = 1,
  RCWState_Standby = 2,
  RCWState_Active = 3,
} _ENUM_PACKED_ RCWStateEnum;  // rcw_state枚举值

typedef enum {
  RCW_NoWarning = 0,
  RCW_Warning_Level1 = 1,
  RCW_Warning_Level2 = 2,
} _ENUM_PACKED_ RCWWarningLevelEnum;  // rcw_warning_level枚举值

typedef struct {
  uint8 rcw_state;          // RCW功能状态
  uint8 rcw_warning_level;  // RCW报警状态
  uint16 rcw_enable_code;
  uint16 rcw_disable_code;
  uint8 rcw_left_warning_flag;
  uint8 rcw_right_warning_flag;
} _STRUCT_ALIGNED_ RCWDebugOutputInfo;

typedef enum {
  FCTAState_Unavailable = 0,
  FCTAState_Off = 1,
  FCTAState_Standby = 2,
  FCTAState_Active = 3,
} _ENUM_PACKED_ FCTAStateEnum;  // fcta_state枚举值

typedef enum {
  FCTA_NoWarning = 0,
  FCTA_Warning_Level1 = 1,
  FCTA_Warning_Level2 = 2,
} _ENUM_PACKED_ FCTAWarningLevelEnum;  // fcta_left_warning_level/fcta_right_warning_level枚举值

typedef struct {
  uint8 fcta_state;                // FCTA功能状态
  uint8 fcta_left_warning_level;   // FCTA左侧报警状态
  uint8 fcta_right_warning_level;  // FCTA右侧报警状态
  uint16 fcta_enable_code;
  uint16 fcta_disable_code;
  uint16 fcta_objs_state_l1;
  uint16 fcta_objs_state_r1;
} _STRUCT_ALIGNED_ FCTADebugOutputInfo;

typedef enum {
  RCTAState_Unavailable = 0,
  RCTAState_Off = 1,
  RCTAState_Standby = 2,
  RCTAState_Active = 3,
} _ENUM_PACKED_ RCTAStateEnum;  // rcta_state枚举值

typedef enum {
  RCTA_NoWarning = 0,
  RCTA_Warning_Level1 = 1,
  RCTA_Warning_Level2 = 2,
} _ENUM_PACKED_ RCTAWarningLevelEnum;  // rcta_left_warning_level/rcta_right_warning_level枚举值

typedef struct {
  uint8 rcta_state;                // RCTA功能状态
  uint8 rcta_left_warning_level;   // RCTA左侧报警状态
  uint8 rcta_right_warning_level;  // RCTA右侧报警状态
  uint16 rcta_enable_code;
  uint16 rcta_disable_code;
  uint16 rcta_objs_state_l1;
  uint16 rcta_objs_state_r1;
} _STRUCT_ALIGNED_ RCTADebugOutputInfo;

typedef enum {
  FCTBState_Unavailable = 0,
  FCTBState_Off = 1,
  FCTBState_Standby = 2,
  FCTBState_Active = 3,
} _ENUM_PACKED_ FCTBStateEnum;  // fctb_state枚举值

typedef struct {
  uint8 fctb_state;                          // FCTB功能状态
  boolean fctb_deceleration_request_status;  // FCTB减速请求状态     (true:Request/false:No Request)
  float32 fctb_deceleration_request_value;   // FCTB减速请求值 单位：m/s^2 激活时，请求值为负值
  boolean fctb_hold_request_status;          // FCTB停车保压请求状态 (true:Request/false:No Request)
  uint16 fctb_enable_code;
  uint16 fctb_disable_code;
  uint16 fctb_brake_code;
  uint16 fctb_autohold_code;
  uint16 fctb_brake_kick_code;
  uint16 fctb_autohold_kick_code;
} _STRUCT_ALIGNED_ FCTBDebugOutputInfo;

typedef enum {
  RCTBState_Unavailable = 0,
  RCTBState_Off = 1,
  RCTBState_Standby = 2,
  RCTBState_Active = 3,
} _ENUM_PACKED_ RCTBStateEnum;  // rctb_state枚举值

typedef struct {
  uint8 rctb_state;                          // RCTB功能状态
  boolean rctb_deceleration_request_status;  // RCTB减速请求状态     (true:Request/false:No Request)
  float32 rctb_deceleration_request_value;   // RCTB减速请求值 单位：m/s^2 激活时，请求值为负值
  boolean rctb_hold_request_status;          // RCTB停车保压请求状态 (true:Request/false:No Request)
  uint16 rctb_enable_code;
  uint16 rctb_disable_code;
  uint16 rctb_brake_code;
  uint16 rctb_autohold_code;
  uint16 rctb_brake_kick_code;
  uint16 rctb_autohold_kick_code;
} _STRUCT_ALIGNED_ RCTBDebugOutputInfo;

typedef struct {
  boolean lat_ctrl_desired_angle_req_status;  // 期望方向盘转角请求状态           (true:Request/false:No Request)
  float32 lat_ctrl_desired_angle;             // 期望方向盘转角                   (deg)
  float32 lat_ctrl_desired_angle_dt;          // 经过斜率限制后的期望方向盘转速   (deg/s)
  float32 lat_ctrl_desired_angle_error;       // 期望方向盘转角误差               (deg)
  float32 lat_ctrl_desired_angle_error_dt;  // 期望方向盘转角误差速度           (deg/s)
  float32 lat_ctrl_trq_feedforward;         // 转角-扭矩:前馈项力矩             (Nm)
  float32 lat_ctrl_trq_feedback;            // 转角-扭矩:反馈项力矩             (Nm)
  float32 lat_ctrl_trq_feedback_p;          // 转角-扭矩:反馈比例项力矩         (Nm)
  float32 lat_ctrl_trq_feedback_p_gain;     // 转角-扭矩:反馈比例项系数         (Nm)
  float32 lat_ctrl_trq_feedback_i;          // 转角-扭矩:反馈积分项力矩         (Nm)
  float32 lat_ctrl_trq_feedback_i_gain;     // 转角-扭矩:反馈积分项系数        (Nm)
  float32 lat_ctrl_trq_feedback_d;          // 转角-扭矩:反馈微分项力矩         (Nm)
  boolean lat_ctrl_trq_req_status;  // 行车横向控制扭矩请求状态         (true:Request/false:No Request)
  float32 lat_ctrl_trq_req_value;   // 行车横向控制扭矩请求值           (Nm)
} _STRUCT_ALIGNED_ ControlAdaptorDebugOutputInfo;

typedef struct {
  boolean obj_vaild;
  uint32 index;
  uint8 obj_id;
  float32 obj_x;
  float32 obj_y;
  float32 obj_v_x;
  float32 obj_v_y;
  float32 obj_a_x;
  float32 obj_a_y;
  float32 obj_length;
  float32 obj_width;
  float32 obj_heading_angle;
  uint8 obj_class;
  float32 obj_ettc;
  float32 obj_turn_out_a_y;
  float32 fcw_level1_threshold;
  float32 fcw_level2_threshold;
  float32 aeb_soft_threshold;
  float32 aeb_hard_threshold;
  boolean fcw_level1_alert;
  boolean fcw_level2_alert;
  boolean aeb_soft_alert;
  boolean aeb_hard_alert;
} _STRUCT_ALIGNED_ AebObjInfo;

typedef struct {
  AebObjInfo cipv_1nd;
  AebObjInfo cipv_2nd;
  AebObjInfo left_1nd;
  AebObjInfo right_1nd;
} _STRUCT_ALIGNED_ CcrOutput;

typedef struct {
  boolean obj_vaild;
  uint32 index;
  uint8 obj_id;
  float32 obj_x;
  float32 obj_y;
  float32 obj_v_x;
  float32 obj_v_y;
  float32 obj_a_x;
  float32 obj_a_y;
  float32 obj_length;
  float32 obj_width;
  float32 obj_heading_angle;
  uint8 obj_class;
  float32 obj_ettc;
  float32 pcw_level1_threshold;
  float32 pcw_level2_threshold;
  float32 aebp_soft_threshold;
  float32 aebp_hard_threshold;
  boolean pcw_level1_alert;
  boolean pcw_level2_alert;
  boolean aebp_soft_alert;
  boolean aebp_hard_alert;
} _STRUCT_ALIGNED_ VruOutput;

typedef struct {
  uint64 ctrl_timestamp;
  uint64 fusion_road_timestamp;
  uint64 fusion_objects_timestamp;
  uint64 hmi_timestamp;
  uint64 corner_radar_fl_timestamp;
  uint64 corner_radar_fr_timestamp;
  uint64 corner_radar_rl_timestamp;
  uint64 corner_radar_rr_timestamp;
  float32 pilot_acc_to_veh;
  float32 pilot_torque_to_veh;
  float32 pilot_steer_angle_to_veh;
  float32 park_acc_to_veh;
  float32 park_steer_to_veh;
  float32 park_torque_to_veh;
} _STRUCT_ALIGNED_ EthDebugOutputInfo;

// 汇总debug的结构体
typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  FCWDebugOutputInfo fcw_output_info;
  PCWDebugOutputInfo pcw_output_info;
  AEBDebugOutputInfo aeb_output_info;
  AEBPDebugOutputInfo aebp_output_info;
  BSDDebugOutputInfo bsd_output_info;
  LCADebugOutputInfo lca_output_info;
  DOWDebugOutputInfo dow_output_info;
  RCWDebugOutputInfo rcw_output_info;
  FCTADebugOutputInfo fcta_output_info;
  RCTADebugOutputInfo rcta_output_info;
  FCTBDebugOutputInfo fctb_output_info;
  RCTBDebugOutputInfo rctb_output_info;
  ControlAdaptorDebugOutputInfo control_adaptor_debug_output_info;
  CcrOutput ccr_output_info;
  VruOutput vru_output_info;
  EthDebugOutputInfo eth_debug_output_info;
} _STRUCT_ALIGNED_ ADASFunctionDebugOutputInfo;

#pragma pack()

#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus

#endif  //_IFLYAUTO_ADAS_FUNCTION_DEBUG_H_INCLUDE_
